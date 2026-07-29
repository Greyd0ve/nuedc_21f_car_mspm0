#include "app_h26_task3.h"
#include "app_ball_link.h"
#include "app_h26_config.h"
#include "RodStepper.h"
#include <stdint.h>

static volatile H26_Task3State_t s_state = H26_T3_IDLE;
static volatile uint32_t s_startMs = 0U;
static volatile uint32_t s_finalElapsedMs = 0U;
static volatile uint16_t s_acquireHoldMs = 0U;
static volatile uint16_t s_targetHoldMs = 0U;
static volatile float s_positionCm = 0.0f;
static volatile float s_ballSpeedCmps = 0.0f;
static volatile float s_targetCm = 0.0f;
static volatile int32_t s_commandHz = 0;
static volatile uint8_t s_visionValid = 0U;
static volatile uint8_t s_confidence = 0U;
static volatile uint32_t s_frameAgeMs = 0xFFFFFFFFUL;
static uint16_t s_lastSequence = 0U;
static uint32_t s_lastMeasurementMs = 0U;
static float s_lastPositionCm = 0.0f;
static uint8_t s_hasMeasurement = 0U;

static float H26_T3_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t H26_T3_AbsInt32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint16_t H26_T3_AddMs(uint16_t value)
{
    if (value > (uint16_t)(0xFFFFU - CAR_CONTROL_PERIOD_MS))
    {
        return 0xFFFFU;
    }
    return (uint16_t)(value + CAR_CONTROL_PERIOD_MS);
}

static int32_t H26_T3_ClampInt32(int32_t value, int32_t lower, int32_t upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

static int32_t H26_T3_SlewHz(int32_t current, int32_t target)
{
    if (current < target)
    {
        current += (int32_t)H26_T3_COMMAND_SLEW_HZ_PER_TICK;
        return (current > target) ? target : current;
    }
    if (current > target)
    {
        current -= (int32_t)H26_T3_COMMAND_SLEW_HZ_PER_TICK;
        return (current < target) ? target : current;
    }
    return current;
}

static void H26_T3_StopCommand(void)
{
    RodStepper_Stop();
    s_commandHz = 0;
}

static uint8_t H26_T3_UpdateMeasurement(uint32_t nowMs)
{
    BallLinkFrame_t frame;
    float position;

    s_visionValid = 0U;
    s_frameAgeMs = App_BallLink_GetFrameAgeMs(nowMs);
    if (App_BallLink_GetLatest(&frame) == 0U ||
        frame.ballValid == 0U || frame.pipeValid == 0U ||
        frame.confidence < H26_T3_MIN_CONFIDENCE ||
        frame.positionCentiCm > H26_T3_PIPE_LENGTH_CENTICM ||
        s_frameAgeMs > H26_T3_MAX_FRAME_AGE_MS)
    {
        return 0U;
    }

    position = (float)frame.positionCentiCm * 0.01f;
    if (s_hasMeasurement == 0U || frame.sequence != s_lastSequence)
    {
        if (s_hasMeasurement != 0U)
        {
            uint32_t dtMs = frame.receiveTimeMs - s_lastMeasurementMs;
            if (dtMs >= H26_T3_MIN_SPEED_DT_MS && dtMs <= H26_T3_MAX_SPEED_DT_MS)
            {
                float rawSpeed = (position - s_lastPositionCm) * 1000.0f / (float)dtMs;
                s_ballSpeedCmps = H26_T3_SPEED_FILTER_ALPHA * rawSpeed +
                    (1.0f - H26_T3_SPEED_FILTER_ALPHA) * s_ballSpeedCmps;
            }
        }
        s_lastSequence = frame.sequence;
        s_lastMeasurementMs = frame.receiveTimeMs;
        s_lastPositionCm = position;
        s_hasMeasurement = 1U;
    }

    s_positionCm = position;
    s_confidence = frame.confidence;
    s_visionValid = 1U;
    return 1U;
}

static void H26_T3_ApplyVelocity(int32_t signedHz)
{
    uint32_t magnitude = (uint32_t)H26_T3_AbsInt32(signedHz);

    if (magnitude == 0U)
    {
        H26_T3_StopCommand();
    }
    else if (signedHz > 0)
    {
        (void)RodStepper_SetVelocity(ROD_STEPPER_DIR_POSITIVE, magnitude);
    }
    else
    {
        (void)RodStepper_SetVelocity(ROD_STEPPER_DIR_NEGATIVE, magnitude);
    }
}

static uint8_t H26_T3_ApplyBallControl(uint32_t nowMs, float targetCm)
{
    float error;
    float command;
    int32_t targetHz;

    if (H26_T3_UpdateMeasurement(nowMs) == 0U)
    {
        H26_T3_StopCommand();
        return 0U;
    }

    error = targetCm - s_positionCm;
    command = H26_T3_POSITION_KP_HZ_PER_CM * error -
        H26_T3_SPEED_KD_HZ_PER_CMPS * s_ballSpeedCmps;
    if (H26_T3_STEPPER_SIGN_FOR_POSITIVE_BALL < 0)
    {
        command = -command;
    }

    targetHz = (int32_t)command;
    targetHz = H26_T3_ClampInt32(targetHz,
        -(int32_t)H26_T3_MAX_COMMAND_HZ,
        (int32_t)H26_T3_MAX_COMMAND_HZ);
    if (H26_T3_AbsFloat(error) <= H26_T3_COMMAND_DEADBAND_CM)
    {
        targetHz = 0;
    }
    else if (H26_T3_AbsInt32(targetHz) < (int32_t)H26_T3_MIN_COMMAND_HZ)
    {
        targetHz = (targetHz >= 0) ? (int32_t)H26_T3_MIN_COMMAND_HZ :
            -(int32_t)H26_T3_MIN_COMMAND_HZ;
    }

    s_commandHz = H26_T3_SlewHz(s_commandHz, targetHz);
    H26_T3_ApplyVelocity(s_commandHz);
    return 1U;
}

static uint8_t H26_T3_IsTargetStable(float targetCm)
{
    return (H26_T3_AbsFloat(s_positionCm - targetCm) <= H26_T3_TARGET_TOLERANCE_CM &&
            H26_T3_AbsFloat(s_ballSpeedCmps) <= H26_T3_STABLE_SPEED_CMPS) ? 1U : 0U;
}

void H26_Task3_Init(void)
{
    H26_Task3_Reset();
}

void H26_Task3_Reset(void)
{
    H26_T3_StopCommand();
    s_state = H26_T3_IDLE;
    s_startMs = 0U;
    s_finalElapsedMs = 0U;
    s_acquireHoldMs = 0U;
    s_targetHoldMs = 0U;
    s_positionCm = 0.0f;
    s_ballSpeedCmps = 0.0f;
    s_targetCm = H26_T3_CENTER_CM;
    s_visionValid = 0U;
    s_confidence = 0U;
    s_frameAgeMs = 0xFFFFFFFFUL;
    s_lastSequence = 0U;
    s_lastMeasurementMs = 0U;
    s_lastPositionCm = 0.0f;
    s_hasMeasurement = 0U;
}

void H26_Task3_Start(uint32_t startMs)
{
    H26_Task3_Reset();
    App_BallLink_Reset();
    s_startMs = startMs;
    s_targetCm = H26_T3_CENTER_CM;
    s_state = H26_T3_ACQUIRE_O;
}

void H26_Task3_ForceFault(void)
{
    H26_T3_StopCommand();
    s_state = H26_T3_FAULT;
}

H26_Task3Result_t H26_Task3_Task10ms(uint32_t nowMs)
{
    uint32_t elapsedMs = nowMs - s_startMs;

    if ((s_state == H26_T3_ACQUIRE_O || s_state == H26_T3_MOVE_PLUS_5 ||
         s_state == H26_T3_HOLD_PLUS_5 || s_state == H26_T3_MOVE_MINUS_5 ||
         s_state == H26_T3_HOLD_MINUS_5) && elapsedMs >= H26_T3_MAX_RUN_TIME_MS)
    {
        H26_Task3_ForceFault();
        return H26_T3_RESULT_FAULT;
    }

    switch (s_state)
    {
    case H26_T3_ACQUIRE_O:
        H26_T3_StopCommand();
        if (H26_T3_UpdateMeasurement(nowMs) != 0U &&
            H26_T3_AbsFloat(s_positionCm - H26_T3_CENTER_CM) <= H26_T3_START_O_TOLERANCE_CM)
        {
            s_acquireHoldMs = H26_T3_AddMs(s_acquireHoldMs);
            if (s_acquireHoldMs >= H26_T3_ACQUIRE_HOLD_MS)
            {
                s_targetCm = H26_T3_CENTER_CM + H26_T3_OFFSET_CM;
                s_state = H26_T3_MOVE_PLUS_5;
            }
        }
        else
        {
            s_acquireHoldMs = 0U;
            if (elapsedMs >= H26_T3_ACQUIRE_TIMEOUT_MS)
            {
                H26_Task3_ForceFault();
                return H26_T3_RESULT_FAULT;
            }
        }
        break;

    case H26_T3_MOVE_PLUS_5:
        if (H26_T3_ApplyBallControl(nowMs, s_targetCm) == 0U)
        {
            H26_Task3_ForceFault();
            return H26_T3_RESULT_FAULT;
        }
        if (H26_T3_IsTargetStable(s_targetCm) != 0U)
        {
            s_targetHoldMs = 0U;
            s_state = H26_T3_HOLD_PLUS_5;
        }
        break;

    case H26_T3_HOLD_PLUS_5:
        if (H26_T3_ApplyBallControl(nowMs, s_targetCm) == 0U)
        {
            H26_Task3_ForceFault();
            return H26_T3_RESULT_FAULT;
        }
        if (H26_T3_IsTargetStable(s_targetCm) != 0U)
        {
            s_targetHoldMs = H26_T3_AddMs(s_targetHoldMs);
            if (s_targetHoldMs >= H26_T3_TARGET_HOLD_MS)
            {
                s_targetCm = H26_T3_CENTER_CM - H26_T3_OFFSET_CM;
                s_targetHoldMs = 0U;
                s_state = H26_T3_MOVE_MINUS_5;
            }
        }
        else
        {
            s_targetHoldMs = 0U;
            s_state = H26_T3_MOVE_PLUS_5;
        }
        break;

    case H26_T3_MOVE_MINUS_5:
        if (H26_T3_ApplyBallControl(nowMs, s_targetCm) == 0U)
        {
            H26_Task3_ForceFault();
            return H26_T3_RESULT_FAULT;
        }
        if (H26_T3_IsTargetStable(s_targetCm) != 0U)
        {
            s_targetHoldMs = 0U;
            s_state = H26_T3_HOLD_MINUS_5;
        }
        break;

    case H26_T3_HOLD_MINUS_5:
        if (H26_T3_ApplyBallControl(nowMs, s_targetCm) == 0U)
        {
            H26_Task3_ForceFault();
            return H26_T3_RESULT_FAULT;
        }
        if (H26_T3_IsTargetStable(s_targetCm) != 0U)
        {
            s_targetHoldMs = H26_T3_AddMs(s_targetHoldMs);
            if (s_targetHoldMs >= H26_T3_TARGET_HOLD_MS)
            {
                H26_T3_StopCommand();
                s_finalElapsedMs = elapsedMs;
                s_state = H26_T3_DONE;
                return H26_T3_RESULT_FINISHED;
            }
        }
        else
        {
            s_targetHoldMs = 0U;
            s_state = H26_T3_MOVE_MINUS_5;
        }
        break;

    case H26_T3_DONE:
        H26_T3_StopCommand();
        return H26_T3_RESULT_FINISHED;

    case H26_T3_FAULT:
        H26_T3_StopCommand();
        return H26_T3_RESULT_FAULT;

    case H26_T3_IDLE:
    default:
        H26_Task3_ForceFault();
        return H26_T3_RESULT_FAULT;
    }

    return H26_T3_RESULT_RUNNING;
}

H26_Task3State_t H26_Task3_GetState(void) { return s_state; }

uint32_t H26_Task3_GetElapsedMs(uint32_t nowMs)
{
    if (s_state == H26_T3_IDLE) return 0U;
    if (s_state == H26_T3_DONE) return s_finalElapsedMs;
    return nowMs - s_startMs;
}

uint32_t H26_Task3_GetFinalElapsedMs(void) { return s_finalElapsedMs; }
float H26_Task3_GetPositionCm(void) { return s_positionCm; }
float H26_Task3_GetBallSpeedCmps(void) { return s_ballSpeedCmps; }
float H26_Task3_GetTargetCm(void) { return s_targetCm; }
int32_t H26_Task3_GetCommandHz(void) { return s_commandHz; }
uint8_t H26_Task3_IsVisionValid(void) { return s_visionValid; }
uint8_t H26_Task3_GetConfidence(void) { return s_confidence; }
uint32_t H26_Task3_GetFrameAgeMs(void) { return s_frameAgeMs; }
