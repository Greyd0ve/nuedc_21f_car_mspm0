#include "app_h26_task3.h"
#include "app_ball_link.h"
#include "app_h26_config.h"
#include "RodEncoder.h"
#include "RodStepper.h"
#include <stdint.h>

typedef enum
{
    H26_T3_VISION_NEW = 0,
    H26_T3_VISION_HELD,
    H26_T3_VISION_FAULT
} H26_Task3VisionStatus_t;

static volatile H26_Task3State_t s_state = H26_T3_IDLE;
static volatile H26_Task3Fault_t s_fault = H26_T3_FAULT_NONE;
static volatile uint32_t s_startMs = 0U;
static volatile uint32_t s_finalElapsedMs = 0U;
static volatile uint16_t s_acquireHoldMs = 0U;
static volatile uint16_t s_targetHoldMs = 0U;
static volatile uint16_t s_rawPositionCentiCm = 0U;
static volatile float s_positionCm = 0.0f;
static volatile float s_ballSpeedCmps = 0.0f;
static volatile float s_targetCm = 0.0f;
static volatile float s_errorCm = 0.0f;
static volatile float s_plusHoldPeakErrorCm = 0.0f;
static volatile float s_minusHoldPeakErrorCm = 0.0f;
static volatile int32_t s_commandHz = 0;
static volatile uint8_t s_visionValid = 0U;
static volatile uint8_t s_confidence = 0U;
static volatile uint8_t s_lastFlags = 0U;
static volatile uint32_t s_frameAgeMs = 0xFFFFFFFFUL;
static volatile uint16_t s_lastSequence = 0U;
static volatile int32_t s_rodEncoderCount = 0;
static volatile uint8_t s_rodSoftLimitActive = 0U;

static uint16_t s_lastPacketSequence = 0U;
static uint32_t s_lastReliableVisionMs = 0U;
static uint32_t s_lastMeasurementMs = 0U;
static uint32_t s_lastRodMotionMs = 0U;
static float s_lastPositionCm = 0.0f;
static int32_t s_lastRodEncoderCount = 0;
static uint16_t s_newMeasurementDtMs = 0U;
static uint8_t s_hasPacket = 0U;
static uint8_t s_hasReliableMeasurement = 0U;
static H26_Task3Fault_t s_visionIssue = H26_T3_FAULT_NONE;
static uint8_t s_stepperActive = 0U;
static RodStepperDirection_t s_appliedDirection = ROD_STEPPER_DIR_NEGATIVE;
static uint8_t s_directionGuardActive = 0U;
static uint32_t s_directionGuardStartMs = 0U;

static float H26_T3_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint32_t H26_T3_AbsInt32ToU32(int32_t value)
{
    return (value < 0) ? ((uint32_t)(-(value + 1)) + 1U) : (uint32_t)value;
}

static int8_t H26_T3_SignInt32(int32_t value)
{
    if (value > 0)
    {
        return 1;
    }
    if (value < 0)
    {
        return -1;
    }
    return 0;
}

static uint16_t H26_T3_AddMs(uint16_t value, uint16_t deltaMs)
{
    if (value > (uint16_t)(0xFFFFU - deltaMs))
    {
        return 0xFFFFU;
    }
    return (uint16_t)(value + deltaMs);
}

static int32_t H26_T3_ClampInt32(int32_t value, int32_t lower, int32_t upper)
{
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}

static int32_t H26_T3_SlewHz(int32_t current, int32_t target)
{
    int32_t step = (int32_t)H26_T3_COMMAND_SLEW_HZ_PER_TICK;

    if (current < target)
    {
        current += step;
        return (current > target) ? target : current;
    }
    if (current > target)
    {
        current -= step;
        return (current < target) ? target : current;
    }
    return current;
}

static uint8_t H26_T3_CalibrationIsValid(void)
{
    int32_t posDelta = (int32_t)H26_T3_RAW_POS5_CENTICM -
        (int32_t)H26_T3_RAW_O_CENTICM;
    int32_t negDelta = (int32_t)H26_T3_RAW_NEG5_CENTICM -
        (int32_t)H26_T3_RAW_O_CENTICM;

    if (H26_T3_RAW_O_CENTICM < H26_T3_RAW_MIN_CENTICM ||
        H26_T3_RAW_O_CENTICM > H26_T3_RAW_MAX_CENTICM ||
        H26_T3_RAW_POS5_CENTICM < H26_T3_RAW_MIN_CENTICM ||
        H26_T3_RAW_POS5_CENTICM > H26_T3_RAW_MAX_CENTICM ||
        H26_T3_RAW_NEG5_CENTICM < H26_T3_RAW_MIN_CENTICM ||
        H26_T3_RAW_NEG5_CENTICM > H26_T3_RAW_MAX_CENTICM ||
        posDelta == 0 || negDelta == 0)
    {
        return 0U;
    }

    return ((posDelta > 0 && negDelta < 0) ||
            (posDelta < 0 && negDelta > 0)) ? 1U : 0U;
}

static uint8_t H26_T3_CalibratePositionCm(uint16_t rawCentiCm, float *positionCm)
{
    int32_t delta;
    int32_t posDelta;
    int32_t negDelta;

    if (positionCm == 0 || rawCentiCm < H26_T3_RAW_MIN_CENTICM ||
        rawCentiCm > H26_T3_RAW_MAX_CENTICM || H26_T3_CalibrationIsValid() == 0U)
    {
        return 0U;
    }

    delta = (int32_t)rawCentiCm - (int32_t)H26_T3_RAW_O_CENTICM;
    posDelta = (int32_t)H26_T3_RAW_POS5_CENTICM -
        (int32_t)H26_T3_RAW_O_CENTICM;
    negDelta = (int32_t)H26_T3_RAW_NEG5_CENTICM -
        (int32_t)H26_T3_RAW_O_CENTICM;

    if (delta == 0)
    {
        *positionCm = 0.0f;
    }
    else if ((delta > 0 && posDelta > 0) || (delta < 0 && posDelta < 0))
    {
        *positionCm = H26_T3_TARGET_POSITIVE_CM *
            (float)delta / (float)posDelta;
    }
    else if ((delta > 0 && negDelta > 0) || (delta < 0 && negDelta < 0))
    {
        *positionCm = H26_T3_TARGET_NEGATIVE_CM *
            (float)delta / (float)negDelta;
    }
    else
    {
        return 0U;
    }

    return 1U;
}

static void H26_T3_StopCommand(void)
{
    RodStepper_Stop();
    s_commandHz = 0;
    s_stepperActive = 0U;
    s_directionGuardActive = 0U;
}

static void H26_T3_EnterFault(H26_Task3Fault_t fault)
{
    H26_T3_StopCommand();
    s_fault = fault;
    s_state = H26_T3_FAULT;
}

static H26_Task3VisionStatus_t H26_T3_NoReliableMeasurement(uint32_t nowMs)
{
    s_visionValid = 0U;

    if (s_hasReliableMeasurement != 0U &&
        (nowMs - s_lastReliableVisionMs) > H26_T3_VISION_FAULT_MS)
    {
        return H26_T3_VISION_FAULT;
    }
    return H26_T3_VISION_HELD;
}

static H26_Task3VisionStatus_t H26_T3_UpdateMeasurement(uint32_t nowMs)
{
    BallLinkFrame_t frame;
    float positionCm;
    uint32_t dtMs;

    s_frameAgeMs = App_BallLink_GetFrameAgeMs(nowMs);
    if (App_BallLink_GetLatest(&frame) == 0U ||
        s_frameAgeMs > H26_T3_MAX_FRAME_AGE_MS)
    {
        return H26_T3_NoReliableMeasurement(nowMs);
    }

    s_lastSequence = frame.sequence;
    s_lastFlags = frame.flags;
    s_confidence = frame.confidence;
    if (s_hasPacket != 0U && frame.sequence == s_lastPacketSequence)
    {
        return H26_T3_NoReliableMeasurement(nowMs);
    }
    s_hasPacket = 1U;
    s_lastPacketSequence = frame.sequence;

    if (frame.ballValid == 0U || frame.pipeValid == 0U ||
        frame.confidence < H26_T3_MIN_CONFIDENCE ||
        H26_T3_CalibratePositionCm(frame.positionCentiCm, &positionCm) == 0U)
    {
        return H26_T3_NoReliableMeasurement(nowMs);
    }

    if (s_hasReliableMeasurement != 0U)
    {
        dtMs = frame.receiveTimeMs - s_lastMeasurementMs;
        if (dtMs >= H26_T3_MIN_SPEED_DT_MS && dtMs <= H26_T3_MAX_SPEED_DT_MS)
        {
            float rawSpeed = (positionCm - s_lastPositionCm) * 1000.0f /
                (float)dtMs;
            if (H26_T3_AbsFloat(positionCm - s_lastPositionCm) >
                    H26_T3_MAX_POSITION_JUMP_CM ||
                H26_T3_AbsFloat(rawSpeed) > H26_T3_MAX_BALL_SPEED_CMPS)
            {
                s_visionIssue = H26_T3_FAULT_VISION_JUMP;
                return H26_T3_NoReliableMeasurement(nowMs);
            }
            s_ballSpeedCmps = H26_T3_SPEED_FILTER_ALPHA * rawSpeed +
                (1.0f - H26_T3_SPEED_FILTER_ALPHA) * s_ballSpeedCmps;
            s_newMeasurementDtMs = (uint16_t)dtMs;
        }
        else
        {
            /* A real frame after a long gap may update position, never velocity. */
            s_ballSpeedCmps = 0.0f;
            s_newMeasurementDtMs = H26_T3_NOMINAL_FRAME_MS;
        }
    }
    else
    {
        s_ballSpeedCmps = 0.0f;
        s_newMeasurementDtMs = H26_T3_NOMINAL_FRAME_MS;
    }

    s_rawPositionCentiCm = frame.positionCentiCm;
    s_positionCm = positionCm;
    s_lastMeasurementMs = frame.receiveTimeMs;
    s_lastReliableVisionMs = frame.receiveTimeMs;
    s_lastPositionCm = positionCm;
    s_hasReliableMeasurement = 1U;
    s_visionIssue = H26_T3_FAULT_NONE;
    s_visionValid = 1U;
    return H26_T3_VISION_NEW;
}

static uint8_t H26_T3_IsRawEndGuarded(int32_t physicalCommandHz)
{
    int32_t posRawDelta;
    int8_t physicalDirection;
    int8_t rawDirection;

    if (physicalCommandHz == 0)
    {
        return 0U;
    }

    posRawDelta = (int32_t)H26_T3_RAW_POS5_CENTICM -
        (int32_t)H26_T3_RAW_O_CENTICM;
    physicalDirection = H26_T3_SignInt32(physicalCommandHz);
    rawDirection = (int8_t)(physicalDirection * H26_T3_SignInt32(posRawDelta));

    if (rawDirection > 0 &&
        (uint32_t)s_rawPositionCentiCm + H26_T3_RAW_END_GUARD_CENTICM >=
            H26_T3_RAW_MAX_CENTICM)
    {
        return 1U;
    }
    if (rawDirection < 0 &&
        s_rawPositionCentiCm <= (uint16_t)(H26_T3_RAW_MIN_CENTICM +
            H26_T3_RAW_END_GUARD_CENTICM))
    {
        return 1U;
    }
    return 0U;
}

static uint8_t H26_T3_CheckRodProtection(uint32_t nowMs, int32_t commandHz)
{
    RodEncoderSnapshot_t snapshot;

    RodEncoder_GetSnapshot(&snapshot);
    s_rodEncoderCount = snapshot.count;
    if (H26_T3_AbsInt32ToU32(snapshot.count) >=
        (uint32_t)H26_T3_ROD_SOFT_LIMIT_COUNT)
    {
        s_rodSoftLimitActive = 1U;
        s_fault = H26_T3_FAULT_ROD_SOFT_LIMIT;
        return 0U;
    }
    s_rodSoftLimitActive = 0U;

    if (H26_T3_AbsInt32ToU32(commandHz) >= H26_T3_ROD_STALL_MIN_COMMAND_HZ)
    {
        if (snapshot.count != s_lastRodEncoderCount)
        {
            s_lastRodEncoderCount = snapshot.count;
            s_lastRodMotionMs = nowMs;
        }
        else if ((nowMs - s_lastRodMotionMs) >= H26_T3_ROD_STALL_FAULT_MS)
        {
            s_fault = H26_T3_FAULT_ROD_STALL;
            return 0U;
        }
    }
    else
    {
        s_lastRodEncoderCount = snapshot.count;
        s_lastRodMotionMs = nowMs;
    }

    return 1U;
}

static uint8_t H26_T3_ApplyVelocity(uint32_t nowMs, int32_t signedHz)
{
    uint32_t magnitude = H26_T3_AbsInt32ToU32(signedHz);
    RodStepperDirection_t direction;

    if (magnitude == 0U)
    {
        H26_T3_StopCommand();
        return 1U;
    }

    direction = (signedHz > 0) ? ROD_STEPPER_DIR_POSITIVE :
        ROD_STEPPER_DIR_NEGATIVE;
    if (s_stepperActive != 0U && direction != s_appliedDirection)
    {
        RodStepper_Stop();
        s_stepperActive = 0U;
        s_directionGuardActive = 1U;
        s_directionGuardStartMs = nowMs;
        return 1U;
    }

    if (s_directionGuardActive != 0U)
    {
        if ((nowMs - s_directionGuardStartMs) < H26_T3_DIRECTION_GUARD_MS)
        {
            return 1U;
        }
        s_directionGuardActive = 0U;
    }

    if (RodStepper_SetVelocity(direction, magnitude) == 0U)
    {
        return 0U;
    }

    s_stepperActive = 1U;
    s_appliedDirection = direction;
    return 1U;
}

static H26_Task3VisionStatus_t H26_T3_ApplyBallControl(uint32_t nowMs,
                                                          float targetCm)
{
    H26_Task3VisionStatus_t visionStatus;
    float physicalCommand;
    int32_t physicalCommandHz;
    int32_t targetStepperHz;

    visionStatus = H26_T3_UpdateMeasurement(nowMs);
    if (visionStatus == H26_T3_VISION_FAULT)
    {
        s_fault = (s_visionIssue == H26_T3_FAULT_NONE) ?
            H26_T3_FAULT_VISION_TIMEOUT : s_visionIssue;
        return H26_T3_VISION_FAULT;
    }
    if (visionStatus == H26_T3_VISION_HELD)
    {
        if (s_hasReliableMeasurement == 0U ||
            (nowMs - s_lastReliableVisionMs) > H26_T3_VISION_SHORT_HOLD_MS)
        {
            s_ballSpeedCmps = 0.0f;
            H26_T3_StopCommand();
        }
        return H26_T3_VISION_HELD;
    }

    s_errorCm = targetCm - s_positionCm;
    physicalCommand = H26_T3_POSITION_KP_HZ_PER_CM * s_errorCm -
        H26_T3_SPEED_KD_HZ_PER_CMPS * s_ballSpeedCmps;
    physicalCommandHz = H26_T3_ClampInt32((int32_t)physicalCommand,
        -(int32_t)H26_T3_MAX_COMMAND_HZ, (int32_t)H26_T3_MAX_COMMAND_HZ);
    if (H26_T3_AbsFloat(s_errorCm) <= H26_T3_COMMAND_DEADBAND_CM)
    {
        physicalCommandHz = 0;
    }
    else if (H26_T3_AbsInt32ToU32(physicalCommandHz) < H26_T3_MIN_COMMAND_HZ)
    {
        physicalCommandHz = (physicalCommandHz >= 0) ?
            (int32_t)H26_T3_MIN_COMMAND_HZ : -(int32_t)H26_T3_MIN_COMMAND_HZ;
    }

    if (H26_T3_IsRawEndGuarded(physicalCommandHz) != 0U)
    {
        s_fault = H26_T3_FAULT_RAW_END_GUARD;
        return H26_T3_VISION_FAULT;
    }

    targetStepperHz = (H26_T3_STEPPER_SIGN_FOR_POSITIVE_BALL < 0) ?
        -physicalCommandHz : physicalCommandHz;
    s_commandHz = H26_T3_SlewHz(s_commandHz, targetStepperHz);
    if (H26_T3_CheckRodProtection(nowMs, s_commandHz) == 0U)
    {
        return H26_T3_VISION_FAULT;
    }
    if (H26_T3_ApplyVelocity(nowMs, s_commandHz) == 0U)
    {
        s_fault = H26_T3_FAULT_STEPPER_OUTPUT;
        return H26_T3_VISION_FAULT;
    }

    return H26_T3_VISION_NEW;
}

static uint8_t H26_T3_IsTargetStable(float targetCm)
{
    return (H26_T3_AbsFloat(s_positionCm - targetCm) <= H26_T3_TARGET_TOLERANCE_CM &&
            H26_T3_AbsFloat(s_ballSpeedCmps) <= H26_T3_STABLE_SPEED_CMPS) ? 1U : 0U;
}

static uint16_t H26_T3_GetStableSampleMs(void)
{
    return (s_newMeasurementDtMs == 0U) ? H26_T3_NOMINAL_FRAME_MS :
        s_newMeasurementDtMs;
}

void H26_Task3_Init(void)
{
    H26_Task3_Reset();
}

void H26_Task3_Reset(void)
{
    H26_T3_StopCommand();
    s_state = H26_T3_IDLE;
    s_fault = H26_T3_FAULT_NONE;
    s_startMs = 0U;
    s_finalElapsedMs = 0U;
    s_acquireHoldMs = 0U;
    s_targetHoldMs = 0U;
    s_rawPositionCentiCm = 0U;
    s_positionCm = 0.0f;
    s_ballSpeedCmps = 0.0f;
    s_targetCm = 0.0f;
    s_errorCm = 0.0f;
    s_plusHoldPeakErrorCm = 0.0f;
    s_minusHoldPeakErrorCm = 0.0f;
    s_visionValid = 0U;
    s_confidence = 0U;
    s_lastFlags = 0U;
    s_frameAgeMs = 0xFFFFFFFFUL;
    s_lastSequence = 0U;
    s_rodEncoderCount = 0;
    s_rodSoftLimitActive = 0U;
    s_lastPacketSequence = 0U;
    s_lastReliableVisionMs = 0U;
    s_lastMeasurementMs = 0U;
    s_lastRodMotionMs = 0U;
    s_lastPositionCm = 0.0f;
    s_lastRodEncoderCount = 0;
    s_newMeasurementDtMs = 0U;
    s_hasPacket = 0U;
    s_hasReliableMeasurement = 0U;
    s_visionIssue = H26_T3_FAULT_NONE;
}

void H26_Task3_Start(uint32_t startMs)
{
    H26_Task3_Reset();
    App_BallLink_Reset();
    RodEncoder_Reset();
    s_startMs = startMs;
    s_lastRodMotionMs = startMs;
    s_targetCm = 0.0f;
    if (H26_T3_CalibrationIsValid() == 0U)
    {
        H26_T3_EnterFault(H26_T3_FAULT_CONFIGURATION);
    }
    else
    {
        s_state = H26_T3_ACQUIRE_O;
    }
}

void H26_Task3_ForceFault(void)
{
    H26_T3_EnterFault((s_fault == H26_T3_FAULT_NONE) ?
        H26_T3_FAULT_ILLEGAL_STATE : s_fault);
}

H26_Task3Result_t H26_Task3_Task10ms(uint32_t nowMs)
{
    H26_Task3VisionStatus_t visionStatus;
    uint32_t elapsedMs = nowMs - s_startMs;

    if ((s_state == H26_T3_ACQUIRE_O || s_state == H26_T3_MOVE_PLUS_5 ||
         s_state == H26_T3_HOLD_PLUS_5 || s_state == H26_T3_MOVE_MINUS_5 ||
         s_state == H26_T3_HOLD_MINUS_5) && elapsedMs > H26_T3_MAX_RUN_TIME_MS)
    {
        H26_T3_EnterFault(H26_T3_FAULT_RUN_TIMEOUT);
        return H26_T3_RESULT_FAULT;
    }

    if (s_state != H26_T3_IDLE && s_state != H26_T3_FAULT &&
        H26_T3_CheckRodProtection(nowMs, s_commandHz) == 0U)
    {
        H26_T3_EnterFault(s_fault);
        return H26_T3_RESULT_FAULT;
    }

    switch (s_state)
    {
    case H26_T3_ACQUIRE_O:
        H26_T3_StopCommand();
        visionStatus = H26_T3_UpdateMeasurement(nowMs);
        if (visionStatus == H26_T3_VISION_NEW &&
            H26_T3_AbsFloat(s_positionCm) <= H26_T3_START_O_TOLERANCE_CM)
        {
            s_acquireHoldMs = H26_T3_AddMs(s_acquireHoldMs,
                H26_T3_GetStableSampleMs());
            if (s_acquireHoldMs >= H26_T3_ACQUIRE_HOLD_MS)
            {
                s_targetCm = H26_T3_TARGET_POSITIVE_CM;
                s_targetHoldMs = 0U;
                s_state = H26_T3_MOVE_PLUS_5;
            }
        }
        else if (visionStatus == H26_T3_VISION_NEW)
        {
            s_acquireHoldMs = 0U;
        }
        else
        {
            s_acquireHoldMs = 0U;
        }
        if (s_state == H26_T3_ACQUIRE_O &&
            elapsedMs >= H26_T3_ACQUIRE_TIMEOUT_MS)
        {
            H26_T3_EnterFault(H26_T3_FAULT_ACQUIRE_TIMEOUT);
            return H26_T3_RESULT_FAULT;
        }
        break;

    case H26_T3_MOVE_PLUS_5:
        visionStatus = H26_T3_ApplyBallControl(nowMs, s_targetCm);
        if (visionStatus == H26_T3_VISION_FAULT)
        {
            H26_T3_EnterFault(s_fault);
            return H26_T3_RESULT_FAULT;
        }
        if (visionStatus == H26_T3_VISION_NEW && H26_T3_IsTargetStable(s_targetCm) != 0U)
        {
            s_targetHoldMs = 0U;
            s_plusHoldPeakErrorCm = H26_T3_AbsFloat(s_errorCm);
            s_state = H26_T3_HOLD_PLUS_5;
        }
        break;

    case H26_T3_HOLD_PLUS_5:
        visionStatus = H26_T3_ApplyBallControl(nowMs, s_targetCm);
        if (visionStatus == H26_T3_VISION_FAULT)
        {
            H26_T3_EnterFault(s_fault);
            return H26_T3_RESULT_FAULT;
        }
        if (visionStatus == H26_T3_VISION_NEW && H26_T3_IsTargetStable(s_targetCm) != 0U)
        {
            if (H26_T3_AbsFloat(s_errorCm) > s_plusHoldPeakErrorCm)
            {
                s_plusHoldPeakErrorCm = H26_T3_AbsFloat(s_errorCm);
            }
            s_targetHoldMs = H26_T3_AddMs(s_targetHoldMs,
                H26_T3_GetStableSampleMs());
            if (s_targetHoldMs >= H26_T3_TARGET_HOLD_MS)
            {
                s_targetCm = H26_T3_TARGET_NEGATIVE_CM;
                s_targetHoldMs = 0U;
                s_state = H26_T3_MOVE_MINUS_5;
            }
        }
        else if (visionStatus == H26_T3_VISION_NEW)
        {
            s_targetHoldMs = 0U;
            s_state = H26_T3_MOVE_PLUS_5;
        }
        else
        {
            s_targetHoldMs = 0U;
        }
        break;

    case H26_T3_MOVE_MINUS_5:
        visionStatus = H26_T3_ApplyBallControl(nowMs, s_targetCm);
        if (visionStatus == H26_T3_VISION_FAULT)
        {
            H26_T3_EnterFault(s_fault);
            return H26_T3_RESULT_FAULT;
        }
        if (visionStatus == H26_T3_VISION_NEW && H26_T3_IsTargetStable(s_targetCm) != 0U)
        {
            s_targetHoldMs = 0U;
            s_minusHoldPeakErrorCm = H26_T3_AbsFloat(s_errorCm);
            s_state = H26_T3_HOLD_MINUS_5;
        }
        break;

    case H26_T3_HOLD_MINUS_5:
        visionStatus = H26_T3_ApplyBallControl(nowMs, s_targetCm);
        if (visionStatus == H26_T3_VISION_FAULT)
        {
            H26_T3_EnterFault(s_fault);
            return H26_T3_RESULT_FAULT;
        }
        if (visionStatus == H26_T3_VISION_NEW && H26_T3_IsTargetStable(s_targetCm) != 0U)
        {
            if (H26_T3_AbsFloat(s_errorCm) > s_minusHoldPeakErrorCm)
            {
                s_minusHoldPeakErrorCm = H26_T3_AbsFloat(s_errorCm);
            }
            s_targetHoldMs = H26_T3_AddMs(s_targetHoldMs,
                H26_T3_GetStableSampleMs());
            if (s_targetHoldMs >= H26_T3_TARGET_HOLD_MS)
            {
                s_finalElapsedMs = elapsedMs;
                s_state = H26_T3_DONE_HOLD;
                return H26_T3_RESULT_FINISHED;
            }
        }
        else if (visionStatus == H26_T3_VISION_NEW)
        {
            s_targetHoldMs = 0U;
            s_state = H26_T3_MOVE_MINUS_5;
        }
        else
        {
            s_targetHoldMs = 0U;
        }
        break;

    case H26_T3_DONE_HOLD:
        visionStatus = H26_T3_ApplyBallControl(nowMs, H26_T3_TARGET_NEGATIVE_CM);
        if (visionStatus == H26_T3_VISION_FAULT)
        {
            H26_T3_EnterFault(s_fault);
            return H26_T3_RESULT_FAULT;
        }
        return H26_T3_RESULT_FINISHED;

    case H26_T3_FAULT:
        H26_T3_StopCommand();
        return H26_T3_RESULT_FAULT;

    case H26_T3_IDLE:
    default:
        H26_T3_EnterFault(H26_T3_FAULT_ILLEGAL_STATE);
        return H26_T3_RESULT_FAULT;
    }

    return H26_T3_RESULT_RUNNING;
}

H26_Task3State_t H26_Task3_GetState(void) { return s_state; }

uint32_t H26_Task3_GetElapsedMs(uint32_t nowMs)
{
    if (s_state == H26_T3_IDLE)
    {
        return 0U;
    }
    if (s_state == H26_T3_DONE_HOLD)
    {
        return s_finalElapsedMs;
    }
    return nowMs - s_startMs;
}

uint32_t H26_Task3_GetFinalElapsedMs(void) { return s_finalElapsedMs; }
float H26_Task3_GetPositionCm(void) { return s_positionCm; }
float H26_Task3_GetTargetCm(void) { return s_targetCm; }
float H26_Task3_GetErrorCm(void) { return s_errorCm; }
float H26_Task3_GetBallSpeedCmps(void) { return s_ballSpeedCmps; }
int32_t H26_Task3_GetCommandHz(void) { return s_commandHz; }
uint8_t H26_Task3_IsVisionValid(void) { return s_visionValid; }
uint8_t H26_Task3_GetConfidence(void) { return s_confidence; }
uint32_t H26_Task3_GetFrameAgeMs(void) { return s_frameAgeMs; }
uint16_t H26_Task3_GetRawPositionCentiCm(void) { return s_rawPositionCentiCm; }
uint16_t H26_Task3_GetLastSequence(void) { return s_lastSequence; }
uint8_t H26_Task3_GetLastFlags(void) { return s_lastFlags; }
uint16_t H26_Task3_GetStableHoldMs(void)
{
    return (s_state == H26_T3_ACQUIRE_O) ? s_acquireHoldMs : s_targetHoldMs;
}
float H26_Task3_GetPlusHoldPeakErrorCm(void) { return s_plusHoldPeakErrorCm; }
float H26_Task3_GetMinusHoldPeakErrorCm(void) { return s_minusHoldPeakErrorCm; }
int32_t H26_Task3_GetRodEncoderCount(void) { return s_rodEncoderCount; }
uint8_t H26_Task3_IsRodSoftLimitActive(void) { return s_rodSoftLimitActive; }
H26_Task3Fault_t H26_Task3_GetFault(void) { return s_fault; }
