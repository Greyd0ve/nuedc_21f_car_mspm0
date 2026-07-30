#include "app_h26_task3.h"
#include "app_h26_ball_control.h"
#include "app_h26_config.h"
#include <stdint.h>

static volatile H26_Task3State_t s_state = H26_T3_IDLE;
static volatile H26_Task3Fault_t s_fault = H26_T3_FAULT_NONE;
static volatile uint32_t s_startMs = 0U;
static volatile uint32_t s_finalElapsedMs = 0U;
static volatile uint16_t s_acquireHoldMs = 0U;
static volatile uint16_t s_targetHoldMs = 0U;
static volatile float s_targetCm = 0.0f;
static volatile float s_plusHoldPeakErrorCm = 0.0f;
static volatile float s_minusHoldPeakErrorCm = 0.0f;

static float H26_T3_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint16_t H26_T3_AddMs(uint16_t value, uint16_t deltaMs)
{
    if (value > (uint16_t)(0xFFFFU - deltaMs))
    {
        return 0xFFFFU;
    }
    return (uint16_t)(value + deltaMs);
}

static void H26_T3_StopCommand(void)
{
    H26_BallControl_Stop();
}

static void H26_T3_EnterFault(H26_Task3Fault_t fault)
{
    H26_T3_StopCommand();
    s_fault = fault;
    s_state = H26_T3_FAULT;
}

static uint8_t H26_T3_IsTargetStable(float targetCm)
{
    return (H26_T3_AbsFloat(H26_BallControl_GetPositionCm() - targetCm) <=
                H26_T3_TARGET_TOLERANCE_CM &&
            H26_T3_AbsFloat(H26_BallControl_GetBallSpeedCmps()) <=
                H26_T3_STABLE_SPEED_CMPS) ? 1U : 0U;
}

void H26_Task3_Init(void)
{
    H26_BallControl_Init();
    H26_Task3_Reset();
}

void H26_Task3_Reset(void)
{
    H26_BallControl_Reset();
    s_state = H26_T3_IDLE;
    s_fault = H26_T3_FAULT_NONE;
    s_startMs = 0U;
    s_finalElapsedMs = 0U;
    s_acquireHoldMs = 0U;
    s_targetHoldMs = 0U;
    s_targetCm = 0.0f;
    s_plusHoldPeakErrorCm = 0.0f;
    s_minusHoldPeakErrorCm = 0.0f;
}

void H26_Task3_Start(uint32_t startMs)
{
    H26_Task3_Reset();
    H26_BallControl_Start();
    s_startMs = startMs;
    s_targetCm = 0.0f;
    s_state = H26_T3_ACQUIRE_O;
}

void H26_Task3_ForceFault(void)
{
    H26_T3_EnterFault((s_fault == H26_T3_FAULT_NONE) ?
        H26_T3_FAULT_ILLEGAL_STATE : s_fault);
}

H26_Task3Result_t H26_Task3_Task10ms(uint32_t nowMs)
{
    H26_BallControlSample_t sample;
    uint32_t elapsedMs = nowMs - s_startMs;

    switch (s_state)
    {
    case H26_T3_ACQUIRE_O:
        sample = H26_BallControl_Task10ms(nowMs, 0.0f);
        if (sample == H26_BALL_SAMPLE_NEW &&
            H26_T3_AbsFloat(H26_BallControl_GetPositionCm()) <=
                H26_T3_START_O_TOLERANCE_CM)
        {
            s_acquireHoldMs = H26_T3_AddMs(s_acquireHoldMs,
                H26_BallControl_GetStableSampleMs());
            if (s_acquireHoldMs >= H26_T3_ACQUIRE_HOLD_MS)
            {
                s_targetCm = H26_T3_TARGET_POSITIVE_CM;
                s_targetHoldMs = 0U;
                s_state = H26_T3_MOVE_PLUS_5;
            }
        }
        else
        {
            s_acquireHoldMs = 0U;
        }
        break;

    case H26_T3_MOVE_PLUS_5:
        sample = H26_BallControl_Task10ms(nowMs, s_targetCm);
        if (sample == H26_BALL_SAMPLE_NEW && H26_T3_IsTargetStable(s_targetCm) != 0U)
        {
            s_targetHoldMs = 0U;
            s_plusHoldPeakErrorCm = H26_T3_AbsFloat(
                H26_BallControl_GetErrorCm());
            s_state = H26_T3_HOLD_PLUS_5;
        }
        break;

    case H26_T3_HOLD_PLUS_5:
        sample = H26_BallControl_Task10ms(nowMs, s_targetCm);
        if (sample == H26_BALL_SAMPLE_NEW && H26_T3_IsTargetStable(s_targetCm) != 0U)
        {
            if (H26_T3_AbsFloat(H26_BallControl_GetErrorCm()) >
                s_plusHoldPeakErrorCm)
            {
                s_plusHoldPeakErrorCm = H26_T3_AbsFloat(
                    H26_BallControl_GetErrorCm());
            }
            s_targetHoldMs = H26_T3_AddMs(s_targetHoldMs,
                H26_BallControl_GetStableSampleMs());
            if (s_targetHoldMs >= H26_T3_TARGET_HOLD_MS)
            {
                s_targetCm = H26_T3_TARGET_NEGATIVE_CM;
                s_targetHoldMs = 0U;
                s_state = H26_T3_MOVE_MINUS_5;
            }
        }
        else if (sample == H26_BALL_SAMPLE_NEW)
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
        sample = H26_BallControl_Task10ms(nowMs, s_targetCm);
        if (sample == H26_BALL_SAMPLE_NEW && H26_T3_IsTargetStable(s_targetCm) != 0U)
        {
            s_targetHoldMs = 0U;
            s_minusHoldPeakErrorCm = H26_T3_AbsFloat(
                H26_BallControl_GetErrorCm());
            s_state = H26_T3_HOLD_MINUS_5;
        }
        break;

    case H26_T3_HOLD_MINUS_5:
        sample = H26_BallControl_Task10ms(nowMs, s_targetCm);
        if (sample == H26_BALL_SAMPLE_NEW && H26_T3_IsTargetStable(s_targetCm) != 0U)
        {
            if (H26_T3_AbsFloat(H26_BallControl_GetErrorCm()) >
                s_minusHoldPeakErrorCm)
            {
                s_minusHoldPeakErrorCm = H26_T3_AbsFloat(
                    H26_BallControl_GetErrorCm());
            }
            s_targetHoldMs = H26_T3_AddMs(s_targetHoldMs,
                H26_BallControl_GetStableSampleMs());
            if (s_targetHoldMs >= H26_T3_TARGET_HOLD_MS)
            {
                s_finalElapsedMs = elapsedMs;
                s_state = H26_T3_DONE_HOLD;
                return H26_T3_RESULT_FINISHED;
            }
        }
        else if (sample == H26_BALL_SAMPLE_NEW)
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
        (void)H26_BallControl_Task10ms(nowMs, H26_T3_TARGET_NEGATIVE_CM);
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
float H26_Task3_GetPositionCm(void) { return H26_BallControl_GetPositionCm(); }
float H26_Task3_GetTargetCm(void) { return H26_BallControl_GetTargetCm(); }
float H26_Task3_GetErrorCm(void) { return H26_BallControl_GetErrorCm(); }
float H26_Task3_GetBallSpeedCmps(void) { return H26_BallControl_GetBallSpeedCmps(); }
int32_t H26_Task3_GetCommandHz(void) { return H26_BallControl_GetCommandHz(); }
uint8_t H26_Task3_IsVisionValid(void) { return H26_BallControl_IsVisionValid(); }
uint8_t H26_Task3_GetConfidence(void) { return H26_BallControl_GetConfidence(); }
uint32_t H26_Task3_GetFrameAgeMs(void) { return H26_BallControl_GetFrameAgeMs(); }
uint16_t H26_Task3_GetRawPositionCentiCm(void)
{
    return H26_BallControl_GetRawPositionCentiCm();
}
uint16_t H26_Task3_GetOriginCentiCm(void)
{
    return H26_BallControl_GetOriginCentiCm();
}
uint8_t H26_Task3_IsOriginCalibrated(void)
{
    return H26_BallControl_IsOriginCalibrated();
}
uint16_t H26_Task3_GetLastSequence(void)
{
    return H26_BallControl_GetLastSequence();
}
uint8_t H26_Task3_GetLastFlags(void) { return H26_BallControl_GetLastFlags(); }
uint16_t H26_Task3_GetStableHoldMs(void)
{
    return (s_state == H26_T3_ACQUIRE_O) ? s_acquireHoldMs : s_targetHoldMs;
}
float H26_Task3_GetPlusHoldPeakErrorCm(void) { return s_plusHoldPeakErrorCm; }
float H26_Task3_GetMinusHoldPeakErrorCm(void) { return s_minusHoldPeakErrorCm; }
int32_t H26_Task3_GetRodEncoderCount(void)
{
    return H26_BallControl_GetRodEncoderCount();
}
int32_t H26_Task3_GetRodTargetCount(void)
{
    return H26_BallControl_GetRodTargetCount();
}
float H26_Task3_GetTiltCommandMm(void)
{
    return H26_BallControl_GetTiltCommandMm();
}
uint8_t H26_Task3_IsRodSoftLimitActive(void) { return 0U; }
H26_Task3Fault_t H26_Task3_GetFault(void) { return s_fault; }
