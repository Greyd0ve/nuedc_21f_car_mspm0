#include "app_h26_task5.h"
#include "app_h26_ball_control.h"
#include "app_h26_config.h"
#include "app_h26_task2.h"
#include "app_control.h"
#include <stdint.h>

static volatile H26_Task5State_t s_state = H26_T5_IDLE;
static volatile H26_Task5Fault_t s_fault = H26_T5_FAULT_NONE;
static volatile uint32_t s_startMs = 0U;
static volatile uint32_t s_chassisStartMs = 0U;
static volatile uint32_t s_finalElapsedMs = 0U;
static volatile uint16_t s_oAcquireHoldMs = 0U;
static volatile float s_ballPeakErrorCm = 0.0f;

static float H26_T5_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint16_t H26_T5_AddMs(uint16_t value, uint16_t deltaMs)
{
    if (value > (uint16_t)(0xFFFFU - deltaMs))
    {
        return 0xFFFFU;
    }
    return (uint16_t)(value + deltaMs);
}

static void H26_T5_StopCar(void)
{
    App_Control_ForcePWMZero();
}

/* Reuse task 4 PID; add encoder acceleration feed-forward only while driving. */
static H26_BallControlSample_t H26_T5_UpdateBallControl(uint32_t nowMs,
                                                          uint8_t mode)
{
    float feedForwardTiltMm = 0.0f;
    float plannedFeedForwardTiltMm;
    float targetCm = H26_T5_O_TARGET_CM;
    float ballKp = H26_T5_BALL_STRAIGHT_KP_MM_PER_CM;
    float ballKi = H26_T5_BALL_STRAIGHT_KI_MM_PER_CM_S;
    float ballKd = H26_T5_BALL_STRAIGHT_KD_MM_PER_CMPS;
    uint32_t elapsedMs;

    plannedFeedForwardTiltMm = H26_T4_FF_TILT_SIGN_FOR_FORWARD_ACCEL *
        H26_T4_FF_K_MM_PER_CMPS2 * H26_T5_LAUNCH_ACCEL_CMPS2;
    if (plannedFeedForwardTiltMm > H26_T4_FF_TILT_LIMIT_MM)
    {
        plannedFeedForwardTiltMm = H26_T4_FF_TILT_LIMIT_MM;
    }

    if (mode == 2U)
    {
        /* Keep a calibrated lead position while the chassis is in motion. */
        targetCm += H26_T5_DRIVE_TARGET_COMPENSATION_CM;
        H26_BallControl_SetIntegralFrozen(
            ((nowMs - s_chassisStartMs) < H26_T5_INTEGRAL_FREEZE_MS) ?
            1U : 0U);
        feedForwardTiltMm = H26_BallControl_UpdateEncoderFeedForward(nowMs);
        elapsedMs = nowMs - s_chassisStartMs;
        if (elapsedMs < H26_T5_FF_HANDOFF_MS)
        {
            feedForwardTiltMm = plannedFeedForwardTiltMm +
                (feedForwardTiltMm - plannedFeedForwardTiltMm) *
                (float)elapsedMs / (float)H26_T5_FF_HANDOFF_MS;
        }
    }
    else if (mode == 1U)
    {
        H26_BallControl_SetIntegralFrozen(1U);
        feedForwardTiltMm = plannedFeedForwardTiltMm;
    }
    else
    {
        H26_BallControl_SetIntegralFrozen(0U);
    }

    if (mode == 2U && H26_Task2_IsCurveMode() != 0U)
    {
        ballKp = H26_T5_BALL_CURVE_KP_MM_PER_CM;
        ballKi = H26_T5_BALL_CURVE_KI_MM_PER_CM_S;
        ballKd = H26_T5_BALL_CURVE_KD_MM_PER_CMPS;
    }

    return H26_BallControl_Task10msWithPidFeedForward(nowMs,
        targetCm,
        ballKp,
        ballKi,
        ballKd,
        H26_T5_BALL_INTEGRAL_LIMIT_CM_S,
        H26_T5_BALL_TILT_COMMAND_LIMIT_MM,
        feedForwardTiltMm,
        H26_T5_BALL_POSITION_DEADBAND_CM);
}

static float H26_T5_GetChassisSpeedLimit(uint32_t nowMs)
{
    uint32_t elapsedMs;

    if (H26_T5_STRAIGHT_ACCEL_RAMP_MS == 0U)
    {
        return H26_T5_STRAIGHT_SPEED_CMPS;
    }

    elapsedMs = nowMs - s_chassisStartMs;
    if (elapsedMs >= H26_T5_STRAIGHT_ACCEL_RAMP_MS)
    {
        return H26_T5_STRAIGHT_SPEED_CMPS;
    }

    return H26_T5_STRAIGHT_SPEED_CMPS * (float)elapsedMs /
        (float)H26_T5_STRAIGHT_ACCEL_RAMP_MS;
}

static void H26_T5_EnterFault(H26_Task5Fault_t fault)
{
    H26_T5_StopCar();
    H26_Task2_ForceFault();
    H26_BallControl_Stop();
    s_fault = fault;
    s_state = H26_T5_FAULT;
}

static void H26_T5_UpdateBallPeak(H26_BallControlSample_t sample)
{
    float errorAbs;

    if (sample != H26_BALL_SAMPLE_NEW)
    {
        return;
    }

    errorAbs = H26_T5_AbsFloat(H26_BallControl_GetPositionCm());
    if (errorAbs > s_ballPeakErrorCm)
    {
        s_ballPeakErrorCm = errorAbs;
    }
}

void H26_Task5_Init(void)
{
    H26_Task5_Reset();
}

void H26_Task5_Reset(void)
{
    H26_T5_StopCar();
    H26_Task2_Reset();
    H26_BallControl_Reset();
    H26_BallControl_ResetEncoderFeedForward(0U);
    s_state = H26_T5_IDLE;
    s_fault = H26_T5_FAULT_NONE;
    s_startMs = 0U;
    s_chassisStartMs = 0U;
    s_finalElapsedMs = 0U;
    s_oAcquireHoldMs = 0U;
    s_ballPeakErrorCm = 0.0f;
}

void H26_Task5_Start(uint32_t startMs)
{
    H26_Task5_Reset();
    H26_BallControl_Start();
    H26_BallControl_ResetEncoderFeedForward(startMs);
    s_startMs = startMs;
    s_state = H26_T5_ACQUIRE_O;
}

void H26_Task5_ForceFault(void)
{
    H26_T5_EnterFault((s_fault == H26_T5_FAULT_NONE) ?
        H26_T5_FAULT_ILLEGAL_STATE : s_fault);
}

H26_Task5Result_t H26_Task5_Task10ms(uint32_t nowMs)
{
    H26_BallControlSample_t sample;
    H26_Task2Result_t chassisResult;

    switch (s_state)
    {
    case H26_T5_ACQUIRE_O:
        H26_T5_StopCar();
        sample = H26_T5_UpdateBallControl(nowMs, 0U);
        H26_T5_UpdateBallPeak(sample);
        if (sample == H26_BALL_SAMPLE_NEW &&
            H26_T5_AbsFloat(H26_BallControl_GetPositionCm()) <=
                H26_T5_O_TOLERANCE_CM)
        {
            s_oAcquireHoldMs = H26_T5_AddMs(s_oAcquireHoldMs,
                H26_BallControl_GetStableSampleMs());
            if (s_oAcquireHoldMs >= H26_T5_O_ACQUIRE_HOLD_MS)
            {
                /* Prime the rod before the first non-zero traction command. */
                s_chassisStartMs = nowMs;
                s_state = H26_T5_PRETILT;
            }
        }
        else
        {
            s_oAcquireHoldMs = 0U;
        }
        break;

    case H26_T5_PRETILT:
        H26_T5_StopCar();
        sample = H26_T5_UpdateBallControl(nowMs, 1U);
        H26_T5_UpdateBallPeak(sample);
        if ((nowMs - s_chassisStartMs) >= H26_T5_PRETILT_MS)
        {
            /* Shared chassis machine with task-2 line-following parameters. */
            s_chassisStartMs = nowMs;
            H26_Task2_StartForTask5(s_chassisStartMs);
            H26_BallControl_ResetEncoderFeedForward(nowMs);
            s_state = H26_T5_LEAVE_A;
        }
        break;

    case H26_T5_LEAVE_A:
    case H26_T5_LAP_RUNNING:
        sample = H26_T5_UpdateBallControl(nowMs, 2U);
        H26_T5_UpdateBallPeak(sample);
        H26_Task2_SetForwardSpeedLimit(H26_T5_GetChassisSpeedLimit(nowMs));
        chassisResult = H26_Task2_Task10ms(nowMs);

        if (H26_Task2_GetState() == H26_T2_LAP_RUNNING)
        {
            s_state = H26_T5_LAP_RUNNING;
        }
        if (chassisResult == H26_T2_RESULT_FINISHED)
        {
            if (s_finalElapsedMs == 0U)
            {
                s_finalElapsedMs = H26_Task2_GetFinalElapsedMs();
            }
            s_state = H26_T5_DONE;
            return H26_T5_RESULT_FINISHED;
        }
        if (chassisResult == H26_T2_RESULT_FAULT)
        {
            H26_T5_EnterFault(H26_T5_FAULT_ILLEGAL_STATE);
            return H26_T5_RESULT_FAULT;
        }
        if (s_finalElapsedMs == 0U &&
            H26_Task2_GetState() == H26_T2_BRAKING)
        {
            /* A is confirmed: freeze task-5 time while task-2 tracks out. */
            s_finalElapsedMs = nowMs - s_startMs;
        }
        break;

    case H26_T5_DONE:
        H26_T5_StopCar();
        return H26_T5_RESULT_FINISHED;

    case H26_T5_FAULT:
        H26_T5_StopCar();
        H26_BallControl_Stop();
        return H26_T5_RESULT_FAULT;

    case H26_T5_IDLE:
    default:
        H26_T5_EnterFault(H26_T5_FAULT_ILLEGAL_STATE);
        return H26_T5_RESULT_FAULT;
    }

    return H26_T5_RESULT_RUNNING;
}

void H26_Task5_HoldBall10ms(uint32_t nowMs)
{
    H26_BallControlSample_t sample = H26_T5_UpdateBallControl(nowMs, 0U);
    H26_T5_UpdateBallPeak(sample);
}

H26_Task5State_t H26_Task5_GetState(void) { return s_state; }
H26_Task5Fault_t H26_Task5_GetFault(void) { return s_fault; }

uint32_t H26_Task5_GetElapsedMs(uint32_t nowMs)
{
    if (s_state == H26_T5_IDLE)
    {
        return 0U;
    }
    if (s_finalElapsedMs != 0U || s_state == H26_T5_DONE)
    {
        return s_finalElapsedMs;
    }
    return nowMs - s_startMs;
}

uint32_t H26_Task5_GetFinalElapsedMs(void) { return s_finalElapsedMs; }
float H26_Task5_GetDistanceCm(void) { return H26_Task2_GetDistanceCm(); }
float H26_Task5_GetBallPeakErrorCm(void) { return s_ballPeakErrorCm; }
float H26_Task5_GetBallPositionCm(void) { return H26_BallControl_GetPositionCm(); }
