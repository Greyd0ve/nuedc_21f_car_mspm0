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
        sample = H26_BallControl_Task10ms(nowMs, 0.0f);
        H26_T5_UpdateBallPeak(sample);
        if (sample == H26_BALL_SAMPLE_NEW &&
            H26_T5_AbsFloat(H26_BallControl_GetPositionCm()) <=
                H26_T5_O_TOLERANCE_CM)
        {
            s_oAcquireHoldMs = H26_T5_AddMs(s_oAcquireHoldMs,
                H26_BallControl_GetStableSampleMs());
            if (s_oAcquireHoldMs >= H26_T5_O_ACQUIRE_HOLD_MS)
            {
                /* Shared chassis machine, with task-5-only line parameters. */
                H26_Task2_StartForTask5(s_startMs);
                s_chassisStartMs = nowMs;
                s_state = H26_T5_LEAVE_A;
            }
        }
        else
        {
            s_oAcquireHoldMs = 0U;
        }
        break;

    case H26_T5_LEAVE_A:
    case H26_T5_LAP_RUNNING:
        sample = H26_BallControl_Task10ms(nowMs, 0.0f);
        H26_T5_UpdateBallPeak(sample);
        H26_Task2_SetForwardSpeedLimit(H26_T5_GetChassisSpeedLimit(nowMs));
        chassisResult = H26_Task2_Task10ms(nowMs);

        if (H26_Task2_GetState() == H26_T2_LAP_RUNNING)
        {
            s_state = H26_T5_LAP_RUNNING;
        }
        if (chassisResult == H26_T2_RESULT_FINISHED)
        {
            s_finalElapsedMs = H26_Task2_GetFinalElapsedMs();
            s_state = H26_T5_DONE;
            return H26_T5_RESULT_FINISHED;
        }
        if (chassisResult == H26_T2_RESULT_FAULT)
        {
            H26_T5_EnterFault(H26_T5_FAULT_ILLEGAL_STATE);
            return H26_T5_RESULT_FAULT;
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
    H26_BallControlSample_t sample = H26_BallControl_Task10ms(nowMs, 0.0f);
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
    if (s_state == H26_T5_DONE)
    {
        return s_finalElapsedMs;
    }
    return nowMs - s_startMs;
}

uint32_t H26_Task5_GetFinalElapsedMs(void) { return s_finalElapsedMs; }
float H26_Task5_GetDistanceCm(void) { return H26_Task2_GetDistanceCm(); }
float H26_Task5_GetBallPeakErrorCm(void) { return s_ballPeakErrorCm; }
float H26_Task5_GetBallPositionCm(void) { return H26_BallControl_GetPositionCm(); }
