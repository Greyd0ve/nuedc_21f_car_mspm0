#include "app_h26_task4.h"
#include "app_h26_ball_control.h"
#include "app_h26_config.h"
#include "app_car_state.h"
#include "app_control.h"
#include "app_line.h"
#include <stdint.h>

static volatile H26_Task4State_t s_state = H26_T4_IDLE;
static volatile H26_Task4Fault_t s_fault = H26_T4_FAULT_NONE;
static volatile uint32_t s_startMs = 0U;
static volatile uint32_t s_finalElapsedMs = 0U;
static volatile int32_t s_startPulse = 0;
static volatile uint16_t s_oAcquireHoldMs = 0U;
static volatile uint8_t s_bPassed = 0U;
static volatile uint32_t s_bPassMs = 0U;
static volatile float s_commandForwardSpeed = 0.0f;
static volatile float s_ballPeakErrorCm = 0.0f;

static float H26_T4_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint16_t H26_T4_AddMs(uint16_t value, uint16_t deltaMs)
{
    if (value > (uint16_t)(0xFFFFU - deltaMs))
    {
        return 0xFFFFU;
    }
    return (uint16_t)(value + deltaMs);
}

static float H26_T4_SlewFloat(float current, float target, float maxStep)
{
    if (maxStep <= 0.0f)
    {
        return target;
    }
    if (current < target)
    {
        current += maxStep;
        return (current > target) ? target : current;
    }
    if (current > target)
    {
        current -= maxStep;
        return (current < target) ? target : current;
    }
    return current;
}

static float H26_T4_GetDistanceCmFromStart(void)
{
    int32_t pulse = g_forwardEncoderTotal - s_startPulse;

    if (pulse < 0)
    {
        pulse = 0;
    }
    return (float)pulse * ECAR_CM_PER_PULSE;
}

static void H26_T4_StopCar(void)
{
    App_Control_ForcePWMZero();
    s_commandForwardSpeed = 0.0f;
}

static void H26_T4_EnterFault(H26_Task4Fault_t fault)
{
    H26_T4_StopCar();
    H26_BallControl_Stop();
    s_fault = fault;
    s_state = H26_T4_FAULT;
}

static void H26_T4_UpdateBallPeak(H26_BallControlSample_t sample)
{
    float errorAbs;

    if (sample != H26_BALL_SAMPLE_NEW)
    {
        return;
    }

    errorAbs = H26_T4_AbsFloat(H26_BallControl_GetPositionCm());
    if (errorAbs > s_ballPeakErrorCm)
    {
        s_ballPeakErrorCm = errorAbs;
    }
}

static void H26_T4_ApplyLineControl(void)
{
    float turnCmd;

    App_Line_Update();
    turnCmd = App_Line_CalcTurnCmd();
    if (turnCmd > H26_T4_TURN_LIMIT_CMPS)
    {
        turnCmd = H26_T4_TURN_LIMIT_CMPS;
    }
    else if (turnCmd < -H26_T4_TURN_LIMIT_CMPS)
    {
        turnCmd = -H26_T4_TURN_LIMIT_CMPS;
    }

    s_commandForwardSpeed = H26_T4_SlewFloat(s_commandForwardSpeed,
        H26_T4_FORWARD_SPEED_CMPS, H26_T4_SPEED_SLEW_CMPS_PER_TICK);
    g_targetForwardSpeed = s_commandForwardSpeed;
    g_targetTurnSpeed = turnCmd;
    g_carEnable = 1U;
    App_Control_ApplyMotorOutput();
}

void H26_Task4_Init(void)
{
    H26_Task4_Reset();
}

void H26_Task4_Reset(void)
{
    H26_T4_StopCar();
    H26_BallControl_Reset();
    s_state = H26_T4_IDLE;
    s_fault = H26_T4_FAULT_NONE;
    s_startMs = 0U;
    s_finalElapsedMs = 0U;
    s_startPulse = 0;
    s_oAcquireHoldMs = 0U;
    s_bPassed = 0U;
    s_bPassMs = 0U;
    s_ballPeakErrorCm = 0.0f;
}

void H26_Task4_Start(uint32_t startMs)
{
    H26_Task4_Reset();
    H26_BallControl_Start();
    s_startMs = startMs;
    s_startPulse = g_forwardEncoderTotal;
    s_state = H26_T4_ACQUIRE_O;
}

void H26_Task4_ForceFault(void)
{
    H26_T4_EnterFault((s_fault == H26_T4_FAULT_NONE) ?
        H26_T4_FAULT_ILLEGAL_STATE : s_fault);
}

H26_Task4Result_t H26_Task4_Task10ms(uint32_t nowMs)
{
    H26_BallControlSample_t sample;

    switch (s_state)
    {
    case H26_T4_ACQUIRE_O:
        H26_T4_StopCar();
        sample = H26_BallControl_Task10ms(nowMs, 0.0f);
        H26_T4_UpdateBallPeak(sample);
        if (sample == H26_BALL_SAMPLE_NEW &&
            H26_T4_AbsFloat(H26_BallControl_GetPositionCm()) <=
                H26_T4_O_TOLERANCE_CM)
        {
            s_oAcquireHoldMs = H26_T4_AddMs(s_oAcquireHoldMs,
                H26_BallControl_GetStableSampleMs());
            if (s_oAcquireHoldMs >= H26_T4_O_ACQUIRE_HOLD_MS)
            {
                s_state = H26_T4_RUN_TO_B;
            }
        }
        else
        {
            s_oAcquireHoldMs = 0U;
        }
        break;

    case H26_T4_RUN_TO_B:
        sample = H26_BallControl_Task10ms(nowMs, 0.0f);
        H26_T4_UpdateBallPeak(sample);
        H26_T4_ApplyLineControl();
        if (H26_T4_GetDistanceCmFromStart() >= H26_T4_B_DISTANCE_CM)
        {
            s_bPassed = 1U;
            s_finalElapsedMs = nowMs - s_startMs;
            s_bPassMs = s_finalElapsedMs;
            H26_T4_StopCar();
            s_state = H26_T4_DONE;
            return H26_T4_RESULT_FINISHED;
        }
        break;

    case H26_T4_DONE:
        H26_T4_StopCar();
        return H26_T4_RESULT_FINISHED;

    case H26_T4_FAULT:
        H26_T4_StopCar();
        H26_BallControl_Stop();
        return H26_T4_RESULT_FAULT;

    case H26_T4_IDLE:
    default:
        H26_T4_EnterFault(H26_T4_FAULT_ILLEGAL_STATE);
        return H26_T4_RESULT_FAULT;
    }

    return H26_T4_RESULT_RUNNING;
}

void H26_Task4_HoldBall10ms(uint32_t nowMs)
{
    H26_BallControlSample_t sample = H26_BallControl_Task10ms(nowMs, 0.0f);
    H26_T4_UpdateBallPeak(sample);
}

H26_Task4State_t H26_Task4_GetState(void) { return s_state; }
H26_Task4Fault_t H26_Task4_GetFault(void) { return s_fault; }

uint32_t H26_Task4_GetElapsedMs(uint32_t nowMs)
{
    if (s_state == H26_T4_IDLE)
    {
        return 0U;
    }
    if (s_state == H26_T4_DONE)
    {
        return s_finalElapsedMs;
    }
    return nowMs - s_startMs;
}

uint32_t H26_Task4_GetFinalElapsedMs(void) { return s_finalElapsedMs; }
float H26_Task4_GetDistanceCm(void) { return H26_T4_GetDistanceCmFromStart(); }
uint8_t H26_Task4_IsBPassed(void) { return s_bPassed; }
uint32_t H26_Task4_GetBPassMs(void) { return s_bPassMs; }
uint16_t H26_Task4_GetOAcquireHoldMs(void) { return s_oAcquireHoldMs; }
float H26_Task4_GetCommandForwardSpeed(void) { return s_commandForwardSpeed; }
float H26_Task4_GetBallPeakErrorCm(void) { return s_ballPeakErrorCm; }
float H26_Task4_GetBallPositionCm(void) { return H26_BallControl_GetPositionCm(); }
float H26_Task4_GetBallSpeedCmps(void)
{
    return H26_BallControl_GetBallSpeedCmps();
}
int32_t H26_Task4_GetRodEncoderCount(void)
{
    return H26_BallControl_GetRodEncoderCount();
}
int32_t H26_Task4_GetRodTargetCount(void)
{
    return H26_BallControl_GetRodTargetCount();
}
float H26_Task4_GetTiltCommandMm(void)
{
    return H26_BallControl_GetTiltCommandMm();
}
