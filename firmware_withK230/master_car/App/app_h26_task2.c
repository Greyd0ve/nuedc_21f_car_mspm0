#include "app_h26_task2.h"
#include "app_h26_config.h"
#include "app_car_state.h"
#include "app_control.h"
#include "app_line.h"
#include <stdint.h>

static volatile H26_Task2State_t s_state = H26_T2_IDLE;
static volatile uint32_t s_startMs = 0U;
static volatile uint32_t s_finalElapsedMs = 0U;
static volatile int32_t s_startPulse = 0;
static volatile uint16_t s_leaveStableMs = 0U;
static volatile uint16_t s_blackHoldMs = 0U;
static volatile uint16_t s_stopHoldMs = 0U;
static volatile uint8_t s_finishEnable = 0U;
static volatile uint8_t s_finishLatched = 0U;
static volatile uint32_t s_finishDetectMs = 0U;
static volatile int32_t s_finishDetectPulse = 0;

static uint16_t H26_T2_SaturatingAddMs(uint16_t value, uint16_t addMs)
{
    if (value > (uint16_t)(0xFFFFU - addMs))
    {
        return 0xFFFFU;
    }
    return (uint16_t)(value + addMs);
}

static float H26_T2_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float H26_T2_GetDistanceCmFromStart(void)
{
    int32_t pulse = g_forwardEncoderTotal - s_startPulse;

    if (pulse < 0)
    {
        pulse = 0;
    }
    return (float)pulse * ECAR_CM_PER_PULSE;
}

static void H26_T2_StopCommand(void)
{
    App_Control_ForcePWMZero();
}

static uint8_t H26_T2_ApplyLineControl(void)
{
    float turnCmd;

    App_Line_Update();
    if (g_lineValid == 0U)
    {
        H26_T2_StopCommand();
        return (g_lineLostMs >= H26_T2_LINE_LOST_FAULT_MS) ? 0U : 1U;
    }

    turnCmd = App_Line_CalcTurnCmd();
    if (turnCmd > H26_T2_TURN_LIMIT_CMPS)
    {
        turnCmd = H26_T2_TURN_LIMIT_CMPS;
    }
    else if (turnCmd < -H26_T2_TURN_LIMIT_CMPS)
    {
        turnCmd = -H26_T2_TURN_LIMIT_CMPS;
    }

    g_targetForwardSpeed = H26_T2_BASE_SPEED_CMPS;
    g_targetTurnSpeed = turnCmd;
    g_carEnable = 1U;
    App_Control_ApplyMotorOutput();
    return 1U;
}

void H26_Task2_Init(void)
{
    H26_Task2_Reset();
}

void H26_Task2_Reset(void)
{
    s_state = H26_T2_IDLE;
    s_startMs = 0U;
    s_finalElapsedMs = 0U;
    s_startPulse = 0;
    s_leaveStableMs = 0U;
    s_blackHoldMs = 0U;
    s_stopHoldMs = 0U;
    s_finishEnable = 0U;
    s_finishLatched = 0U;
    s_finishDetectMs = 0U;
    s_finishDetectPulse = 0;
}

void H26_Task2_Start(uint32_t startMs)
{
    s_startMs = startMs;
    s_finalElapsedMs = 0U;
    s_startPulse = g_forwardEncoderTotal;
    s_leaveStableMs = 0U;
    s_blackHoldMs = 0U;
    s_stopHoldMs = 0U;
    s_finishEnable = 0U;
    s_finishLatched = 0U;
    s_finishDetectMs = 0U;
    s_finishDetectPulse = 0;
    s_state = H26_T2_LEAVE_A;
}

void H26_Task2_ForceFault(void)
{
    H26_T2_StopCommand();
    s_state = H26_T2_FAULT;
}

H26_Task2State_t H26_Task2_GetState(void)
{
    return s_state;
}

uint32_t H26_Task2_GetElapsedMs(uint32_t nowMs)
{
    if (s_state == H26_T2_IDLE)
    {
        return 0U;
    }
    if (s_state == H26_T2_DONE)
    {
        return s_finalElapsedMs;
    }
    return nowMs - s_startMs;
}

uint32_t H26_Task2_GetFinalElapsedMs(void)
{
    return s_finalElapsedMs;
}

float H26_Task2_GetDistanceCm(void)
{
    return H26_T2_GetDistanceCmFromStart();
}

uint16_t H26_Task2_GetBlackHoldMs(void)
{
    return s_blackHoldMs;
}

uint16_t H26_Task2_GetStopHoldMs(void)
{
    return s_stopHoldMs;
}

uint8_t H26_Task2_IsFinishEnabled(void)
{
    return s_finishEnable;
}

uint8_t H26_Task2_IsFinishLatched(void)
{
    return s_finishLatched;
}

uint32_t H26_Task2_GetFinishDetectMs(void)
{
    return s_finishDetectMs;
}

int32_t H26_Task2_GetFinishDetectPulse(void)
{
    return s_finishDetectPulse;
}

H26_Task2Result_t H26_Task2_Task10ms(uint32_t nowMs)
{
    uint32_t elapsedMs = nowMs - s_startMs;

    if ((s_state == H26_T2_LEAVE_A || s_state == H26_T2_LAP_RUNNING) &&
        elapsedMs >= H26_T2_MAX_RUN_TIME_MS)
    {
        H26_Task2_ForceFault();
        return H26_T2_RESULT_FAULT;
    }

    switch (s_state)
    {
    case H26_T2_LEAVE_A:
        if (H26_T2_ApplyLineControl() == 0U)
        {
            H26_Task2_ForceFault();
            return H26_T2_RESULT_FAULT;
        }

        if (H26_T2_GetDistanceCmFromStart() >= H26_T2_LEAVE_DISTANCE_CM &&
            g_lineBlackCount < H26_T2_FINISH_BLACK_CHANNELS &&
            g_lineValid != 0U)
        {
            s_leaveStableMs = H26_T2_SaturatingAddMs(
                s_leaveStableMs, CAR_CONTROL_PERIOD_MS);
        }
        else
        {
            s_leaveStableMs = 0U;
        }

        if (s_leaveStableMs >= H26_T2_LEAVE_LINE_STABLE_MS)
        {
            s_state = H26_T2_LAP_RUNNING;
        }
        break;

    case H26_T2_LAP_RUNNING:
        if (H26_T2_ApplyLineControl() == 0U)
        {
            H26_Task2_ForceFault();
            return H26_T2_RESULT_FAULT;
        }

        if (s_finishEnable == 0U &&
            H26_T2_GetDistanceCmFromStart() >= H26_T2_MIN_FINISH_DISTANCE_CM &&
            elapsedMs >= H26_T2_MIN_FINISH_TIME_MS)
        {
            s_finishEnable = 1U;
            s_blackHoldMs = 0U;
        }

        if (s_finishEnable != 0U)
        {
            if (g_lineBlackCount >= H26_T2_FINISH_BLACK_CHANNELS)
            {
                s_blackHoldMs = H26_T2_SaturatingAddMs(
                    s_blackHoldMs, CAR_CONTROL_PERIOD_MS);
            }
            else
            {
                s_blackHoldMs = 0U;
            }

            if (s_blackHoldMs >= H26_T2_FINISH_HOLD_MS)
            {
                s_finishLatched = 1U;
                s_finishDetectMs = nowMs;
                s_finishDetectPulse = g_forwardEncoderTotal;
                H26_T2_StopCommand();
                s_state = H26_T2_BRAKING;
            }
        }
        break;

    case H26_T2_BRAKING:
        H26_T2_StopCommand();
        s_stopHoldMs = 0U;
        s_state = H26_T2_WAIT_STOP;
        break;

    case H26_T2_WAIT_STOP:
        H26_T2_StopCommand();
        if (H26_T2_AbsFloat(g_leftSpeed) < H26_T2_STOP_SPEED_CMPS &&
            H26_T2_AbsFloat(g_rightSpeed) < H26_T2_STOP_SPEED_CMPS)
        {
            s_stopHoldMs = H26_T2_SaturatingAddMs(
                s_stopHoldMs, CAR_CONTROL_PERIOD_MS);
        }
        else
        {
            s_stopHoldMs = 0U;
        }

        if (s_stopHoldMs >= H26_T2_STOP_HOLD_MS)
        {
            s_finalElapsedMs = nowMs - s_startMs;
            s_state = H26_T2_DONE;
            return H26_T2_RESULT_FINISHED;
        }
        break;

    case H26_T2_DONE:
        H26_T2_StopCommand();
        return H26_T2_RESULT_FINISHED;

    case H26_T2_FAULT:
        H26_T2_StopCommand();
        return H26_T2_RESULT_FAULT;

    case H26_T2_IDLE:
    default:
        H26_Task2_ForceFault();
        return H26_T2_RESULT_FAULT;
    }

    return H26_T2_RESULT_RUNNING;
}
