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
static volatile int32_t s_straightStartPulse = 0;
static volatile int32_t s_curveLeftStartPulse = 0;
static volatile uint16_t s_leaveStableMs = 0U;
static volatile uint16_t s_blackHoldMs = 0U;
static volatile uint16_t s_stopHoldMs = 0U;
static volatile uint8_t s_finishEnable = 0U;
static volatile uint8_t s_finishLatched = 0U;
static volatile uint32_t s_finishDetectMs = 0U;
static volatile int32_t s_finishDetectPulse = 0;
static volatile H26_Task2SpeedZone_t s_speedZone = H26_T2_SPEED_ZONE_STRAIGHT;
static volatile uint8_t s_curveMode = 0U;
static volatile uint16_t s_curveEnterHoldMs = 0U;
static volatile uint16_t s_curveExitHoldMs = 0U;
static volatile float s_commandForwardSpeed = 0.0f;
static volatile float s_forwardSpeedLimitCmps = H26_T2_STRAIGHT_SPEED_CMPS;
static volatile uint8_t s_task5Profile = 0U;

#define H26_T2_PROFILE_VALUE(task2Value, task5Value) \
    ((s_task5Profile != 0U) ? (task5Value) : (task2Value))

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

static float H26_T2_SlewFloat(float current, float target, float maxStep)
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

/*
 * Task 2 uses the physical wide black A-point marker, not curve counting.
 * Once both lap guards have elapsed, the marker is confirmed directly from
 * the required number of black sensor channels.
 */
static uint8_t H26_T2_DetectFinishMarker(uint32_t nowMs, uint32_t elapsedMs)
{
    if (s_task5Profile != 0U)
    {
        if (elapsedMs < H26_T5_A_DETECT_ENABLE_MS)
        {
            s_blackHoldMs = 0U;
            return 0U;
        }
    }
    else if (H26_T2_GetDistanceCmFromStart() < H26_T2_MIN_FINISH_DISTANCE_CM ||
             elapsedMs < H26_T2_MIN_FINISH_TIME_MS)
    {
        s_blackHoldMs = 0U;
        return 0U;
    }

    s_finishEnable = 1U;

    if (g_lineBlackCount < H26_T2_PROFILE_VALUE(
            H26_T2_FINISH_BLACK_CHANNELS, H26_T5_A_DETECT_BLACK_CHANNELS))
    {
        s_blackHoldMs = 0U;
        return 0U;
    }

    s_blackHoldMs = H26_T2_SaturatingAddMs(
        s_blackHoldMs, CAR_CONTROL_PERIOD_MS);
    if (s_blackHoldMs < H26_T2_PROFILE_VALUE(H26_T2_FINISH_HOLD_MS,
                                              H26_T5_A_DETECT_HOLD_MS))
    {
        return 0U;
    }

    s_finishLatched = 1U;
    s_finishDetectMs = nowMs;
    s_finishDetectPulse = g_forwardEncoderTotal;
    return 1U;
}

static uint8_t H26_T2_UpdateCurveMode(float turnCmd)
{
    float turnAbs = H26_T2_AbsFloat(turnCmd);
    uint8_t curveExitEvent = 0U;

    if (s_curveMode == 0U)
    {
        s_curveExitHoldMs = 0U;
        if ((g_forwardEncoderTotal - s_straightStartPulse) >=
                H26_T2_PROFILE_VALUE(H26_T2_CURVE_ENTER_STRAIGHT_PULSE,
                                     H26_T5_CURVE_ENTER_STRAIGHT_PULSE) &&
            turnAbs >= H26_T2_PROFILE_VALUE(H26_T2_CURVE_ENTER_TURN_CMPS,
                                             H26_T5_CURVE_ENTER_TURN_CMPS))
        {
            s_curveEnterHoldMs = H26_T2_SaturatingAddMs(
                s_curveEnterHoldMs, CAR_CONTROL_PERIOD_MS);
            if (s_curveEnterHoldMs >= H26_T2_PROFILE_VALUE(
                    H26_T2_CURVE_ENTER_HOLD_MS, H26_T5_CURVE_ENTER_HOLD_MS))
            {
                s_curveMode = 1U;
                s_curveEnterHoldMs = 0U;
                s_curveLeftStartPulse = g_leftEncoderTotal;
            }
        }
        else
        {
            s_curveEnterHoldMs = 0U;
        }
    }
    else
    {
        s_curveEnterHoldMs = 0U;
        if ((g_leftEncoderTotal - s_curveLeftStartPulse) >=
                H26_T2_PROFILE_VALUE(H26_T2_CURVE_EXIT_LEFT_PULSE,
                                     H26_T5_CURVE_EXIT_LEFT_PULSE) &&
            turnAbs <= H26_T2_PROFILE_VALUE(H26_T2_CURVE_EXIT_TURN_CMPS,
                                             H26_T5_CURVE_EXIT_TURN_CMPS))
        {
            s_curveExitHoldMs = H26_T2_SaturatingAddMs(
                s_curveExitHoldMs, CAR_CONTROL_PERIOD_MS);
            if (s_curveExitHoldMs >= H26_T2_PROFILE_VALUE(
                    H26_T2_CURVE_EXIT_HOLD_MS, H26_T5_CURVE_EXIT_HOLD_MS))
            {
                s_curveMode = 0U;
                s_curveExitHoldMs = 0U;
                curveExitEvent = 1U;
                s_straightStartPulse = g_forwardEncoderTotal;
            }
        }
        else
        {
            s_curveExitHoldMs = 0U;
        }
    }

    return curveExitEvent;
}

static float H26_T2_SelectForwardSpeed(void)
{
    float targetSpeed;

    if (s_curveMode != 0U)
    {
        s_speedZone = H26_T2_SPEED_ZONE_CURVE;
        targetSpeed = H26_T2_PROFILE_VALUE(H26_T2_CURVE_SPEED_CMPS,
                                            H26_T5_CURVE_SPEED_CMPS);
    }
    else
    {
        s_speedZone = H26_T2_SPEED_ZONE_STRAIGHT;
        targetSpeed = H26_T2_PROFILE_VALUE(H26_T2_STRAIGHT_SPEED_CMPS,
                                            H26_T5_STRAIGHT_SPEED_CMPS);
    }

    if (targetSpeed > s_forwardSpeedLimitCmps)
    {
        targetSpeed = s_forwardSpeedLimitCmps;
    }

    s_commandForwardSpeed = H26_T2_SlewFloat(s_commandForwardSpeed,
        targetSpeed, H26_T2_PROFILE_VALUE(H26_T2_SPEED_SLEW_CMPS_PER_TICK,
                                           H26_T5_SPEED_SLEW_CMPS_PER_TICK));
    return s_commandForwardSpeed;
}

static uint8_t H26_T2_ApplyLineControl(void)
{
    float turnCmd;
    float turnLimit;
    uint8_t curveExitEvent;

    App_Line_Update();
    if (g_lineValid == 0U)
    {
        if (g_lineLostMs >= H26_T2_PROFILE_VALUE(H26_T2_LINE_LOST_STOP_MS,
                                                  H26_T5_LINE_LOST_STOP_MS))
        {
            H26_T2_StopCommand();
            return 0U;
        }

        /* App_Line_Update() retains g_lineError from the last non-white
         * sample, so a brief 8-channel white gap continues smoothly rather
         * than cutting PWM and resetting the wheel-speed controllers. */
    }

    turnCmd = App_Line_CalcTurnCmd();
    if (s_curveMode != 0U)
    {
        turnLimit = H26_T2_PROFILE_VALUE(H26_T2_CURVE_TURN_LIMIT_CMPS,
                                         H26_T5_CURVE_TURN_LIMIT_CMPS);
    }
    else
    {
        turnLimit = H26_T2_PROFILE_VALUE(H26_T2_STRAIGHT_TURN_LIMIT_CMPS,
                                         H26_T5_STRAIGHT_TURN_LIMIT_CMPS);
    }
    if (turnCmd > turnLimit)
    {
        turnCmd = turnLimit;
    }
    else if (turnCmd < -turnLimit)
    {
        turnCmd = -turnLimit;
    }

    curveExitEvent = H26_T2_UpdateCurveMode(turnCmd);
    g_targetForwardSpeed = H26_T2_SelectForwardSpeed();
    g_targetTurnSpeed = turnCmd;
    g_carEnable = 1U;
    App_Control_ApplyMotorOutput();
    return curveExitEvent;
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
    s_straightStartPulse = 0;
    s_curveLeftStartPulse = 0;
    s_leaveStableMs = 0U;
    s_blackHoldMs = 0U;
    s_stopHoldMs = 0U;
    s_finishEnable = 0U;
    s_finishLatched = 0U;
    s_finishDetectMs = 0U;
    s_finishDetectPulse = 0;
    s_speedZone = H26_T2_SPEED_ZONE_STRAIGHT;
    s_curveMode = 0U;
    s_curveEnterHoldMs = 0U;
    s_curveExitHoldMs = 0U;
    s_commandForwardSpeed = 0.0f;
    s_forwardSpeedLimitCmps = H26_T2_STRAIGHT_SPEED_CMPS;
    s_task5Profile = 0U;
}

void H26_Task2_Start(uint32_t startMs)
{
    s_startMs = startMs;
    s_finalElapsedMs = 0U;
    s_startPulse = g_forwardEncoderTotal;
    s_straightStartPulse = g_forwardEncoderTotal;
    s_curveLeftStartPulse = g_leftEncoderTotal;
    s_leaveStableMs = 0U;
    s_blackHoldMs = 0U;
    s_stopHoldMs = 0U;
    s_finishEnable = 0U;
    s_finishLatched = 0U;
    s_finishDetectMs = 0U;
    s_finishDetectPulse = 0;
    s_speedZone = H26_T2_SPEED_ZONE_STRAIGHT;
    s_curveMode = 0U;
    s_curveEnterHoldMs = 0U;
    s_curveExitHoldMs = 0U;
    s_commandForwardSpeed = 0.0f;
    s_forwardSpeedLimitCmps = H26_T2_STRAIGHT_SPEED_CMPS;
    s_task5Profile = 0U;
    s_state = H26_T2_LEAVE_A;
}

void H26_Task2_StartForTask5(uint32_t startMs)
{
    H26_Task2_Start(startMs);
    s_task5Profile = 1U;
    s_forwardSpeedLimitCmps = H26_T5_STRAIGHT_SPEED_CMPS;
}

void H26_Task2_ForceFault(void)
{
    H26_T2_StopCommand();
    s_state = H26_T2_FAULT;
}

void H26_Task2_SetForwardSpeedLimit(float limitCmps)
{
    if (limitCmps < 0.0f)
    {
        limitCmps = 0.0f;
    }
    if (limitCmps > H26_T2_PROFILE_VALUE(H26_T2_STRAIGHT_SPEED_CMPS,
                                          H26_T5_STRAIGHT_SPEED_CMPS))
    {
        limitCmps = H26_T2_PROFILE_VALUE(H26_T2_STRAIGHT_SPEED_CMPS,
                                          H26_T5_STRAIGHT_SPEED_CMPS);
    }
    s_forwardSpeedLimitCmps = limitCmps;
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

H26_Task2SpeedZone_t H26_Task2_GetSpeedZone(void)
{
    return s_speedZone;
}

uint8_t H26_Task2_IsCurveMode(void)
{
    return s_curveMode;
}

float H26_Task2_GetCommandForwardSpeed(void)
{
    return s_commandForwardSpeed;
}

H26_Task2Result_t H26_Task2_Task10ms(uint32_t nowMs)
{
    uint32_t elapsedMs = nowMs - s_startMs;

    if ((s_state == H26_T2_LEAVE_A || s_state == H26_T2_LAP_RUNNING) &&
        elapsedMs >= H26_T2_PROFILE_VALUE(H26_T2_MAX_RUN_TIME_MS,
                                          H26_T5_MAX_RUN_TIME_MS))
    {
        H26_Task2_ForceFault();
        return H26_T2_RESULT_FAULT;
    }

    switch (s_state)
    {
    case H26_T2_LEAVE_A:
        (void)H26_T2_ApplyLineControl();

        if (H26_T2_GetDistanceCmFromStart() >= H26_T2_PROFILE_VALUE(
                H26_T2_LEAVE_DISTANCE_CM, H26_T5_LEAVE_DISTANCE_CM) &&
            g_lineBlackCount < H26_T2_PROFILE_VALUE(
                H26_T2_LEAVE_CLEAR_BLACK_CHANNELS,
                H26_T5_LEAVE_CLEAR_BLACK_CHANNELS) &&
            g_lineValid != 0U)
        {
            s_leaveStableMs = H26_T2_SaturatingAddMs(
                s_leaveStableMs, CAR_CONTROL_PERIOD_MS);
        }
        else
        {
            s_leaveStableMs = 0U;
        }

        if (s_leaveStableMs >= H26_T2_PROFILE_VALUE(
                H26_T2_LEAVE_LINE_STABLE_MS, H26_T5_LEAVE_LINE_STABLE_MS))
        {
            s_state = H26_T2_LAP_RUNNING;
        }
        break;

    case H26_T2_LAP_RUNNING:
    {
        (void)H26_T2_ApplyLineControl();
        if (H26_T2_DetectFinishMarker(nowMs, elapsedMs) != 0U)
        {
            s_state = H26_T2_BRAKING;
        }
        break;
    }

    case H26_T2_BRAKING:
        if ((nowMs - s_finishDetectMs) < H26_T2_PROFILE_VALUE(
                H26_T2_FINISH_EXIT_STOP_DELAY_MS,
                H26_T5_FINISH_EXIT_STOP_DELAY_MS))
        {
            /* Continue ordinary line following before the final PWM stop. */
            (void)H26_T2_ApplyLineControl();
        }
        else
        {
            H26_T2_StopCommand();
            s_stopHoldMs = 0U;
            s_state = H26_T2_WAIT_STOP;
        }
        break;

    case H26_T2_WAIT_STOP:
        H26_T2_StopCommand();
        if (H26_T2_AbsFloat(g_leftSpeed) < H26_T2_PROFILE_VALUE(
                H26_T2_STOP_SPEED_CMPS, H26_T5_STOP_SPEED_CMPS) &&
            H26_T2_AbsFloat(g_rightSpeed) < H26_T2_PROFILE_VALUE(
                H26_T2_STOP_SPEED_CMPS, H26_T5_STOP_SPEED_CMPS))
        {
            s_stopHoldMs = H26_T2_SaturatingAddMs(
                s_stopHoldMs, CAR_CONTROL_PERIOD_MS);
        }
        else
        {
            s_stopHoldMs = 0U;
        }

        if (s_stopHoldMs >= H26_T2_PROFILE_VALUE(H26_T2_STOP_HOLD_MS,
                                                  H26_T5_STOP_HOLD_MS))
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
