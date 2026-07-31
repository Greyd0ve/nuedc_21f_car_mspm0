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
static volatile uint32_t s_oLockMs = 0U;
static volatile int32_t s_driveStartPulse = 0;
static volatile float s_commandForwardSpeedCmps = 0.0f;
static volatile uint8_t s_curveObserved = 0U;

static float H26_T4_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float H26_T4_ClampFloat(float value, float lower, float upper)
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

static float H26_T4_GetDistanceCmFromStart(void)
{
    int32_t pulse = g_forwardEncoderTotal - s_driveStartPulse;

    if (pulse < 0)
    {
        pulse = 0;
    }
    return (float)pulse * ECAR_CM_PER_PULSE;
}

static void H26_T4_StopCar(void)
{
    App_Control_ForcePWMZero();
    s_commandForwardSpeedCmps = 0.0f;
}

/* Keep the last line error through a white gap; task 4 has no lost-line stop. */
static float H26_T4_ApplyDriveLineControl(float turnLimitCmps,
                                           float forwardSpeedCmps)
{
    float turnCmd;

    App_Line_Update();
    turnCmd = App_Line_CalcTurnCmd();
    turnCmd = H26_T4_ClampFloat(turnCmd,
        -turnLimitCmps, turnLimitCmps);

    s_commandForwardSpeedCmps = forwardSpeedCmps;
    g_targetForwardSpeed = s_commandForwardSpeedCmps;
    g_targetTurnSpeed = turnCmd;
    g_carEnable = 1U;
    App_Control_ApplyMotorOutput();
    return turnCmd;
}

static void H26_T4_UpdateBallControl(uint32_t nowMs)
{
    (void)H26_BallControl_Task10msWithPidFeedForward(nowMs,
        H26_T4_O_TARGET_CM,
        H26_T4_BALL_KP_MM_PER_CM,
        H26_T4_BALL_KI_MM_PER_CM_S,
        H26_T4_BALL_KD_MM_PER_CMPS,
        H26_T4_BALL_INTEGRAL_LIMIT_CM_S,
        H26_T4_BALL_TILT_COMMAND_LIMIT_MM,
        H26_BallControl_UpdateEncoderFeedForward(nowMs));
}

static void H26_T4_EnterFault(H26_Task4Fault_t fault)
{
    H26_T4_StopCar();
    H26_BallControl_Stop();
    s_fault = fault;
    s_state = H26_T4_FAULT;
}

void H26_Task4_Init(void)
{
    H26_Task4_Reset();
}

void H26_Task4_Reset(void)
{
    H26_T4_StopCar();
    H26_BallControl_Reset();
    H26_BallControl_ResetEncoderFeedForward(0U);
    s_state = H26_T4_IDLE;
    s_fault = H26_T4_FAULT_NONE;
    s_startMs = 0U;
    s_oLockMs = 0U;
    s_driveStartPulse = 0;
    s_commandForwardSpeedCmps = 0.0f;
    s_curveObserved = 0U;
}

void H26_Task4_Start(uint32_t startMs)
{
    H26_Task4_Reset();

    /* The first valid K230 frame after this call defines the O-point origin. */
    H26_BallControl_Start();
    H26_BallControl_ResetEncoderFeedForward(startMs);
    s_startMs = startMs;
    s_state = H26_T4_MANUAL_MOVE_HOLD_O;
}

void H26_Task4_StartDrive(uint32_t startMs)
{
    H26_Task4_Reset();

    /* First valid K230 frame defines O; then hold still before the route run. */
    H26_BallControl_Start();
    H26_BallControl_ResetEncoderFeedForward(startMs);
    s_startMs = startMs;
    s_driveStartPulse = g_forwardEncoderTotal;
    s_state = H26_T4_DRIVE_ACQUIRE_O;
}

void H26_Task4_ForceFault(void)
{
    H26_T4_EnterFault((s_fault == H26_T4_FAULT_NONE) ?
        H26_T4_FAULT_ILLEGAL_STATE : s_fault);
}

H26_Task4Result_t H26_Task4_Task10ms(uint32_t nowMs)
{
    switch (s_state)
    {
    case H26_T4_MANUAL_MOVE_HOLD_O:
        /* Motor outputs coast; manual vehicle motion drives encoder feed-forward. */
        H26_T4_StopCar();
        H26_T4_UpdateBallControl(nowMs);
        return H26_T4_RESULT_RUNNING;

    case H26_T4_DRIVE_ACQUIRE_O:
        /* Do not let the car move until this run has captured its O origin. */
        H26_T4_StopCar();
        H26_T4_UpdateBallControl(nowMs);
        if ((nowMs - s_startMs) >= H26_T4_DRIVE_TIMEOUT_MS)
        {
            H26_T4_EnterFault(H26_T4_FAULT_TIMEOUT);
            return H26_T4_RESULT_FAULT;
        }
        if (H26_BallControl_IsOriginCalibrated() != 0U)
        {
            H26_BallControl_ResetEncoderFeedForward(nowMs);
            s_oLockMs = nowMs;
            s_state = H26_T4_DRIVE_HOLD_O;
        }
        return H26_T4_RESULT_RUNNING;

    case H26_T4_DRIVE_HOLD_O:
        /* Let the first-frame O reference settle for 100 ms before takeoff. */
        H26_T4_StopCar();
        H26_T4_UpdateBallControl(nowMs);
        if ((nowMs - s_startMs) >= H26_T4_DRIVE_TIMEOUT_MS)
        {
            H26_T4_EnterFault(H26_T4_FAULT_TIMEOUT);
            return H26_T4_RESULT_FAULT;
        }
        if ((nowMs - s_oLockMs) >= H26_T4_O_LOCK_HOLD_MS)
        {
            s_driveStartPulse = g_forwardEncoderTotal;
            H26_BallControl_ResetEncoderFeedForward(nowMs);
            s_state = H26_T4_DRIVE_STRAIGHT;
        }
        return H26_T4_RESULT_RUNNING;

    case H26_T4_DRIVE_STRAIGHT:
        (void)H26_T4_ApplyDriveLineControl(H26_T4_STRAIGHT_TURN_LIMIT_CMPS,
            H26_T4_DRIVE_STRAIGHT_SPEED_CMPS);
        H26_T4_UpdateBallControl(nowMs);
        if ((nowMs - s_startMs) >= H26_T4_DRIVE_TIMEOUT_MS)
        {
            H26_T4_EnterFault(H26_T4_FAULT_TIMEOUT);
            return H26_T4_RESULT_FAULT;
        }
        if (H26_T4_GetDistanceCmFromStart() >= H26_T4_STRAIGHT_DISTANCE_CM)
        {
            s_curveObserved = 0U;
            s_state = H26_T4_DRIVE_CURVE;
        }
        return H26_T4_RESULT_RUNNING;

    case H26_T4_DRIVE_CURVE:
    {
        float turnCmd = H26_T4_ApplyDriveLineControl(
            H26_T4_CURVE_TURN_LIMIT_CMPS,
            H26_T4_DRIVE_CURVE_SPEED_CMPS);
        float turnAbs = H26_T4_AbsFloat(turnCmd);

        H26_T4_UpdateBallControl(nowMs);
        if ((nowMs - s_startMs) >= H26_T4_DRIVE_TIMEOUT_MS)
        {
            H26_T4_EnterFault(H26_T4_FAULT_TIMEOUT);
            return H26_T4_RESULT_FAULT;
        }

        /* See the curve first, then use the low-turn line command as its exit. */
        if (s_curveObserved == 0U)
        {
            if (turnAbs >= H26_T4_CURVE_ENTER_TURN_CMPS)
            {
                s_curveObserved = 1U;
            }
        }
        else if (turnAbs <= H26_T4_CURVE_EXIT_TURN_CMPS)
        {
            H26_T4_StopCar();
            s_state = H26_T4_DONE;
            return H26_T4_RESULT_FINISHED;
        }
        return H26_T4_RESULT_RUNNING;
    }

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
}

void H26_Task4_HoldBall10ms(uint32_t nowMs)
{
    if (s_state != H26_T4_DONE)
    {
        return;
    }

    /* B has been reached: traction remains off, while rod PID handles coast. */
    H26_T4_StopCar();
    H26_T4_UpdateBallControl(nowMs);
}

H26_Task4State_t H26_Task4_GetState(void) { return s_state; }
H26_Task4Fault_t H26_Task4_GetFault(void) { return s_fault; }

uint32_t H26_Task4_GetElapsedMs(uint32_t nowMs)
{
    if (s_state == H26_T4_IDLE)
    {
        return 0U;
    }
    return nowMs - s_startMs;
}

float H26_Task4_GetBallPositionCm(void)
{
    return H26_BallControl_GetPositionCm();
}

float H26_Task4_GetBallErrorCm(void)
{
    return H26_BallControl_GetErrorCm();
}

float H26_Task4_GetBallSpeedCmps(void)
{
    return H26_BallControl_GetBallSpeedCmps();
}

float H26_Task4_GetTiltCommandMm(void)
{
    return H26_BallControl_GetTiltCommandMm();
}

float H26_Task4_GetPidTiltCommandMm(void)
{
    return H26_BallControl_GetPidTiltCommandMm();
}

float H26_Task4_GetFeedForwardTiltMm(void)
{
    return H26_BallControl_GetFeedForwardTiltMm();
}

float H26_Task4_GetForwardSpeedCmps(void)
{
    return g_forwardSpeed;
}

float H26_Task4_GetForwardAccelerationCmps2(void)
{
    return H26_BallControl_GetForwardAccelerationCmps2();
}

float H26_Task4_GetDistanceCm(void)
{
    return H26_T4_GetDistanceCmFromStart();
}

float H26_Task4_GetCommandForwardSpeedCmps(void)
{
    return s_commandForwardSpeedCmps;
}

int32_t H26_Task4_GetRodEncoderCount(void)
{
    return H26_BallControl_GetRodEncoderCount();
}

int32_t H26_Task4_GetRodTargetCount(void)
{
    return H26_BallControl_GetRodTargetCount();
}

uint8_t H26_Task4_IsVisionValid(void)
{
    return H26_BallControl_IsVisionValid();
}

uint8_t H26_Task4_IsOriginCalibrated(void)
{
    return H26_BallControl_IsOriginCalibrated();
}

uint8_t H26_Task4_GetVisionConfidence(void)
{
    return H26_BallControl_GetConfidence();
}
