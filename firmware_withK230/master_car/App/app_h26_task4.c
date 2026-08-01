#include "app_h26_task4.h"
#include "app_h26_task6.h"
#include "app_h26_ball_control.h"
#include "app_h26_config.h"
#include "app_h26_task2.h"
#include "app_car_state.h"
#include "app_control.h"
#include <stdint.h>

static volatile H26_Task4State_t s_state = H26_T4_IDLE;
static volatile H26_Task4Fault_t s_fault = H26_T4_FAULT_NONE;
static volatile uint32_t s_startMs = 0U;
static volatile uint32_t s_chassisStartMs = 0U;
static volatile uint32_t s_finalElapsedMs = 0U;
static volatile float s_absoluteDistanceCm = 13.00f;
static volatile uint16_t s_taskOriginCentiCm = 0U;
static volatile float s_controlTargetCm = 0.0f;
static volatile float s_filteredKp = 1.00f;
static volatile float s_filteredKi = 0.09f;
static volatile float s_filteredKd = 0.95f;
static volatile float s_filteredKff = 0.60f;

static float H26_T4_ClampFloat(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static H26_BallControlSample_t H26_T4_UpdateBallControl(uint32_t nowMs, uint8_t mode)
{
    H26_Task6ScheduledControl_t sched;
    float desiredTargetCm, delta, finalKd;
    float ffSign, forwardAccel, plannedFf, encoderFf, ffTiltMm;
    uint32_t elapsedMs;
    uint8_t isCurve;

    sched = H26_T6_GetScheduledControl(s_absoluteDistanceCm);

    s_filteredKp  += H26_T6_SCHEDULE_FILTER_ALPHA * (sched.kp - s_filteredKp);
    s_filteredKi  += H26_T6_SCHEDULE_FILTER_ALPHA * (sched.ki - s_filteredKi);
    s_filteredKff  = sched.feedForwardK;

    isCurve = (mode == 2U && H26_Task2_IsCurveMode() != 0U) ? 1U : 0U;
    if (isCurve != 0U)
        s_filteredKd += H26_T6_SCHEDULE_FILTER_ALPHA * (sched.kdCurve - s_filteredKd);
    else
        s_filteredKd += H26_T6_SCHEDULE_FILTER_ALPHA * (sched.kdStraight - s_filteredKd);
    finalKd = s_filteredKd;

    desiredTargetCm = H26_T6_O_TARGET_CM + sched.positionCompensationCm;

    delta = desiredTargetCm - s_controlTargetCm;
    if (delta > H26_T6_TARGET_SLEW_CM_PER_TICK)
        s_controlTargetCm += H26_T6_TARGET_SLEW_CM_PER_TICK;
    else if (delta < -H26_T6_TARGET_SLEW_CM_PER_TICK)
        s_controlTargetCm -= H26_T6_TARGET_SLEW_CM_PER_TICK;
    else
        s_controlTargetCm = desiredTargetCm;

    ffSign = H26_T4_FF_TILT_SIGN_FOR_FORWARD_ACCEL;
    plannedFf = ffSign * H26_T6_FF_K_MM_PER_CMPS2 * H26_T6_LAUNCH_ACCEL_CMPS2;
    plannedFf = H26_T4_ClampFloat(plannedFf, -H26_T6_FF_TILT_LIMIT_MM, H26_T6_FF_TILT_LIMIT_MM);

    if (mode == 2U) {
        (void)H26_BallControl_UpdateEncoderFeedForward(nowMs);
        forwardAccel = H26_BallControl_GetForwardAccelerationCmps2();
        if (forwardAccel > 0.0f)
            encoderFf = ffSign * H26_T6_FF_K_MM_PER_CMPS2 * forwardAccel;
        else
            encoderFf = 0.0f;
        encoderFf = H26_T4_ClampFloat(encoderFf, -H26_T6_FF_TILT_LIMIT_MM, H26_T6_FF_TILT_LIMIT_MM);

        elapsedMs = nowMs - s_chassisStartMs;
        if (elapsedMs < H26_T6_FF_HANDOFF_MS)
            ffTiltMm = plannedFf + (encoderFf - plannedFf) * (float)elapsedMs / (float)H26_T6_FF_HANDOFF_MS;
        else
            ffTiltMm = encoderFf;

        H26_BallControl_SetIntegralFrozen(
            ((nowMs - s_chassisStartMs) < H26_T6_INTEGRAL_FREEZE_MS) ? 1U : 0U);
    } else if (mode == 1U) {
        H26_BallControl_SetIntegralFrozen(1U);
        ffTiltMm = plannedFf;
    } else {
        H26_BallControl_SetIntegralFrozen(0U);
        ffTiltMm = 0.0f;
    }

    return H26_BallControl_Task10msWithPidFeedForward(nowMs,
        s_controlTargetCm, s_filteredKp, s_filteredKi, finalKd,
        H26_T6_BALL_INTEGRAL_LIMIT_CM_S, H26_T6_BALL_TILT_COMMAND_LIMIT_MM,
        ffTiltMm, H26_T6_BALL_POSITION_DEADBAND_CM);
}

static void H26_T4_EnterFault(H26_Task4Fault_t fault)
{
    App_Control_ForcePWMZero();
    H26_Task2_ForceFault();
    H26_BallControl_Stop();
    s_fault = fault;
    s_state = H26_T4_FAULT;
}

void H26_Task4_Init(void) { H26_Task4_Reset(); }

void H26_Task4_Reset(void)
{
    App_Control_ForcePWMZero();
    H26_Task2_Reset();
    H26_BallControl_Reset();
    H26_BallControl_ResetEncoderFeedForward(0U);
    s_state = H26_T4_IDLE;
    s_fault = H26_T4_FAULT_NONE;
    s_startMs = 0U;
    s_chassisStartMs = 0U;
    s_finalElapsedMs = 0U;
    s_taskOriginCentiCm = 0U;
    s_controlTargetCm = 0.0f;
    s_filteredKp  = 1.00f;
    s_filteredKi  = 0.09f;
    s_filteredKd  = 0.95f;
    s_filteredKff = 0.60f;
}

/* -- K3: no traction, ball-PID + encoder FF by hand push -- */
void H26_Task4_Start(uint32_t startMs)
{
    H26_Task4_Reset();
    H26_BallControl_Start();
    H26_BallControl_ResetEncoderFeedForward(startMs);
    s_startMs = startMs;
    s_state = H26_T4_MANUAL_MOVE_HOLD_O;
}

/* -- K2: 120 cm straight + one curve, shared T6 framework -- */
void H26_Task4_StartDrive(uint32_t startMs)
{
    H26_Task4_Reset();
    H26_BallControl_Start();
    H26_BallControl_ResetEncoderFeedForward(startMs);
    s_startMs = startMs;
    s_state = H26_T4_DRIVE_ACQUIRE_O;
}

void H26_Task4_ForceFault(void)
{
    H26_T4_EnterFault((s_fault == H26_T4_FAULT_NONE) ?
        H26_T4_FAULT_ILLEGAL_STATE : s_fault);
}

H26_Task4Result_t H26_Task4_Task10ms(uint32_t nowMs)
{
    H26_BallControlSample_t sample;
    H26_Task2Result_t chassisResult;

    switch (s_state)
    {
    case H26_T4_MANUAL_MOVE_HOLD_O:
        App_Control_ForcePWMZero();
        (void)H26_T4_UpdateBallControl(nowMs, 0U);
        return H26_T4_RESULT_RUNNING;

    case H26_T4_DRIVE_ACQUIRE_O:
        App_Control_ForcePWMZero();
        (void)H26_BallControl_Observe10ms(nowMs);
        if (H26_BallControl_IsOriginCalibrated() != 0U) {
            s_taskOriginCentiCm = H26_BallControl_GetOriginCentiCm();
            s_absoluteDistanceCm = (float)s_taskOriginCentiCm / 100.0f;
            s_chassisStartMs = nowMs;
            s_state = H26_T4_PRETILT;
        }
        break;

    case H26_T4_PRETILT:
        App_Control_ForcePWMZero();
        sample = H26_T4_UpdateBallControl(nowMs, 1U);
        (void)sample;
        if ((nowMs - s_chassisStartMs) >= H26_T6_PRETILT_MS) {
            s_chassisStartMs = nowMs;
            H26_Task2_StartForTask5(s_chassisStartMs);
            H26_BallControl_ResetEncoderFeedForward(nowMs);
            s_state = H26_T4_DRIVE_STRAIGHT;
        }
        break;

    case H26_T4_DRIVE_STRAIGHT:
    case H26_T4_DRIVE_CURVE:
        sample = H26_T4_UpdateBallControl(nowMs, 2U);
        (void)sample;
        {
            uint32_t rampMs = nowMs - s_chassisStartMs;
            float t4Speed;

            if (rampMs < H26_T6_STRAIGHT_ACCEL_RAMP_MS)
                t4Speed = H26_T4_STRAIGHT_SPEED_CMPS * (float)rampMs /
                    (float)H26_T6_STRAIGHT_ACCEL_RAMP_MS;
            else
                t4Speed = H26_T4_STRAIGHT_SPEED_CMPS;

            H26_Task2_SetForwardSpeedLimitRaw(t4Speed);
        }
        chassisResult = H26_Task2_Task10ms(nowMs);

        if (chassisResult == H26_T2_RESULT_FAULT) {
            H26_T4_EnterFault(H26_T4_FAULT_ILLEGAL_STATE);
            return H26_T4_RESULT_FAULT;
        }

        /* Curve entry: freeze timer, keep driving. */
        if (s_state == H26_T4_DRIVE_STRAIGHT && H26_Task2_IsCurveMode() != 0U) {
            if (s_finalElapsedMs == 0U)
                s_finalElapsedMs = nowMs - s_startMs;
            s_state = H26_T4_DRIVE_CURVE;
        }

        /* Curve exit → DONE. */
        if (s_state == H26_T4_DRIVE_CURVE && H26_Task2_IsCurveMode() == 0U) {
            App_Control_ForcePWMZero();
            s_state = H26_T4_DONE;
            return H26_T4_RESULT_FINISHED;
        }
        break;

    case H26_T4_DONE:
        App_Control_ForcePWMZero();
        return H26_T4_RESULT_FINISHED;

    case H26_T4_FAULT:
        App_Control_ForcePWMZero();
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
    App_Control_ForcePWMZero();
    (void)H26_T4_UpdateBallControl(nowMs, 0U);
}

H26_Task4State_t H26_Task4_GetState(void)          { return s_state; }
H26_Task4Fault_t H26_Task4_GetFault(void)          { return s_fault; }
uint32_t H26_Task4_GetElapsedMs(uint32_t nowMs)
{
    if (s_state == H26_T4_IDLE) return 0U;
    if (s_finalElapsedMs != 0U || s_state == H26_T4_DONE) return s_finalElapsedMs;
    return nowMs - s_startMs;
}
float H26_Task4_GetBallPositionCm(void)             { return H26_BallControl_GetPositionCm(); }
float H26_Task4_GetBallErrorCm(void)                { return H26_BallControl_GetErrorCm(); }
float H26_Task4_GetBallSpeedCmps(void)              { return H26_BallControl_GetBallSpeedCmps(); }
float H26_Task4_GetTiltCommandMm(void)              { return H26_BallControl_GetTiltCommandMm(); }
float H26_Task4_GetPidTiltCommandMm(void)           { return H26_BallControl_GetPidTiltCommandMm(); }
float H26_Task4_GetFeedForwardTiltMm(void)          { return H26_BallControl_GetFeedForwardTiltMm(); }
float H26_Task4_GetForwardSpeedCmps(void)           { return g_forwardSpeed; }
float H26_Task4_GetForwardAccelerationCmps2(void)   { return H26_BallControl_GetForwardAccelerationCmps2(); }
float H26_Task4_GetDistanceCm(void)
{
    return (float)(g_forwardEncoderTotal > 0 ? g_forwardEncoderTotal : 0) * ECAR_CM_PER_PULSE;
}
float H26_Task4_GetCommandForwardSpeedCmps(void)    { return H26_Task2_GetCommandForwardSpeed(); }
int32_t H26_Task4_GetRodEncoderCount(void)          { return H26_BallControl_GetRodEncoderCount(); }
int32_t H26_Task4_GetRodTargetCount(void)           { return H26_BallControl_GetRodTargetCount(); }
uint8_t H26_Task4_IsVisionValid(void)               { return H26_BallControl_IsVisionValid(); }
uint8_t H26_Task4_IsOriginCalibrated(void)          { return H26_BallControl_IsOriginCalibrated(); }
uint8_t H26_Task4_GetVisionConfidence(void)          { return H26_BallControl_GetConfidence(); }
