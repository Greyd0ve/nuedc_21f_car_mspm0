#include "app_h26_task6.h"
#include "app_h26_ball_control.h"
#include "app_h26_config.h"
#include "app_h26_task2.h"
#include "app_car_state.h"
#include "app_control.h"
#include <stdint.h>

static volatile H26_Task6State_t s_state = H26_T6_IDLE;
static volatile H26_Task6Fault_t s_fault = H26_T6_FAULT_NONE;
static volatile uint32_t s_startMs = 0U;
static volatile uint32_t s_chassisStartMs = 0U;
static volatile uint32_t s_finalElapsedMs = 0U;
static volatile float s_ballPeakErrorCm = 0.0f;
static volatile float s_absoluteDistanceCm = 13.00f;
static volatile uint16_t s_taskOriginCentiCm = 0U;
static volatile float s_controlTargetCm = 0.0f;
static volatile float s_filteredKp = 1.00f;
static volatile float s_filteredKi = 0.09f;
static volatile float s_filteredKd = 0.95f;
static volatile float s_filteredKff = 0.60f;
static volatile uint8_t s_saturationActive = 0U;
/* sat_l: latched -- set once and never cleared until Reset. */
static volatile uint8_t s_saturationLatched = 0U;

static float H26_T6_ClampFloat(float value, float lower, float upper)
{
    if (value < lower) { return lower; }
    if (value > upper) { return upper; }
    return value;
}

static float H26_T6_Sqrtf(float x)
{
    float r; int32_t i;
    if (x <= 0.0f) return 0.0f;
    r = x; i = *(int32_t *)&r; i = 0x1FC00000 + (i >> 1); r = *(float *)&i;
    r = 0.5f * (r + x / r); r = 0.5f * (r + x / r); r = 0.5f * (r + x / r);
    return r;
}

/* -------------------------------------------------------
 * Cruise compensation -- 6-point piecewise linear LUT
 * ----------------------------------------------------- */
static float H26_T6_GetCruiseCompensationCm(float absoluteDistanceCm)
{
    static const float distanceTable[6] =
    { 3.10f, 8.20f, 13.00f, 17.00f, 18.00f, 21.00f };
    static const float compensationTable[6] =
    { -0.24f, -0.17f, -0.14f, -0.46f, -0.11f, 0.83f };

    uint8_t idx; float d, ratio, c;

    d = H26_T6_ClampFloat(absoluteDistanceCm, 3.10f, 21.00f);
    for (idx = 0U; idx < 5U; idx++)
    {
        if (d <= distanceTable[idx + 1U])
        {
            ratio = (d - distanceTable[idx]) /
                (distanceTable[idx + 1U] - distanceTable[idx]);
            c = compensationTable[idx] +
                ratio * (compensationTable[idx + 1U] - compensationTable[idx]);
            return H26_T6_ClampFloat(c, -0.50f, 0.85f);
        }
    }
    return compensationTable[5];
}

/* -------------------------------------------------------
 * End-of-lap dynamic compensation: linear (D in cm)
 *   EndComp(D) = 1.8517 - 0.07763 * D
 * ----------------------------------------------------- */
static float H26_T6_GetEndCompensationCm(float absoluteDistanceCm)
{
    float d, c;
    d = H26_T6_ClampFloat(absoluteDistanceCm, 3.10f, 21.00f);
    c = 1.8517f - 0.07763f * d;
    return H26_T6_ClampFloat(c, H26_T6_END_COMP_MIN_CM, H26_T6_END_COMP_MAX_CM);
}

/* -------------------------------------------------------
 * End-comp blend: 0 -> 1 ramp between START_MS and FULL_MS
 * ----------------------------------------------------- */
static float H26_T6_GetEndCompensationBlend(uint32_t elapsedMs)
{
    if (elapsedMs <= H26_T6_END_COMP_START_MS) return 0.0f;
    if (elapsedMs >= H26_T6_END_COMP_FULL_MS)  return 1.0f;
    return (float)(elapsedMs - H26_T6_END_COMP_START_MS) /
           (float)(H26_T6_END_COMP_FULL_MS - H26_T6_END_COMP_START_MS);
}

/* ---- Kp / Ki / Kd / Kff scheduling (unchanged) ---- */

static float H26_T6_GetScheduledKp(float absoluteDistanceCm)
{
    float d, delta, kp;
    d = H26_T6_ClampFloat(absoluteDistanceCm, H26_T6_MIN_DISTANCE_CM, H26_T6_MAX_DISTANCE_CM);
    delta = d - 13.00f;
    kp = 1.00f + 0.003754f * delta + 0.001093f * delta * delta;
    return H26_T6_ClampFloat(kp, 1.00f, 1.10f);
}

H26_Task6ScheduledControl_t H26_T6_GetScheduledControl(float absoluteDistanceCm)
{
    H26_Task6ScheduledControl_t result;
    float d, kp;

    d  = H26_T6_ClampFloat(absoluteDistanceCm, H26_T6_MIN_DISTANCE_CM, H26_T6_MAX_DISTANCE_CM);
    kp = H26_T6_GetScheduledKp(d);

    result.distanceCm = d;
    result.normalizedDistance = (d - H26_T6_MIN_DISTANCE_CM) /
        (H26_T6_MAX_DISTANCE_CM - H26_T6_MIN_DISTANCE_CM);
    result.positionCompensationCm = H26_T6_GetCruiseCompensationCm(d);
    result.kp = kp;
    result.ki = H26_T6_ClampFloat(0.09f * kp, 0.09f, 0.10f);
    result.kdStraight = H26_T6_ClampFloat(0.95f * H26_T6_Sqrtf(kp), 0.95f, 1.00f);
    result.kdCurve    = H26_T6_ClampFloat(1.05f * H26_T6_Sqrtf(kp), 1.05f, 1.11f);
    result.feedForwardK = H26_T6_FF_K_MM_PER_CMPS2;
    return result;
}

/* ---- Combined ball + chassis control update ---- */

static H26_BallControlSample_t H26_T6_UpdateBallControl(uint32_t nowMs, uint8_t mode)
{
    H26_Task6ScheduledControl_t sched;
    float cruiseC, endC, endB, finalC, desiredTargetCm, delta;
    float plannedFeedForwardTiltMm, encoderFeedForwardTiltMm, feedForwardTiltMm;
    float forwardAccel, finalKd, ffSign;
    uint32_t elapsedMs, taskElapsed;
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

    /* Combine cruise + end-of-lap compensation */
    cruiseC = sched.positionCompensationCm;
    taskElapsed = H26_Task6_GetElapsedMs(nowMs);
    endC  = H26_T6_GetEndCompensationCm(s_absoluteDistanceCm);
    endB  = H26_T6_GetEndCompensationBlend(taskElapsed);
    finalC = cruiseC + endB * endC;
    finalC = H26_T6_ClampFloat(finalC, H26_T6_FINAL_COMP_MIN_CM, H26_T6_FINAL_COMP_MAX_CM);
    desiredTargetCm = H26_T6_O_TARGET_CM + finalC;

    delta = desiredTargetCm - s_controlTargetCm;
    if (delta > H26_T6_TARGET_SLEW_CM_PER_TICK)
        s_controlTargetCm += H26_T6_TARGET_SLEW_CM_PER_TICK;
    else if (delta < -H26_T6_TARGET_SLEW_CM_PER_TICK)
        s_controlTargetCm -= H26_T6_TARGET_SLEW_CM_PER_TICK;
    else
        s_controlTargetCm = desiredTargetCm;

    ffSign = H26_T4_FF_TILT_SIGN_FOR_FORWARD_ACCEL;
    plannedFeedForwardTiltMm = ffSign * H26_T6_FF_K_MM_PER_CMPS2 * H26_T6_LAUNCH_ACCEL_CMPS2;
    plannedFeedForwardTiltMm = H26_T6_ClampFloat(plannedFeedForwardTiltMm,
        -H26_T6_FF_TILT_LIMIT_MM, H26_T6_FF_TILT_LIMIT_MM);

    if (mode == 2U) {
        (void)H26_BallControl_UpdateEncoderFeedForward(nowMs);
        forwardAccel = H26_BallControl_GetForwardAccelerationCmps2();
        if (forwardAccel > 0.0f)
            encoderFeedForwardTiltMm = ffSign * H26_T6_FF_K_MM_PER_CMPS2 * forwardAccel;
        else
            encoderFeedForwardTiltMm = 0.0f;
        encoderFeedForwardTiltMm = H26_T6_ClampFloat(encoderFeedForwardTiltMm,
            -H26_T6_FF_TILT_LIMIT_MM, H26_T6_FF_TILT_LIMIT_MM);

        elapsedMs = nowMs - s_chassisStartMs;
        if (elapsedMs < H26_T6_FF_HANDOFF_MS)
            feedForwardTiltMm = plannedFeedForwardTiltMm +
                (encoderFeedForwardTiltMm - plannedFeedForwardTiltMm) *
                (float)elapsedMs / (float)H26_T6_FF_HANDOFF_MS;
        else
            feedForwardTiltMm = encoderFeedForwardTiltMm;

        H26_BallControl_SetIntegralFrozen(
            ((nowMs - s_chassisStartMs) < H26_T6_INTEGRAL_FREEZE_MS) ? 1U : 0U);
    } else if (mode == 1U) {
        H26_BallControl_SetIntegralFrozen(1U);
        feedForwardTiltMm = plannedFeedForwardTiltMm;
    } else {
        H26_BallControl_SetIntegralFrozen(0U);
        feedForwardTiltMm = 0.0f;
    }

    return H26_BallControl_Task10msWithPidFeedForward(nowMs,
        s_controlTargetCm, s_filteredKp, s_filteredKi, finalKd,
        H26_T6_BALL_INTEGRAL_LIMIT_CM_S, H26_T6_BALL_TILT_COMMAND_LIMIT_MM,
        feedForwardTiltMm, H26_T6_BALL_POSITION_DEADBAND_CM);
}

/* ---- Fault / init / reset / start ---- */

static void H26_T6_EnterFault(H26_Task6Fault_t fault)
{
    App_Control_ForcePWMZero();
    H26_Task2_ForceFault();
    H26_BallControl_Stop();
    s_fault = fault;
    s_state = H26_T6_FAULT;
}

void H26_Task6_Init(void) { H26_Task6_Reset(); }

void H26_Task6_Reset(void)
{
    App_Control_ForcePWMZero();
    H26_Task2_Reset();
    H26_BallControl_Reset();
    H26_BallControl_ResetEncoderFeedForward(0U);
    s_state = H26_T6_IDLE;
    s_fault = H26_T6_FAULT_NONE;
    s_startMs = 0U;
    s_chassisStartMs = 0U;
    s_finalElapsedMs = 0U;
    s_ballPeakErrorCm = 0.0f;
    s_taskOriginCentiCm = 0U;
    s_controlTargetCm = 0.0f;
    s_filteredKp  = 1.00f;
    s_filteredKi  = 0.09f;
    s_filteredKd  = 0.95f;
    s_filteredKff = 0.60f;
    s_saturationActive  = 0U;
    s_saturationLatched  = 0U;
}

void H26_Task6_Start(uint32_t startMs)
{
    H26_Task6_Reset();
    H26_BallControl_Start();
    H26_BallControl_ResetEncoderFeedForward(startMs);
    s_startMs = startMs;
    s_state = H26_T6_ACQUIRE_REFERENCE;
}

void H26_Task6_ForceFault(void)
{
    H26_T6_EnterFault((s_fault == H26_T6_FAULT_NONE) ?
        H26_T6_FAULT_ILLEGAL_STATE : s_fault);
}

H26_Task6Result_t H26_Task6_Task10ms(uint32_t nowMs)
{
    H26_BallControlSample_t sample;
    H26_Task2Result_t chassisResult;
    float errorAbs;

    switch (s_state)
    {
    case H26_T6_ACQUIRE_REFERENCE:
        App_Control_ForcePWMZero();
        (void)H26_BallControl_Observe10ms(nowMs);
        if (H26_BallControl_IsOriginCalibrated() != 0U) {
            s_taskOriginCentiCm = H26_BallControl_GetOriginCentiCm();
            s_absoluteDistanceCm = (float)s_taskOriginCentiCm / 100.0f;
            s_chassisStartMs = nowMs;
            s_state = H26_T6_PRETILT;
        }
        break;

    case H26_T6_PRETILT:
        App_Control_ForcePWMZero();
        sample = H26_T6_UpdateBallControl(nowMs, 1U);
        if (sample == H26_BALL_SAMPLE_NEW) {
            errorAbs = (H26_BallControl_GetPositionCm() < 0.0f) ?
                -H26_BallControl_GetPositionCm() : H26_BallControl_GetPositionCm();
            if (errorAbs > s_ballPeakErrorCm) s_ballPeakErrorCm = errorAbs;
        }
        if ((nowMs - s_chassisStartMs) >= H26_T6_PRETILT_MS) {
            s_chassisStartMs = nowMs;
            H26_Task2_StartForTask5(s_chassisStartMs);
            H26_BallControl_ResetEncoderFeedForward(nowMs);
            s_state = H26_T6_LEAVE_A;
        }
        break;

    case H26_T6_LEAVE_A:
    case H26_T6_LAP_RUNNING:
        sample = H26_T6_UpdateBallControl(nowMs, 2U);
        if (sample == H26_BALL_SAMPLE_NEW) {
            float tiltAbs;
            errorAbs = (H26_BallControl_GetPositionCm() < 0.0f) ?
                -H26_BallControl_GetPositionCm() : H26_BallControl_GetPositionCm();
            if (errorAbs > s_ballPeakErrorCm) s_ballPeakErrorCm = errorAbs;
            tiltAbs = (H26_BallControl_GetTiltCommandMm() < 0.0f) ?
                -H26_BallControl_GetTiltCommandMm() : H26_BallControl_GetTiltCommandMm();
            if (tiltAbs >= H26_T6_BALL_TILT_COMMAND_LIMIT_MM) {
                s_saturationActive  = 1U;
                s_saturationLatched = 1U;
            } else {
                s_saturationActive = 0U;
            }
        }
        {
            uint32_t rampMs = nowMs - s_chassisStartMs;
            if (rampMs < H26_T6_STRAIGHT_ACCEL_RAMP_MS)
                H26_Task2_SetForwardSpeedLimit(H26_T6_STRAIGHT_SPEED_CMPS * (float)rampMs /
                    (float)H26_T6_STRAIGHT_ACCEL_RAMP_MS);
            else
                H26_Task2_SetForwardSpeedLimit(H26_T6_STRAIGHT_SPEED_CMPS);
        }
        chassisResult = H26_Task2_Task10ms(nowMs);
        if (H26_Task2_GetState() == H26_T2_LAP_RUNNING) s_state = H26_T6_LAP_RUNNING;
        if (chassisResult == H26_T2_RESULT_FINISHED) {
            if (s_finalElapsedMs == 0U) s_finalElapsedMs = H26_Task2_GetFinalElapsedMs();
            s_state = H26_T6_DONE;
            return H26_T6_RESULT_FINISHED;
        }
        if (chassisResult == H26_T2_RESULT_FAULT) {
            H26_T6_EnterFault(H26_T6_FAULT_ILLEGAL_STATE);
            return H26_T6_RESULT_FAULT;
        }
        if (s_finalElapsedMs == 0U && H26_Task2_GetState() == H26_T2_BRAKING)
            s_finalElapsedMs = nowMs - s_startMs;
        break;

    case H26_T6_DONE:
        App_Control_ForcePWMZero();
        return H26_T6_RESULT_FINISHED;
    case H26_T6_FAULT:
        App_Control_ForcePWMZero();
        H26_BallControl_Stop();
        return H26_T6_RESULT_FAULT;
    case H26_T6_IDLE:
    case H26_T6_TARGET_TRANSITION:
    default:
        H26_T6_EnterFault(H26_T6_FAULT_ILLEGAL_STATE);
        return H26_T6_RESULT_FAULT;
    }
    return H26_T6_RESULT_RUNNING;
}

void H26_Task6_HoldBall10ms(uint32_t nowMs)
{
    H26_BallControlSample_t sample = H26_T6_UpdateBallControl(nowMs, 0U);
    float errorAbs;
    if (sample == H26_BALL_SAMPLE_NEW) {
        errorAbs = (H26_BallControl_GetPositionCm() < 0.0f) ?
            -H26_BallControl_GetPositionCm() : H26_BallControl_GetPositionCm();
        if (errorAbs > s_ballPeakErrorCm) s_ballPeakErrorCm = errorAbs;
    }
}

H26_Task6State_t  H26_Task6_GetState(void)             { return s_state; }
H26_Task6Fault_t  H26_Task6_GetFault(void)             { return s_fault; }
uint32_t H26_Task6_GetElapsedMs(uint32_t nowMs)
{
    if (s_state == H26_T6_IDLE) return 0U;
    if (s_finalElapsedMs != 0U || s_state == H26_T6_DONE) return s_finalElapsedMs;
    return nowMs - s_startMs;
}
uint32_t H26_Task6_GetFinalElapsedMs(void)              { return s_finalElapsedMs; }
float H26_Task6_GetBallPeakErrorCm(void)                { return s_ballPeakErrorCm; }
float H26_Task6_GetBallPositionCm(void)     { return H26_BallControl_GetPositionCm(); }
float H26_Task6_GetBallErrorCm(void)        { return H26_BallControl_GetErrorCm(); }
float H26_Task6_GetBallSpeedCmps(void)      { return H26_BallControl_GetBallSpeedCmps(); }
float H26_Task6_GetTiltCommandMm(void)      { return H26_BallControl_GetTiltCommandMm(); }
float H26_Task6_GetPidTiltCommandMm(void)   { return H26_BallControl_GetPidTiltCommandMm(); }
float H26_Task6_GetFeedForwardTiltMm(void)  { return H26_BallControl_GetFeedForwardTiltMm(); }
float H26_Task6_GetForwardSpeedCmps(void)   { return g_forwardSpeed; }
float H26_Task6_GetForwardAccelerationCmps2(void) { return H26_BallControl_GetForwardAccelerationCmps2(); }
int32_t H26_Task6_GetRodEncoderCount(void)  { return H26_BallControl_GetRodEncoderCount(); }
int32_t H26_Task6_GetRodTargetCount(void)   { return H26_BallControl_GetRodTargetCount(); }
uint8_t H26_Task6_IsVisionValid(void)       { return H26_BallControl_IsVisionValid(); }
uint8_t H26_Task6_IsCurveMode(void)         { return H26_Task2_IsCurveMode(); }
float H26_Task6_GetScheduledKp(void)        { return s_filteredKp; }
float H26_Task6_GetScheduledKi(void)        { return s_filteredKi; }
float H26_Task6_GetScheduledKd(void)        { return s_filteredKd; }
float H26_Task6_GetScheduledFeedForwardK(void){ return s_filteredKff; }
float H26_Task6_GetAbsoluteDistanceCm(void)  { return s_absoluteDistanceCm; }
float H26_Task6_GetControlTargetCm(void)     { return s_controlTargetCm; }
uint8_t H26_Task6_IsSaturationActive(void)   { return s_saturationActive; }
uint8_t H26_Task6_IsSaturationLatched(void)   { return s_saturationLatched; }
