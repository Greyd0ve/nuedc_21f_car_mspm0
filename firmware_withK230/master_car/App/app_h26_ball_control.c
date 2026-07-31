#include "app_h26_ball_control.h"
#include "app_ball_link.h"
#include "app_h26_config.h"
#include "app_car_state.h"
#include "RodEncoder.h"
#include "RodStepper.h"
#include <stdint.h>

static volatile uint16_t s_rawPositionCentiCm = 0U;
static volatile uint16_t s_originCentiCm = 0U;
static volatile uint8_t s_originCalibrated = 0U;
static volatile float s_positionCm = 0.0f;
static volatile float s_ballSpeedCmps = 0.0f;
static volatile float s_targetCm = 0.0f;
static volatile float s_errorCm = 0.0f;
static volatile int32_t s_commandHz = 0;
static volatile uint8_t s_visionValid = 0U;
static volatile uint8_t s_confidence = 0U;
static volatile uint8_t s_lastFlags = 0U;
static volatile uint32_t s_frameAgeMs = 0xFFFFFFFFUL;
static volatile uint16_t s_lastSequence = 0U;
static volatile int32_t s_rodEncoderCount = 0;
static volatile int32_t s_rodTargetCount = 0;
static volatile float s_tiltCommandMm = 0.0f;
static volatile float s_pidTiltCommandMm = 0.0f;
static volatile float s_feedForwardTiltMm = 0.0f;
static volatile float s_errorIntegralCmS = 0.0f;
static volatile uint8_t s_integralFrozen = 0U;

static float s_previousControlErrorCm = 0.0f;
static uint16_t s_lastPacketSequence = 0U;
static uint32_t s_lastMeasurementMs = 0U;
static float s_lastPositionCm = 0.0f;
static uint16_t s_newMeasurementDtMs = 0U;
static uint8_t s_hasPacket = 0U;
static uint8_t s_hasReliableMeasurement = 0U;
static uint8_t s_hasPreviousControlError = 0U;
static float s_previousForwardSpeedCmps = 0.0f;
static float s_forwardAccelerationCmps2 = 0.0f;
static float s_encoderFeedForwardTiltMm = 0.0f;
static uint32_t s_lastFeedForwardMs = 0U;
static uint8_t s_feedForwardInitialized = 0U;

static float H26_Ball_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint32_t H26_Ball_AbsInt32ToU32(int32_t value)
{
    return (value < 0) ? ((uint32_t)(-(value + 1)) + 1U) : (uint32_t)value;
}

static float H26_Ball_ClampFloat(float value, float lower, float upper)
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

static uint8_t H26_Ball_ErrorCrossedZero(float previousError,
                                         float currentError)
{
    return ((previousError < 0.0f && currentError >= 0.0f) ||
            (previousError > 0.0f && currentError <= 0.0f)) ? 1U : 0U;
}

static int32_t H26_Ball_RoundFloatToInt32(float value)
{
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
        (int32_t)(value - 0.5f);
}

static uint8_t H26_Ball_CalibratePositionCm(uint16_t rawCentiCm,
                                             float *positionCm)
{
#if H26_T3_AUTO_ZERO_AT_TASK_START
    int32_t delta;

    if (positionCm == 0 || s_originCalibrated == 0U)
    {
        return 0U;
    }

    delta = ((int32_t)rawCentiCm - (int32_t)s_originCentiCm) *
        (int32_t)H26_T3_CAMERA_SIGN_FOR_POSITIVE_BALL;
    *positionCm = (float)delta / (float)H26_T3_CAMERA_CENTICM_PER_CM;
    return 1U;
#else
    int32_t delta;
    int32_t posDelta;
    int32_t negDelta;

    if (positionCm == 0)
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
#endif
}

static H26_BallControlSample_t H26_Ball_NoReliableMeasurement(void)
{
    s_visionValid = 0U;
    return H26_BALL_SAMPLE_HELD;
}

static H26_BallControlSample_t H26_Ball_UpdateMeasurement(uint32_t nowMs)
{
    BallLinkFrame_t frame;
    float positionCm;
    uint32_t dtMs;

    s_frameAgeMs = App_BallLink_GetFrameAgeMs(nowMs);
    if (App_BallLink_GetLatest(&frame) == 0U)
    {
        return H26_Ball_NoReliableMeasurement();
    }

    s_lastSequence = frame.sequence;
    s_lastFlags = frame.flags;
    s_confidence = frame.confidence;
    if (s_hasPacket != 0U && frame.sequence == s_lastPacketSequence)
    {
        return H26_Ball_NoReliableMeasurement();
    }
    s_hasPacket = 1U;
    s_lastPacketSequence = frame.sequence;

    if (frame.ballValid == 0U || frame.pipeValid == 0U ||
        frame.confidence < H26_T3_MIN_CONFIDENCE)
    {
        return H26_Ball_NoReliableMeasurement();
    }

#if H26_T3_AUTO_ZERO_AT_TASK_START
    if (s_originCalibrated == 0U)
    {
        s_originCentiCm = frame.positionCentiCm;
        s_originCalibrated = 1U;
    }
#endif

    if (H26_Ball_CalibratePositionCm(frame.positionCentiCm, &positionCm) == 0U)
    {
        return H26_Ball_NoReliableMeasurement();
    }

    if (s_hasReliableMeasurement != 0U)
    {
        dtMs = frame.receiveTimeMs - s_lastMeasurementMs;
        if (dtMs != 0U)
        {
            float rawSpeed = (positionCm - s_lastPositionCm) * 1000.0f /
                (float)dtMs;
            s_ballSpeedCmps = H26_T3_SPEED_FILTER_ALPHA * rawSpeed +
                (1.0f - H26_T3_SPEED_FILTER_ALPHA) * s_ballSpeedCmps;
            s_newMeasurementDtMs = (uint16_t)dtMs;
        }
        else
        {
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
    s_lastPositionCm = positionCm;
    s_hasReliableMeasurement = 1U;
    s_visionValid = 1U;
    return H26_BALL_SAMPLE_NEW;
}

static void H26_Ball_UpdateRodEncoderTelemetry(void)
{
    RodEncoderSnapshot_t snapshot;

    RodEncoder_GetSnapshot(&snapshot);
    s_rodEncoderCount = snapshot.count;
}

static int32_t H26_Ball_TiltMmToRodCount(float tiltMm)
{
    return H26_Ball_RoundFloatToInt32(tiltMm *
        H26_T3_ROD_ENCODER_COUNTS_PER_MM *
        (float)H26_T3_ROD_ENCODER_SIGN_FOR_POSITIVE_BALL);
}

void H26_BallControl_ResetEncoderFeedForward(uint32_t nowMs)
{
    s_previousForwardSpeedCmps = g_forwardSpeed;
    s_forwardAccelerationCmps2 = 0.0f;
    s_encoderFeedForwardTiltMm = 0.0f;
    s_lastFeedForwardMs = nowMs;
    s_feedForwardInitialized = 1U;
}

float H26_BallControl_UpdateEncoderFeedForward(uint32_t nowMs)
{
    float currentForwardSpeedCmps = g_forwardSpeed;
    float dtSeconds;
    float rawAccelerationCmps2;

#if !H26_T4_ENCODER_FF_ENABLE
    H26_BallControl_ResetEncoderFeedForward(nowMs);
    return 0.0f;
#else
    if (H26_Ball_AbsFloat(g_leftSpeed - g_rightSpeed) >
            H26_T4_FF_MAX_WHEEL_SPEED_DIFF_CMPS)
    {
        H26_BallControl_ResetEncoderFeedForward(nowMs);
        return 0.0f;
    }
    if (s_feedForwardInitialized == 0U)
    {
        H26_BallControl_ResetEncoderFeedForward(nowMs);
        return 0.0f;
    }
    if (nowMs == s_lastFeedForwardMs)
    {
        return s_encoderFeedForwardTiltMm;
    }

    dtSeconds = (float)(nowMs - s_lastFeedForwardMs) / 1000.0f;
    if (dtSeconds <= 0.0f || dtSeconds > 0.05f)
    {
        H26_BallControl_ResetEncoderFeedForward(nowMs);
        return 0.0f;
    }

    rawAccelerationCmps2 = H26_Ball_ClampFloat(
        (currentForwardSpeedCmps - s_previousForwardSpeedCmps) / dtSeconds,
        -H26_T4_FF_ACCEL_LIMIT_CMPS2, H26_T4_FF_ACCEL_LIMIT_CMPS2);
    s_forwardAccelerationCmps2 += H26_T4_FF_ACCEL_FILTER_ALPHA *
        (rawAccelerationCmps2 - s_forwardAccelerationCmps2);
    s_encoderFeedForwardTiltMm = (s_forwardAccelerationCmps2 > 0.0f) ?
        H26_Ball_ClampFloat(H26_T4_FF_TILT_SIGN_FOR_FORWARD_ACCEL *
            H26_T4_FF_K_MM_PER_CMPS2 * s_forwardAccelerationCmps2,
            -H26_T4_FF_TILT_LIMIT_MM, H26_T4_FF_TILT_LIMIT_MM) : 0.0f;
    s_previousForwardSpeedCmps = currentForwardSpeedCmps;
    s_lastFeedForwardMs = nowMs;
    return s_encoderFeedForwardTiltMm;
#endif
}

float H26_BallControl_GetForwardAccelerationCmps2(void)
{
    return s_forwardAccelerationCmps2;
}

void H26_BallControl_SetIntegralFrozen(uint8_t frozen)
{
    s_integralFrozen = (frozen != 0U) ? 1U : 0U;
}

static void H26_Ball_ApplyRodPositionControl(void)
{
    int32_t encoderError = s_rodTargetCount - s_rodEncoderCount;
    uint32_t encoderErrorAbs = H26_Ball_AbsInt32ToU32(encoderError);
    uint32_t commandHz;
    RodStepperDirection_t direction;

    if (encoderErrorAbs <= (uint32_t)H26_T3_ROD_POSITION_DEADBAND_COUNT)
    {
        H26_BallControl_Stop();
        return;
    }

    commandHz = (uint32_t)((float)encoderErrorAbs *
        H26_T3_ROD_POSITION_KP_HZ_PER_COUNT);
    if (commandHz < H26_T3_ROD_POSITION_MIN_HZ)
    {
        commandHz = H26_T3_ROD_POSITION_MIN_HZ;
    }
    if (commandHz > H26_T3_ROD_POSITION_MAX_HZ)
    {
        commandHz = H26_T3_ROD_POSITION_MAX_HZ;
    }

    if ((encoderError < 0 && H26_T3_ROD_ENCODER_SIGN_FOR_STEPPER_POSITIVE < 0) ||
        (encoderError > 0 && H26_T3_ROD_ENCODER_SIGN_FOR_STEPPER_POSITIVE > 0))
    {
        direction = ROD_STEPPER_DIR_POSITIVE;
        s_commandHz = (int32_t)commandHz;
    }
    else
    {
        direction = ROD_STEPPER_DIR_NEGATIVE;
        s_commandHz = -(int32_t)commandHz;
    }

    (void)RodStepper_SetVelocity(direction, commandHz);
}

void H26_BallControl_Init(void)
{
    H26_BallControl_Reset();
}

void H26_BallControl_Stop(void)
{
    RodStepper_Stop();
    s_commandHz = 0;
}

void H26_BallControl_Reset(void)
{
    H26_BallControl_Stop();
    s_rawPositionCentiCm = 0U;
    s_originCentiCm = 0U;
    s_originCalibrated = 0U;
    s_positionCm = 0.0f;
    s_ballSpeedCmps = 0.0f;
    s_targetCm = 0.0f;
    s_errorCm = 0.0f;
    s_visionValid = 0U;
    s_confidence = 0U;
    s_lastFlags = 0U;
    s_frameAgeMs = 0xFFFFFFFFUL;
    s_lastSequence = 0U;
    s_rodEncoderCount = 0;
    s_rodTargetCount = 0;
    s_tiltCommandMm = 0.0f;
    s_pidTiltCommandMm = 0.0f;
    s_feedForwardTiltMm = 0.0f;
    s_errorIntegralCmS = 0.0f;
    s_integralFrozen = 0U;
    s_previousControlErrorCm = 0.0f;
    s_lastPacketSequence = 0U;
    s_lastMeasurementMs = 0U;
    s_lastPositionCm = 0.0f;
    s_newMeasurementDtMs = 0U;
    s_hasPacket = 0U;
    s_hasReliableMeasurement = 0U;
    s_hasPreviousControlError = 0U;
    H26_BallControl_ResetEncoderFeedForward(0U);
}

void H26_BallControl_Start(void)
{
    H26_BallControl_Reset();
    App_BallLink_Reset();
    App_BallLink_ResetDiagnostics();
    RodEncoder_Reset();
}

H26_BallControlSample_t H26_BallControl_Observe10ms(uint32_t nowMs)
{
    H26_Ball_UpdateRodEncoderTelemetry();
    return H26_Ball_UpdateMeasurement(nowMs);
}

static H26_BallControlSample_t H26_BallControl_Task10msCore(
    uint32_t nowMs,
    float targetCm,
    float positionKpMmPerCm,
    float positionKiMmPerCmS,
    float speedKdMmPerCmps,
    float integralLimitCmS,
    float tiltCommandLimitMm,
    float feedForwardTiltMm)
{
    H26_BallControlSample_t sample;
    float tiltMm;

    H26_Ball_UpdateRodEncoderTelemetry();
    sample = H26_Ball_UpdateMeasurement(nowMs);
    if (sample == H26_BALL_SAMPLE_NEW)
    {
        s_targetCm = targetCm;
        s_errorCm = targetCm - s_positionCm;
        if (s_hasPreviousControlError != 0U &&
            H26_Ball_ErrorCrossedZero(s_previousControlErrorCm,
                                      s_errorCm) != 0U)
        {
            /* Do not let the old side's integral push through the O point. */
            s_errorIntegralCmS = 0.0f;
        }
        if (H26_Ball_AbsFloat(s_errorCm) <= H26_T3_TILT_DEADBAND_CM &&
            H26_Ball_AbsFloat(s_ballSpeedCmps) <= H26_T3_TILT_DEADBAND_SPEED_CMPS)
        {
            s_errorIntegralCmS = 0.0f;
            s_pidTiltCommandMm = 0.0f;
        }
        else
        {
            if (s_integralFrozen == 0U && positionKiMmPerCmS != 0.0f &&
                integralLimitCmS > 0.0f)
            {
                s_errorIntegralCmS = H26_Ball_ClampFloat(
                    s_errorIntegralCmS + s_errorCm *
                    ((float)H26_BallControl_GetStableSampleMs() / 1000.0f),
                    -integralLimitCmS, integralLimitCmS);
            }
            else if (positionKiMmPerCmS == 0.0f || integralLimitCmS <= 0.0f)
            {
                s_errorIntegralCmS = 0.0f;
            }

            tiltMm = positionKpMmPerCm * s_errorCm +
                positionKiMmPerCmS * s_errorIntegralCmS -
                speedKdMmPerCmps * s_ballSpeedCmps;
            s_pidTiltCommandMm = H26_Ball_ClampFloat(tiltMm,
                -tiltCommandLimitMm, tiltCommandLimitMm);
        }
        s_previousControlErrorCm = s_errorCm;
        s_hasPreviousControlError = 1U;
    }

    /* Feed-forward refreshes at 10 ms even between two K230 vision frames. */
    s_feedForwardTiltMm = H26_Ball_ClampFloat(feedForwardTiltMm,
        -tiltCommandLimitMm, tiltCommandLimitMm);
    s_tiltCommandMm = H26_Ball_ClampFloat(s_pidTiltCommandMm +
        s_feedForwardTiltMm, -tiltCommandLimitMm, tiltCommandLimitMm);
    s_rodTargetCount = H26_Ball_TiltMmToRodCount(s_tiltCommandMm);

    /* Do not manufacture a software stop on a dropped camera frame. */
    H26_Ball_ApplyRodPositionControl();
    return sample;
}

H26_BallControlSample_t H26_BallControl_Task10ms(uint32_t nowMs,
                                                   float targetCm)
{
    return H26_BallControl_Task10msCore(nowMs, targetCm,
        H26_T3_BALL_POSITION_TO_TILT_MM_PER_CM,
        0.0f,
        H26_T3_BALL_SPEED_TO_TILT_MM_PER_CMPS,
        0.0f,
        H26_T3_TILT_COMMAND_LIMIT_MM,
        0.0f);
}

H26_BallControlSample_t H26_BallControl_Task10msWithPid(
    uint32_t nowMs,
    float targetCm,
    float positionKpMmPerCm,
    float positionKiMmPerCmS,
    float speedKdMmPerCmps,
    float integralLimitCmS,
    float tiltCommandLimitMm)
{
    return H26_BallControl_Task10msCore(nowMs, targetCm,
        positionKpMmPerCm,
        positionKiMmPerCmS,
        speedKdMmPerCmps,
        integralLimitCmS,
        tiltCommandLimitMm,
        0.0f);
}

H26_BallControlSample_t H26_BallControl_Task10msWithPidFeedForward(
    uint32_t nowMs,
    float targetCm,
    float positionKpMmPerCm,
    float positionKiMmPerCmS,
    float speedKdMmPerCmps,
    float integralLimitCmS,
    float tiltCommandLimitMm,
    float feedForwardTiltMm)
{
    return H26_BallControl_Task10msCore(nowMs, targetCm,
        positionKpMmPerCm,
        positionKiMmPerCmS,
        speedKdMmPerCmps,
        integralLimitCmS,
        tiltCommandLimitMm,
        feedForwardTiltMm);
}

float H26_BallControl_GetPositionCm(void) { return s_positionCm; }
float H26_BallControl_GetBallSpeedCmps(void) { return s_ballSpeedCmps; }
float H26_BallControl_GetTargetCm(void) { return s_targetCm; }
float H26_BallControl_GetErrorCm(void) { return s_errorCm; }
int32_t H26_BallControl_GetCommandHz(void) { return s_commandHz; }
uint8_t H26_BallControl_IsVisionValid(void) { return s_visionValid; }
uint8_t H26_BallControl_GetConfidence(void) { return s_confidence; }
uint32_t H26_BallControl_GetFrameAgeMs(void) { return s_frameAgeMs; }
uint16_t H26_BallControl_GetRawPositionCentiCm(void) { return s_rawPositionCentiCm; }
uint16_t H26_BallControl_GetOriginCentiCm(void) { return s_originCentiCm; }
uint8_t H26_BallControl_IsOriginCalibrated(void) { return s_originCalibrated; }
uint16_t H26_BallControl_GetLastSequence(void) { return s_lastSequence; }
uint8_t H26_BallControl_GetLastFlags(void) { return s_lastFlags; }
uint16_t H26_BallControl_GetStableSampleMs(void)
{
    return (s_newMeasurementDtMs == 0U) ? H26_T3_NOMINAL_FRAME_MS :
        s_newMeasurementDtMs;
}
int32_t H26_BallControl_GetRodEncoderCount(void) { return s_rodEncoderCount; }
int32_t H26_BallControl_GetRodTargetCount(void) { return s_rodTargetCount; }
float H26_BallControl_GetTiltCommandMm(void) { return s_tiltCommandMm; }
float H26_BallControl_GetPidTiltCommandMm(void) { return s_pidTiltCommandMm; }
float H26_BallControl_GetFeedForwardTiltMm(void) { return s_feedForwardTiltMm; }
