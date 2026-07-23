#include "app_control.h"
#include "app_config.h"
#include "app_tuning.h"
#include "app_car_state.h"
#include "Board_Config.h"
#include "Encoder.h"
#include "Motor.h"
#include "PWM.h"
#include "StepperMotor.h"
#include "pid.h"
#include <stdint.h>

#define APP_PWM_LIMIT_MIN          0.0f
#define APP_FORWARD_I_LIMIT        260.0f
#define APP_TURN_I_LIMIT           220.0f

/*
 * Stepper control: target speed (cm/s) → step frequency (Hz).
 *
 * wheel_rev_per_sec = speed_cmps / wheel_circumference_cm
 * step_freq_hz = wheel_rev_per_sec * STEP_PER_REV
 *
 * First version uses direct conversion as feed-forward.
 * Encoder PI correction can be enabled by setting g_stepperSpeedKp > 0.
 */
#if ECAR_MOTOR_TYPE_STEPPER
#ifndef STEPPER_SPEED_FF_ENABLE
#define STEPPER_SPEED_FF_ENABLE 1
#endif
#ifndef STEPPER_SPEED_KP
#define STEPPER_SPEED_KP        0.0f
#endif
#ifndef STEPPER_SPEED_KI
#define STEPPER_SPEED_KI        0.0f
#endif
#ifndef STEPPER_SPEED_I_LIMIT_HZ
#define STEPPER_SPEED_I_LIMIT_HZ 500.0f
#endif
#endif

static PID_TypeDef ForwardPID;
static PID_TypeDef TurnPID;

static float s_leftSpeedI = 0.0f;
static float s_rightSpeedI = 0.0f;

volatile int16_t g_rightLastNonZeroDelta = 0;
volatile uint32_t g_rightNonZeroDeltaCount = 0U;
volatile uint32_t g_rightLimitDeltaCount = 0U;

static float App_Control_LimitFloat(float value, float minVal, float maxVal)
{
    if (value < minVal)
    {
        return minVal;
    }

    if (value > maxVal)
    {
        return maxVal;
    }

    return value;
}

static int16_t App_Control_LimitI16(int32_t value, int16_t minVal, int16_t maxVal)
{
    if (value < minVal)
    {
        return minVal;
    }

    if (value > maxVal)
    {
        return maxVal;
    }

    return (int16_t)value;
}

static int16_t App_Control_SlewI16(int16_t current, int16_t target, int16_t step)
{
    if (step <= 0)
    {
        return target;
    }

    if (target > (int16_t)(current + step))
    {
        return (int16_t)(current + step);
    }

    if (target < (int16_t)(current - step))
    {
        return (int16_t)(current - step);
    }

    return target;
}

static float App_Control_SpeedFeedForward(float targetSpeed)
{
    float absSpeed;
    float pwm;

    if ((targetSpeed > -TUNE_SPEED_FF_DEAD_BAND_CMPS) &&
        (targetSpeed <  TUNE_SPEED_FF_DEAD_BAND_CMPS))
    {
        return 0.0f;
    }

    absSpeed = (targetSpeed >= 0.0f) ? targetSpeed : -targetSpeed;

    if (absSpeed <= TUNE_SPEED_FF_BREAK_CMPS)
    {
        pwm = TUNE_SPEED_FF_LOW_BASE_PWM + TUNE_SPEED_FF_LOW_K * absSpeed;
    }
    else
    {
        pwm = TUNE_SPEED_FF_HIGH_BASE_PWM
              + TUNE_SPEED_FF_HIGH_K * (absSpeed - TUNE_SPEED_FF_BREAK_CMPS);
    }

    if (targetSpeed < 0.0f)
    {
        pwm = -pwm;
    }

    return pwm;
}

#if ECAR_MOTOR_TYPE_STEPPER
/*
 * Convert wheel speed (cm/s) to step frequency (Hz).
 * Positive = forward, negative = backward.
 */
static int32_t App_Control_SpeedToStepFreqHz(float speedCmps)
{
    float revPerSec;
    float freq;

    if (speedCmps > -0.01f && speedCmps < 0.01f)
    {
        return 0;
    }

    revPerSec = speedCmps / ECAR_WHEEL_CIRCUMFERENCE_CM;
    freq = revPerSec * (float)STEPPER_STEP_PER_REV;

    if (freq > (float)STEPPER_MAX_FREQ_HZ)
    {
        freq = (float)STEPPER_MAX_FREQ_HZ;
    }
    else if (freq < -(float)STEPPER_MAX_FREQ_HZ)
    {
        freq = -(float)STEPPER_MAX_FREQ_HZ;
    }

    return (int32_t)freq;
}
#endif

void App_Control_Init(void)
{
    /*
     * 当前速度测试阶段暂时不用 PID_Calc 输出速度 PWM，
     * 但保留 PID 初始化，避免其他文件接口受影响。
     */
    PID_Init(&ForwardPID, g_forwardKp, g_forwardKi, g_forwardKd,
             (float)PWM_MAX_DUTY, APP_FORWARD_I_LIMIT);

    PID_Init(&TurnPID, g_turnKp, g_turnKi, g_turnKd,
             (float)PWM_MAX_DUTY * 0.85f, APP_TURN_I_LIMIT);
}

void App_Control_UpdatePIDParam(void)
{
    PID_SetTunings(&ForwardPID, g_forwardKp, g_forwardKi, g_forwardKd);
    PID_SetTunings(&TurnPID, g_turnKp, g_turnKi, g_turnKd);
}

void App_Control_ResetPID(void)
{
    PID_Reset(&ForwardPID);
    PID_Reset(&TurnPID);

    s_leftSpeedI = 0.0f;
    s_rightSpeedI = 0.0f;
}

void App_Control_ForcePWMZero(void)
{
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;

    g_speedPwm = 0.0f;
    g_diffPwm = 0.0f;

    g_leftPwm = 0;
    g_rightPwm = 0;

    g_carEnable = 0U;

#if ECAR_MOTOR_TYPE_STEPPER
    StepperMotor_StopAll();
#else
    Motor_StopAll();
#endif
    App_Control_ResetPID();
}

void App_Control_UpdateEncoderSpeed(uint16_t periodMs)
{
    int16_t leftDelta;
    int16_t rightDelta;
    float speedScale;
    float leftSpeedNow;
    float rightSpeedNow;

    if (periodMs == 0U)
    {
        periodMs = ECAR_ENCODER_SPEED_PERIOD_MS;
    }

    leftDelta = Encoder_GetLeftDelta();
    rightDelta = Encoder_GetRightDelta();

    if (rightDelta != 0)
    {
        g_rightLastNonZeroDelta = rightDelta;
        g_rightNonZeroDeltaCount++;

        if ((rightDelta == 32767) || (rightDelta == -32768))
        {
            g_rightLimitDeltaCount++;
        }
    }

    speedScale = ECAR_CM_PER_PULSE * 1000.0f / (float)periodMs;

    g_leftEncoderDelta = leftDelta;
    g_rightEncoderDelta = rightDelta;

    g_leftEncoderTotal += leftDelta;
    g_rightEncoderTotal += rightDelta;

    g_forwardEncoderTotal = (g_leftEncoderTotal + g_rightEncoderTotal) / 2;
    g_turnEncoderTotal = (g_rightEncoderTotal - g_leftEncoderTotal) / 2;

    leftSpeedNow = (float)leftDelta * speedScale;
    rightSpeedNow = (float)rightDelta * speedScale;

    g_leftSpeed += TUNE_SPEED_FILTER_ALPHA * (leftSpeedNow - g_leftSpeed);
    g_rightSpeed += TUNE_SPEED_FILTER_ALPHA * (rightSpeedNow - g_rightSpeed);

    g_forwardSpeed = (g_leftSpeed + g_rightSpeed) * 0.5f;
    g_turnSpeed = (g_rightSpeed - g_leftSpeed) * 0.5f;
}

void App_Control_ApplyMotorOutput(void)
{
    float pwmLimit;
    int16_t pwmLimitI16;

    float leftTarget;
    float rightTarget;

    float leftErr;
    float rightErr;

    float leftFF;
    float rightFF;

    float leftPwmF;
    float rightPwmF;

    int16_t targetLeftPwm;
    int16_t targetRightPwm;

#if ECAR_MOTOR_TYPE_STEPPER
    int32_t leftFreqHz;
    int32_t rightFreqHz;
    float leftFreqFF;
    float rightFreqFF;
    float leftFreqCorr;
    float rightFreqCorr;
#endif

    if (!g_carEnable || g_pwmLimit <= 0.5f)
    {
        g_forwardSpeedError = g_targetForwardSpeed - g_forwardSpeed;

        g_speedPwm = 0.0f;
        g_diffPwm = 0.0f;

        g_leftPwm = 0;
        g_rightPwm = 0;

        s_leftSpeedI = 0.0f;
        s_rightSpeedI = 0.0f;

#if ECAR_MOTOR_TYPE_STEPPER
        StepperMotor_StopAll();
#else
        Motor_StopAll();
#endif
        App_Control_ResetPID();
        return;
    }

    pwmLimit = App_Control_LimitFloat(g_pwmLimit,
                                      APP_PWM_LIMIT_MIN,
                                      (float)PWM_MAX_DUTY);
    pwmLimitI16 = (int16_t)pwmLimit;

    leftTarget = g_targetForwardSpeed - g_targetTurnSpeed;
    rightTarget = g_targetForwardSpeed + g_targetTurnSpeed;

    leftTarget = App_Control_LimitFloat(leftTarget,
                                        -TUNE_WHEEL_TARGET_LIMIT_CMPS,
                                        TUNE_WHEEL_TARGET_LIMIT_CMPS);
    rightTarget = App_Control_LimitFloat(rightTarget,
                                         -TUNE_WHEEL_TARGET_LIMIT_CMPS,
                                         TUNE_WHEEL_TARGET_LIMIT_CMPS);

#if ECAR_MOTOR_TYPE_STEPPER

    leftErr = leftTarget - g_leftSpeed;
    rightErr = rightTarget - g_rightSpeed;

    s_leftSpeedI += leftErr;
    s_rightSpeedI += rightErr;

    s_leftSpeedI = App_Control_LimitFloat(s_leftSpeedI,
                                          -STEPPER_SPEED_I_LIMIT_HZ,
                                          STEPPER_SPEED_I_LIMIT_HZ);
    s_rightSpeedI = App_Control_LimitFloat(s_rightSpeedI,
                                           -STEPPER_SPEED_I_LIMIT_HZ,
                                           STEPPER_SPEED_I_LIMIT_HZ);

    leftFreqFF  = (float)App_Control_SpeedToStepFreqHz(leftTarget);
    rightFreqFF = (float)App_Control_SpeedToStepFreqHz(rightTarget);

    leftFreqCorr = leftFreqFF
                   + STEPPER_SPEED_KP * leftErr
                   + STEPPER_SPEED_KI * s_leftSpeedI;
    rightFreqCorr = rightFreqFF
                    + STEPPER_SPEED_KP * rightErr
                    + STEPPER_SPEED_KI * s_rightSpeedI;

    if (leftFreqCorr > (float)STEPPER_MAX_FREQ_HZ)
        { leftFreqCorr = (float)STEPPER_MAX_FREQ_HZ; }
    else if (leftFreqCorr < -(float)STEPPER_MAX_FREQ_HZ)
        { leftFreqCorr = -(float)STEPPER_MAX_FREQ_HZ; }

    if (rightFreqCorr > (float)STEPPER_MAX_FREQ_HZ)
        { rightFreqCorr = (float)STEPPER_MAX_FREQ_HZ; }
    else if (rightFreqCorr < -(float)STEPPER_MAX_FREQ_HZ)
        { rightFreqCorr = -(float)STEPPER_MAX_FREQ_HZ; }

    leftFreqHz  = (int32_t)(leftFreqCorr * LEFT_STEPPER_DIR_SIGN);
    rightFreqHz = (int32_t)(rightFreqCorr * RIGHT_STEPPER_DIR_SIGN);

    StepperMotor_SetTargetFrequency(leftFreqHz, rightFreqHz);

    g_leftPwm  = 0;
    g_rightPwm = 0;
    g_speedPwm = (float)(leftFreqHz + rightFreqHz) * 0.5f;
    g_diffPwm  = (float)(rightFreqHz - leftFreqHz) * 0.5f;
    g_forwardSpeedError = g_targetForwardSpeed - g_forwardSpeed;

#else

    leftErr = leftTarget - g_leftSpeed;
    rightErr = rightTarget - g_rightSpeed;

    s_leftSpeedI += leftErr;
    s_rightSpeedI += rightErr;

    s_leftSpeedI = App_Control_LimitFloat(s_leftSpeedI,
                                          -TUNE_SPEED_I_LIMIT,
                                          TUNE_SPEED_I_LIMIT);
    s_rightSpeedI = App_Control_LimitFloat(s_rightSpeedI,
                                           -TUNE_SPEED_I_LIMIT,
                                           TUNE_SPEED_I_LIMIT);

    leftFF = App_Control_SpeedFeedForward(leftTarget);
    rightFF = App_Control_SpeedFeedForward(rightTarget);

    leftPwmF = leftFF + TUNE_SPEED_PI_KP * leftErr + TUNE_SPEED_PI_KI * s_leftSpeedI;
    rightPwmF = rightFF + TUNE_SPEED_PI_KP * rightErr + TUNE_SPEED_PI_KI * s_rightSpeedI;

    targetLeftPwm = App_Control_LimitI16((int32_t)leftPwmF,
                                         (int16_t)(-pwmLimitI16),
                                         pwmLimitI16);
    targetRightPwm = App_Control_LimitI16((int32_t)rightPwmF,
                                          (int16_t)(-pwmLimitI16),
                                          pwmLimitI16);

    g_leftPwm = App_Control_SlewI16(g_leftPwm,
                                    targetLeftPwm,
                                    TUNE_PWM_SLEW_STEP);
    g_rightPwm = App_Control_SlewI16(g_rightPwm,
                                     targetRightPwm,
                                     TUNE_PWM_SLEW_STEP);

    g_speedPwm = (float)(g_leftPwm + g_rightPwm) * 0.5f;
    g_diffPwm = (float)(g_rightPwm - g_leftPwm) * 0.5f;
    g_forwardSpeedError = g_targetForwardSpeed - g_forwardSpeed;

    Motor_SetPWM(g_leftPwm, g_rightPwm);
#endif
}