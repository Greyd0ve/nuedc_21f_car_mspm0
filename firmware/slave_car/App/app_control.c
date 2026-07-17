#include "app_control.h"
#include "app_config.h"
#include "app_tuning.h"
#include "app_car_state.h"
#include "Encoder.h"
#include "Motor.h"
#include "PWM.h"
#include "pid.h"
#include <stdint.h>

#define APP_PWM_LIMIT_MIN          0.0f
#define APP_FORWARD_I_LIMIT        260.0f
#define APP_TURN_I_LIMIT           220.0f

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

    Motor_StopAll();
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

    if (!g_carEnable || g_pwmLimit <= 0.5f)
    {
        g_forwardSpeedError = g_targetForwardSpeed - g_forwardSpeed;

        g_speedPwm = 0.0f;
        g_diffPwm = 0.0f;

        g_leftPwm = 0;
        g_rightPwm = 0;

        s_leftSpeedI = 0.0f;
        s_rightSpeedI = 0.0f;

        Motor_StopAll();
        App_Control_ResetPID();
        return;
    }

    pwmLimit = App_Control_LimitFloat(g_pwmLimit,
                                      APP_PWM_LIMIT_MIN,
                                      (float)PWM_MAX_DUTY);
    pwmLimitI16 = (int16_t)pwmLimit;

    /*
     * g_targetForwardSpeed：车体前进速度，cm/s
     * g_targetTurnSpeed：左右轮差速修正，cm/s
     *
     * turn 为正时：右轮目标速度更高，左轮更低。
     */
    leftTarget = g_targetForwardSpeed - g_targetTurnSpeed;
    rightTarget = g_targetForwardSpeed + g_targetTurnSpeed;

    leftTarget = App_Control_LimitFloat(leftTarget,
                                        -TUNE_WHEEL_TARGET_LIMIT_CMPS,
                                        TUNE_WHEEL_TARGET_LIMIT_CMPS);
    rightTarget = App_Control_LimitFloat(rightTarget,
                                         -TUNE_WHEEL_TARGET_LIMIT_CMPS,
                                         TUNE_WHEEL_TARGET_LIMIT_CMPS);

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
}