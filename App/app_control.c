#include "app_control.h"
#include "app_config.h"
#include "Encoder.h"
#include "Motor.h"
#include "PWM.h"
#include "pid.h"
#include <stdint.h>

/* 闭环速度控制模块。
 * 编码器速度单位为 cm/s，转向速度为左右轮差速量，最终输出为 Motor 模块约定的有符号 PWM。
 */
#define APP_PWM_LIMIT_MIN       0.0f    /* PWM 输出下限，防止负限幅传入 PID */
#define APP_FORWARD_I_LIMIT     260.0f  /* 前进速度 PID 积分限幅 */
#define APP_TURN_I_LIMIT        220.0f  /* 转向差速 PID 积分限幅 */

/* PID 调参量和运行遥测变量由 app_e_car.c 定义，串口、OLED 和控制环共享同一份状态。
 */
extern volatile float g_forwardKp;
extern volatile float g_forwardKi;
extern volatile float g_forwardKd;
extern volatile float g_turnKp;
extern volatile float g_turnKi;
extern volatile float g_turnKd;

extern volatile float g_pwmLimit;
extern volatile float g_targetForwardSpeed;
extern volatile float g_targetTurnSpeed;
extern volatile float g_leftSpeed;
extern volatile float g_rightSpeed;
extern volatile float g_forwardSpeed;
extern volatile float g_turnSpeed;
extern volatile float g_speedPwm;
extern volatile float g_diffPwm;
extern volatile float g_forwardSpeedError;

extern volatile int16_t g_leftEncoderDelta;
extern volatile int16_t g_rightEncoderDelta;
extern volatile int16_t g_leftPwm;
extern volatile int16_t g_rightPwm;
extern volatile uint8_t g_carEnable;

extern volatile int32_t g_leftEncoderTotal;
extern volatile int32_t g_rightEncoderTotal;
extern volatile int32_t g_forwardEncoderTotal;
extern volatile int32_t g_turnEncoderTotal;

static PID_TypeDef ForwardPID;
static PID_TypeDef TurnPID;

volatile int16_t g_rightLastNonZeroDelta = 0;
volatile uint32_t g_rightNonZeroDeltaCount = 0U;
volatile uint32_t g_rightLimitDeltaCount = 0U;

/* 本地限幅函数：让 PID/PWM 限幅在调用处保持清晰。 */
static float App_Control_LimitFloat(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

static int16_t App_Control_LimitI16(int32_t value, int16_t minVal, int16_t maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return (int16_t)value;
}

void App_Control_Init(void)
{
    /* 输出限幅会在每个控制周期用 g_pwmLimit 刷新。 */
    PID_Init(&ForwardPID, g_forwardKp, g_forwardKi, g_forwardKd, (float)PWM_MAX_DUTY, APP_FORWARD_I_LIMIT);
    PID_Init(&TurnPID, g_turnKp, g_turnKi, g_turnKd, (float)PWM_MAX_DUTY * 0.85f, APP_TURN_I_LIMIT);
}

void App_Control_UpdatePIDParam(void)
{
    /* 串口调参后立即更新 PID 参数，但不清空积分项。 */
    PID_SetTunings(&ForwardPID, g_forwardKp, g_forwardKi, g_forwardKd);
    PID_SetTunings(&TurnPID, g_turnKp, g_turnKi, g_turnKd);
}

void App_Control_ResetPID(void)
{
    PID_Reset(&ForwardPID);
    PID_Reset(&TurnPID);
}

void App_Control_ForcePWMZero(void)
{
    /* 统一安全停车路径：正常停车、故障停车、板级测试锁定都走这里。 */
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_speedPwm = 0.0f;
    g_diffPwm = 0.0f;
    g_leftPwm = 0;
    g_rightPwm = 0;
    g_carEnable = 0;
    Motor_StopAll();
    App_Control_ResetPID();
}

void App_Control_UpdateEncoderSpeed(uint16_t periodMs)
{
    int16_t leftDelta;
    int16_t rightDelta;
    float speedScale;

    if (periodMs == 0U)
    {
        /* 防御性处理：避免 periodMs 为 0 时除 0。 */
        periodMs = ECAR_ENCODER_SPEED_PERIOD_MS;
    }

    /* Encoder_Get*Delta() 会原子读取并清零累计脉冲。 */
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

    /* 总里程保持原始脉冲单位，方便角点/圈数阈值直接调参。 */
    g_leftEncoderDelta = leftDelta;
    g_rightEncoderDelta = rightDelta;
    g_leftEncoderTotal += leftDelta;
    g_rightEncoderTotal += rightDelta;
    g_forwardEncoderTotal = (g_leftEncoderTotal + g_rightEncoderTotal) / 2;
    g_turnEncoderTotal = (g_rightEncoderTotal - g_leftEncoderTotal) / 2;

    g_leftSpeed = (float)leftDelta * speedScale;
    g_rightSpeed = (float)rightDelta * speedScale;
    g_forwardSpeed = (g_leftSpeed + g_rightSpeed) * 0.5f;
    g_turnSpeed = (g_rightSpeed - g_leftSpeed) * 0.5f;
}

void App_Control_ApplyMotorOutput(void)
{
    float pwmLimit;
    int32_t leftPwmTemp;
    int32_t rightPwmTemp;

    if (!g_carEnable || g_pwmLimit <= 0.5f)
    {
        /* 控制关闭时必须主动停车并清 PID，避免恢复使能后输出旧积分。 */
        g_forwardSpeedError = g_targetForwardSpeed - g_forwardSpeed;
        g_speedPwm = 0.0f;
        g_diffPwm = 0.0f;
        g_leftPwm = 0;
        g_rightPwm = 0;
        Motor_StopAll();
        App_Control_ResetPID();
        return;
    }

    /* 前进 PID 生成共模 PWM，转向 PID 生成左右轮差速 PWM。 */
    pwmLimit = App_Control_LimitFloat(g_pwmLimit, APP_PWM_LIMIT_MIN, (float)PWM_MAX_DUTY);
    PID_SetOutputLimit(&ForwardPID, pwmLimit);
    PID_SetOutputLimit(&TurnPID, pwmLimit * 0.85f);

    g_forwardSpeedError = g_targetForwardSpeed - g_forwardSpeed;
    g_speedPwm = PID_Calc(&ForwardPID, g_targetForwardSpeed, g_forwardSpeed);
    g_diffPwm = PID_Calc(&TurnPID, g_targetTurnSpeed, g_turnSpeed);

    leftPwmTemp = (int32_t)(g_speedPwm - g_diffPwm);
    rightPwmTemp = (int32_t)(g_speedPwm + g_diffPwm);

    g_leftPwm = App_Control_LimitI16(leftPwmTemp, (int16_t)(-pwmLimit), (int16_t)pwmLimit);
    g_rightPwm = App_Control_LimitI16(rightPwmTemp, (int16_t)(-pwmLimit), (int16_t)pwmLimit);

    Motor_SetPWM(g_leftPwm, g_rightPwm);
}
