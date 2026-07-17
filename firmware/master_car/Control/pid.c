#include "pid.h"

/* 通用位置式 PID 核心，前进速度环和转向差速环共用。 */
static float PID_LimitFloat(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float outputLimit, float integralLimit)
{
    if (pid == 0) return;

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Integral = 0.0f;
    pid->LastError = 0.0f;
    pid->OutputLimit = outputLimit;
    pid->IntegralLimit = integralLimit;
}

void PID_SetTunings(PID_TypeDef *pid, float kp, float ki, float kd)
{
    if (pid == 0) return;

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

void PID_SetOutputLimit(PID_TypeDef *pid, float outputLimit)
{
    if (pid == 0) return;

    pid->OutputLimit = outputLimit;
}

void PID_Reset(PID_TypeDef *pid)
{
    if (pid == 0) return;

    pid->Integral = 0.0f;
    pid->LastError = 0.0f;
}

float PID_Calc(PID_TypeDef *pid, float target, float measure)
{
    float error;
    float derivative;
    float integralCandidate;
    float output;

    if (pid == 0) return 0.0f;

    error = target - measure;
    derivative = error - pid->LastError;
    integralCandidate = pid->Integral + error;

    /* 先限制积分候选值，再计算输出，减少积分饱和。 */
    integralCandidate = PID_LimitFloat(integralCandidate, -pid->IntegralLimit, pid->IntegralLimit);
    output = pid->Kp * error + pid->Ki * integralCandidate + pid->Kd * derivative;

    if (output > pid->OutputLimit)
    {
        /* 条件积分：输出饱和时只允许有助于退出饱和的积分更新。 */
        output = pid->OutputLimit;
        if (error < 0.0f) pid->Integral = integralCandidate;
    }
    else if (output < -pid->OutputLimit)
    {
        output = -pid->OutputLimit;
        if (error > 0.0f) pid->Integral = integralCandidate;
    }
    else
    {
        pid->Integral = integralCandidate;
    }

    pid->LastError = error;
    return output;
}
