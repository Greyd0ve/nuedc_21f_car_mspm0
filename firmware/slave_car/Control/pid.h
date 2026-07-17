#ifndef __PID_H
#define __PID_H

typedef struct
{
    /* PID 参数；实际单位由调用者的目标值/测量值单位决定。 */
    float Kp;
    float Ki;
    float Kd;
    /* 运行状态；执行器关闭时应复位。 */
    float Integral;
    float LastError;
    /* 输出和累计积分项的对称限幅。 */
    float OutputLimit;
    float IntegralLimit;
} PID_TypeDef;

/* 速度环和转向环共用同一个位置式 PID 核心。
 * OutputLimit 用于限制电机 PWM 输出，IntegralLimit 用于抑制积分饱和。
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float outputLimit, float integralLimit);
void PID_SetTunings(PID_TypeDef *pid, float kp, float ki, float kd);
void PID_SetOutputLimit(PID_TypeDef *pid, float outputLimit);
void PID_Reset(PID_TypeDef *pid);
float PID_Calc(PID_TypeDef *pid, float target, float measure);

#endif
