#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/* 初始化 PWM 和方向 GPIO，完成后保持左右电机停止。 */
void Motor_Init(void);

/* 设置单侧电机有符号逻辑 PWM；实际极性在 Motor.c 内修正。 */
void Motor_SetLeftPWM(int16_t pwm);
void Motor_SetRightPWM(int16_t pwm);

/* 同时设置左右电机有符号逻辑 PWM。 */
void Motor_SetPWM(int16_t leftPwm, int16_t rightPwm);

/* 强制占空比为 0，并清空 TB6612 所有方向输入。 */
void Motor_StopAll(void);

#endif
