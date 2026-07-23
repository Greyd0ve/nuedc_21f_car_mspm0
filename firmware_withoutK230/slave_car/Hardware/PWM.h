#ifndef __PWM_H
#define __PWM_H

#include <stdint.h>

/* Motor.c 和 PID 输出限幅使用的逻辑占空比标尺。 */
#define PWM_MAX_DUTY 1000U /* 逻辑最大占空比，1000 表示 100% */

/* 将两个 PWM 通道初始化为 0 占空比，并启动定时器。 */
void PWM_Init(void);

/* 根据 0..PWM_MAX_DUTY 的逻辑占空比设置左右 PWM 比较值。 */
void PWM_SetCompareA(uint16_t compare);
void PWM_SetCompareB(uint16_t compare);

#endif
