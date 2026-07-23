#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include <stdint.h>

/* 用当前全局调参值和安全限幅初始化 PID 对象。 */
void App_Control_Init(void);

/* 将运行时调好的 PID 参数同步到当前 PID 对象。 */
void App_Control_UpdatePIDParam(void);

/* 清空 PID 积分和上次误差，不直接操作电机 GPIO。 */
void App_Control_ResetPID(void);

/* 强制目标速度、PWM 状态归零，并停止左右电机。 */
void App_Control_ForcePWMZero(void);

/* 按 periodMs 实际周期更新编码器总脉冲和轮速，轮速单位 cm/s。 */
void App_Control_UpdateEncoderSpeed(uint16_t periodMs);

/* 执行前进/转向 PID，并将有符号 PWM 写入电机驱动。 */
void App_Control_ApplyMotorOutput(void);

#endif
