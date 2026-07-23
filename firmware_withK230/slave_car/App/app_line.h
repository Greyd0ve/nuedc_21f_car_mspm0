#ifndef __APP_LINE_H
#define __APP_LINE_H

#include <stdint.h>

/* 循迹公开遥测量，供状态机、OLED 和串口曲线输出读取。 */
extern volatile int16_t g_lineError;
extern volatile uint8_t g_lineValid;
extern volatile uint8_t g_lineMask;
extern volatile uint8_t g_lineRawMask;
extern volatile uint8_t g_lineBlackCount;
extern volatile uint8_t g_lineBadMaskCount;
extern volatile uint8_t g_lineZeroMaskCount;
extern volatile uint16_t g_lineLostMs;

/* 重新初始化灰度 GPIO 层，用于应用启动时强制刷新硬件状态。 */
void App_Line_GPIOForceInit(void);

/* 清空滤波器、计数器和最近一次有效黑线记忆。 */
void App_Line_ResetState(void);

/* 读取全部灰度通道，并更新 mask、有效性、线误差和角点稳定计数。 */
void App_Line_Update(void);

/* 将当前线误差转换为限幅后的转向速度命令。 */
float App_Line_CalcTurnCmd(void);

#endif
