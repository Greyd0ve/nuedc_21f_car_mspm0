#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>

/* 复位按键消抖状态。 */
void Key_Init(void);

/* 返回并清除一个释放触发的按键事件；无事件返回 0。 */
uint8_t Key_GetNum(void);

/* 直接读取当前低有效按键电平，不做消抖。 */
uint8_t Key_GetState(void);

/* 1ms tick 钩子；内部每 20ms 执行一次轮询/消抖。 */
void Key_Tick(void);

#endif
