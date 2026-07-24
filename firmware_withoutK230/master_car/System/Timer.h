#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>

/* 配置并启动前台调度器使用的 1ms 系统 tick。 */
void Timer_Init(void);
uint32_t Timer_GetMillis(void);

#endif
