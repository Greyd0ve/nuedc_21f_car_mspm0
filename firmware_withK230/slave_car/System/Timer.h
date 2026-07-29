#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>

void Timer_Init(void);

/* Monotonic unsigned millisecond timebase; naturally wraps after 49.7 days. */
uint32_t Timer_GetMs(void);

#endif
