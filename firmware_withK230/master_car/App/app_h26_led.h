#ifndef __APP_H26_LED_H
#define __APP_H26_LED_H

#include <stdint.h>

typedef enum
{
    H26_LED_OFF = 0,
    H26_LED_SHOW_TASK,
    H26_LED_RUNNING,
    H26_LED_FINISHED,
    H26_LED_FAULT
} H26_LedMode_t;

void H26_LedInit(void);
void H26_LedTick1ms(void);
void H26_LedSetMode(H26_LedMode_t mode);
void H26_LedShowTask(uint8_t taskNumber, H26_LedMode_t restoreMode);
H26_LedMode_t H26_LedGetMode(void);

#endif
