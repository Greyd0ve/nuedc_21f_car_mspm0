#ifndef __APP_TASK_MODE_H
#define __APP_TASK_MODE_H

#include <stdint.h>

typedef enum
{
    F21_TASK_MODE_BASIC = 0,
    F21_TASK_MODE_COOP
} F21TaskMode_t;

void App_TaskMode_Init(void);
F21TaskMode_t App_TaskMode_Get(void);
void App_TaskMode_Tick1ms(void);
void App_TaskMode_KeyProcess(void);

#endif
