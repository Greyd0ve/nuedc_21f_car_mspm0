#ifndef __APP_H26_H
#define __APP_H26_H

#include <stdint.h>

typedef enum
{
    H26_TASK_2 = 2,
    H26_TASK_3 = 3,
    H26_TASK_4 = 4,
    H26_TASK_5 = 5,
    H26_TASK_6 = 6
} H26_TaskId_t;

typedef enum
{
    H26_SYS_SELECT = 0,
    H26_SYS_PREPARE,
    H26_SYS_RUNNING,
    H26_SYS_FINISHED,
    H26_SYS_STOPPED,
    H26_SYS_FAULT
} H26_SystemState_t;

void H26_Init(void);
void H26_Tick1ms(void);
void H26_KeyProcess(void);
void H26_Task10ms(void);
void H26_Task100ms(void);
void H26_Task200ms(void);

H26_SystemState_t H26_GetSystemState(void);
H26_TaskId_t H26_GetSelectedTask(void);

#endif
