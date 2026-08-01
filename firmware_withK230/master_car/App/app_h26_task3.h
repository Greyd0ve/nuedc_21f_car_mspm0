#ifndef __APP_H26_TASK3_H
#define __APP_H26_TASK3_H

#include <stdint.h>

typedef enum
{
    H26_T3_IDLE = 0,
    H26_T3_READY,
    H26_T3_EXTEND_10MM,
    H26_T3_HOLD_TO_STAGE1,
    H26_T3_FINAL_PID_STAGE1,
    H26_T3_HOLD_TO_RETRACT,
    H26_T3_RETRACT_10MM,
    H26_T3_HOLD_TO_STAGE2,
    H26_T3_FINAL_PID_STAGE2,
    H26_T3_DONE,
    H26_T3_FINAL_PID_STAGE3,
    H26_T3_FAULT
} H26_Task3State_t;

typedef enum
{
    H26_T3_FAULT_NONE = 0,
    H26_T3_FAULT_STEPPER_OUTPUT,
    H26_T3_FAULT_ILLEGAL_STATE
} H26_Task3Fault_t;

typedef enum
{
    H26_T3_RESULT_RUNNING = 0,
    H26_T3_RESULT_FINISHED,
    H26_T3_RESULT_FAULT
} H26_Task3Result_t;

void H26_Task3_Init(void);
void H26_Task3_Reset(void);
void H26_Task3_Start(uint32_t startMs);
void H26_Task3_ForceFault(void);
H26_Task3Result_t H26_Task3_Task10ms(uint32_t nowMs);

H26_Task3State_t H26_Task3_GetState(void);
uint32_t H26_Task3_GetElapsedMs(uint32_t nowMs);
uint32_t H26_Task3_GetFinalElapsedMs(void);
H26_Task3Fault_t H26_Task3_GetFault(void);

#endif
