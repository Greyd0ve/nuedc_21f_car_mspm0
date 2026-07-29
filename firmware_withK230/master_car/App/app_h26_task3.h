#ifndef __APP_H26_TASK3_H
#define __APP_H26_TASK3_H

#include <stdint.h>

typedef enum
{
    H26_T3_IDLE = 0,
    H26_T3_ACQUIRE_O,
    H26_T3_MOVE_PLUS_5,
    H26_T3_HOLD_PLUS_5,
    H26_T3_MOVE_MINUS_5,
    H26_T3_HOLD_MINUS_5,
    H26_T3_DONE,
    H26_T3_FAULT
} H26_Task3State_t;

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
float H26_Task3_GetPositionCm(void);
float H26_Task3_GetBallSpeedCmps(void);
float H26_Task3_GetTargetCm(void);
int32_t H26_Task3_GetCommandHz(void);
uint8_t H26_Task3_IsVisionValid(void);
uint8_t H26_Task3_GetConfidence(void);
uint32_t H26_Task3_GetFrameAgeMs(void);

#endif
