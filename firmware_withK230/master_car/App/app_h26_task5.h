#ifndef __APP_H26_TASK5_H
#define __APP_H26_TASK5_H

#include <stdint.h>

typedef enum
{
    H26_T5_IDLE = 0,
    H26_T5_ACQUIRE_O,
    H26_T5_LEAVE_A,
    H26_T5_LAP_RUNNING,
    H26_T5_DONE,
    H26_T5_FAULT,
    /* Appended so the established DONE/FAULT telemetry values stay stable. */
    H26_T5_PRETILT
} H26_Task5State_t;

typedef enum
{
    H26_T5_FAULT_NONE = 0,
    H26_T5_FAULT_ILLEGAL_STATE
} H26_Task5Fault_t;

typedef enum
{
    H26_T5_RESULT_RUNNING = 0,
    H26_T5_RESULT_FINISHED,
    H26_T5_RESULT_FAULT
} H26_Task5Result_t;

void H26_Task5_Init(void);
void H26_Task5_Reset(void);
void H26_Task5_Start(uint32_t startMs);
void H26_Task5_ForceFault(void);
H26_Task5Result_t H26_Task5_Task10ms(uint32_t nowMs);
/* Keeps the ball at O after the lap timer is frozen. */
void H26_Task5_HoldBall10ms(uint32_t nowMs);

H26_Task5State_t H26_Task5_GetState(void);
H26_Task5Fault_t H26_Task5_GetFault(void);
uint32_t H26_Task5_GetElapsedMs(uint32_t nowMs);
uint32_t H26_Task5_GetFinalElapsedMs(void);
float H26_Task5_GetDistanceCm(void);
float H26_Task5_GetBallPeakErrorCm(void);
float H26_Task5_GetBallPositionCm(void);

#endif
