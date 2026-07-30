#ifndef __APP_H26_TASK4_H
#define __APP_H26_TASK4_H

#include <stdint.h>

typedef enum
{
    H26_T4_IDLE = 0,
    H26_T4_ACQUIRE_O,
    H26_T4_RUN_TO_B,
    H26_T4_DONE,
    H26_T4_FAULT
} H26_Task4State_t;

typedef enum
{
    H26_T4_FAULT_NONE = 0,
    H26_T4_FAULT_ILLEGAL_STATE
} H26_Task4Fault_t;

typedef enum
{
    H26_T4_RESULT_RUNNING = 0,
    H26_T4_RESULT_FINISHED,
    H26_T4_RESULT_FAULT
} H26_Task4Result_t;

void H26_Task4_Init(void);
void H26_Task4_Reset(void);
void H26_Task4_Start(uint32_t startMs);
void H26_Task4_ForceFault(void);
H26_Task4Result_t H26_Task4_Task10ms(uint32_t nowMs);
/* Used after B is passed: vehicle remains stopped while the ball stays at O. */
void H26_Task4_HoldBall10ms(uint32_t nowMs);

H26_Task4State_t H26_Task4_GetState(void);
H26_Task4Fault_t H26_Task4_GetFault(void);
uint32_t H26_Task4_GetElapsedMs(uint32_t nowMs);
uint32_t H26_Task4_GetFinalElapsedMs(void);
float H26_Task4_GetDistanceCm(void);
uint8_t H26_Task4_IsBPassed(void);
uint32_t H26_Task4_GetBPassMs(void);
uint16_t H26_Task4_GetOAcquireHoldMs(void);
float H26_Task4_GetCommandForwardSpeed(void);
float H26_Task4_GetBallPeakErrorCm(void);
float H26_Task4_GetBallPositionCm(void);
float H26_Task4_GetBallSpeedCmps(void);
int32_t H26_Task4_GetRodEncoderCount(void);
int32_t H26_Task4_GetRodTargetCount(void);
float H26_Task4_GetTiltCommandMm(void);

#endif
