#ifndef __APP_H26_TASK2_H
#define __APP_H26_TASK2_H

#include <stdint.h>

typedef enum
{
    H26_T2_IDLE = 0,
    H26_T2_LEAVE_A,
    H26_T2_LAP_RUNNING,
    H26_T2_BRAKING,
    H26_T2_WAIT_STOP,
    H26_T2_DONE,
    H26_T2_FAULT
} H26_Task2State_t;

typedef enum
{
    H26_T2_SPEED_ZONE_STRAIGHT = 0,
    H26_T2_SPEED_ZONE_CURVE,
    H26_T2_SPEED_ZONE_FINISH
} H26_Task2SpeedZone_t;

typedef enum
{
    H26_T2_RESULT_RUNNING = 0,
    H26_T2_RESULT_FINISHED,
    H26_T2_RESULT_FAULT
} H26_Task2Result_t;

void H26_Task2_Init(void);
void H26_Task2_Start(uint32_t startMs);
void H26_Task2_Reset(void);
void H26_Task2_ForceFault(void);
/* Optional upper limit used by task 5's non-abrupt chassis launch. */
void H26_Task2_SetForwardSpeedLimit(float limitCmps);
H26_Task2Result_t H26_Task2_Task10ms(uint32_t nowMs);

H26_Task2State_t H26_Task2_GetState(void);
uint32_t H26_Task2_GetElapsedMs(uint32_t nowMs);
uint32_t H26_Task2_GetFinalElapsedMs(void);
float H26_Task2_GetDistanceCm(void);
uint16_t H26_Task2_GetBlackHoldMs(void);
uint16_t H26_Task2_GetStopHoldMs(void);
uint8_t H26_Task2_IsFinishEnabled(void);
uint8_t H26_Task2_IsFinishLatched(void);
uint32_t H26_Task2_GetFinishDetectMs(void);
int32_t H26_Task2_GetFinishDetectPulse(void);
H26_Task2SpeedZone_t H26_Task2_GetSpeedZone(void);
uint8_t H26_Task2_IsCurveMode(void);
float H26_Task2_GetCommandForwardSpeed(void);

#endif
