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
    H26_T3_DONE_HOLD,
    H26_T3_FAULT
} H26_Task3State_t;

typedef enum
{
    H26_T3_FAULT_NONE = 0,
    H26_T3_FAULT_CONFIGURATION,
    H26_T3_FAULT_ACQUIRE_TIMEOUT,
    H26_T3_FAULT_RUN_TIMEOUT,
    H26_T3_FAULT_VISION_TIMEOUT,
    H26_T3_FAULT_VISION_JUMP,
    H26_T3_FAULT_RAW_END_GUARD,
    H26_T3_FAULT_ROD_SOFT_LIMIT,
    H26_T3_FAULT_ROD_STALL,
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
float H26_Task3_GetPositionCm(void);
float H26_Task3_GetBallSpeedCmps(void);
float H26_Task3_GetTargetCm(void);
float H26_Task3_GetErrorCm(void);
int32_t H26_Task3_GetCommandHz(void);
uint8_t H26_Task3_IsVisionValid(void);
uint8_t H26_Task3_GetConfidence(void);
uint32_t H26_Task3_GetFrameAgeMs(void);
uint16_t H26_Task3_GetRawPositionCentiCm(void);
uint16_t H26_Task3_GetOriginCentiCm(void);
uint8_t H26_Task3_IsOriginCalibrated(void);
uint16_t H26_Task3_GetLastSequence(void);
uint8_t H26_Task3_GetLastFlags(void);
uint16_t H26_Task3_GetStableHoldMs(void);
float H26_Task3_GetPlusHoldPeakErrorCm(void);
float H26_Task3_GetMinusHoldPeakErrorCm(void);
int32_t H26_Task3_GetRodEncoderCount(void);
int32_t H26_Task3_GetRodTargetCount(void);
float H26_Task3_GetTiltCommandMm(void);
uint8_t H26_Task3_IsRodSoftLimitActive(void);
H26_Task3Fault_t H26_Task3_GetFault(void);

#endif
