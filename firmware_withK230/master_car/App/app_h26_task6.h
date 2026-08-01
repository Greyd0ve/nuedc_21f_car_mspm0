#ifndef __APP_H26_TASK6_H
#define __APP_H26_TASK6_H

#include <stdint.h>

typedef enum
{
    H26_T6_IDLE = 0,
    H26_T6_ACQUIRE_REFERENCE,
    H26_T6_PRETILT,
    H26_T6_LEAVE_A,
    H26_T6_LAP_RUNNING,
    H26_T6_TARGET_TRANSITION,
    H26_T6_DONE,
    H26_T6_FAULT
} H26_Task6State_t;

typedef enum
{
    H26_T6_FAULT_NONE = 0,
    H26_T6_FAULT_ILLEGAL_STATE,
    H26_T6_FAULT_TIMEOUT,
    H26_T6_FAULT_ROD_TRACKING
} H26_Task6Fault_t;

typedef enum
{
    H26_T6_RESULT_RUNNING = 0,
    H26_T6_RESULT_FINISHED,
    H26_T6_RESULT_FAULT
} H26_Task6Result_t;

typedef struct
{
    float distanceCm;
    float normalizedDistance;
    float positionCompensationCm;
    float kp;
    float ki;
    float kdStraight;
    float kdCurve;
    float feedForwardK;
} H26_Task6ScheduledControl_t;

void H26_Task6_Init(void);
void H26_Task6_Reset(void);
void H26_Task6_Start(uint32_t startMs);
void H26_Task6_ForceFault(void);
H26_Task6Result_t H26_Task6_Task10ms(uint32_t nowMs);
void H26_Task6_HoldBall10ms(uint32_t nowMs);

H26_Task6State_t H26_Task6_GetState(void);
H26_Task6Fault_t H26_Task6_GetFault(void);
uint32_t H26_Task6_GetElapsedMs(uint32_t nowMs);
uint32_t H26_Task6_GetFinalElapsedMs(void);
float H26_Task6_GetBallPeakErrorCm(void);
float H26_Task6_GetBallPositionCm(void);
float H26_Task6_GetBallErrorCm(void);
float H26_Task6_GetBallSpeedCmps(void);
float H26_Task6_GetTiltCommandMm(void);
float H26_Task6_GetPidTiltCommandMm(void);
float H26_Task6_GetFeedForwardTiltMm(void);
float H26_Task6_GetForwardSpeedCmps(void);
float H26_Task6_GetForwardAccelerationCmps2(void);
int32_t H26_Task6_GetRodEncoderCount(void);
int32_t H26_Task6_GetRodTargetCount(void);
uint8_t H26_Task6_IsVisionValid(void);
uint8_t H26_Task6_IsCurveMode(void);
float H26_Task6_GetScheduledKp(void);
float H26_Task6_GetScheduledKi(void);
float H26_Task6_GetScheduledKd(void);
float H26_Task6_GetScheduledFeedForwardK(void);
float H26_Task6_GetAbsoluteDistanceCm(void);
float H26_Task6_GetControlTargetCm(void);
uint8_t H26_Task6_IsSaturationActive(void);
uint8_t H26_Task6_IsSaturationLatched(void);

/* Exported for Task 4 to share the same distance-scheduled parameters. */
H26_Task6ScheduledControl_t H26_T6_GetScheduledControl(float absoluteDistanceCm);

#endif
