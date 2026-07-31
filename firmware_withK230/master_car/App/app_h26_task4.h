#ifndef __APP_H26_TASK4_H
#define __APP_H26_TASK4_H

#include <stdint.h>

typedef enum
{
    H26_T4_IDLE = 0,
    H26_T4_MANUAL_MOVE_HOLD_O,
    H26_T4_DRIVE_ACQUIRE_O,
    H26_T4_DRIVE_HOLD_O,
    H26_T4_DRIVE_STRAIGHT,
    H26_T4_DONE,
    H26_T4_FAULT,
    /* Kept after DONE/FAULT so their existing telemetry values stay stable. */
    H26_T4_DRIVE_CURVE
} H26_Task4State_t;

typedef enum
{
    H26_T4_FAULT_NONE = 0,
    H26_T4_FAULT_ILLEGAL_STATE,
    H26_T4_FAULT_TIMEOUT
} H26_Task4Fault_t;

typedef enum
{
    H26_T4_RESULT_RUNNING = 0,
    H26_T4_RESULT_FINISHED,
    H26_T4_RESULT_FAULT
} H26_Task4Result_t;

void H26_Task4_Init(void);
void H26_Task4_Reset(void);
/* K3: keep traction off and test ball PID + encoder feed-forward by hand. */
void H26_Task4_Start(uint32_t startMs);
/* K2: 130 cm straight line, ball PID and encoder acceleration feed-forward. */
void H26_Task4_StartDrive(uint32_t startMs);
void H26_Task4_ForceFault(void);
H26_Task4Result_t H26_Task4_Task10ms(uint32_t nowMs);
/* Arrival at B stops traction only; retain the rod controller for observation. */
void H26_Task4_HoldBall10ms(uint32_t nowMs);

H26_Task4State_t H26_Task4_GetState(void);
H26_Task4Fault_t H26_Task4_GetFault(void);
uint32_t H26_Task4_GetElapsedMs(uint32_t nowMs);
float H26_Task4_GetBallPositionCm(void);
float H26_Task4_GetBallErrorCm(void);
float H26_Task4_GetBallSpeedCmps(void);
float H26_Task4_GetTiltCommandMm(void);
float H26_Task4_GetPidTiltCommandMm(void);
float H26_Task4_GetFeedForwardTiltMm(void);
float H26_Task4_GetForwardSpeedCmps(void);
float H26_Task4_GetForwardAccelerationCmps2(void);
float H26_Task4_GetDistanceCm(void);
float H26_Task4_GetCommandForwardSpeedCmps(void);
int32_t H26_Task4_GetRodEncoderCount(void);
int32_t H26_Task4_GetRodTargetCount(void);
uint8_t H26_Task4_IsVisionValid(void);
uint8_t H26_Task4_IsOriginCalibrated(void);
uint8_t H26_Task4_GetVisionConfidence(void);

#endif
