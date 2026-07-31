#ifndef __APP_H26_BALL_CONTROL_H
#define __APP_H26_BALL_CONTROL_H

#include <stdint.h>

/* A new valid K230 position was consumed, or the previous target is held. */
typedef enum
{
    H26_BALL_SAMPLE_NEW = 0,
    H26_BALL_SAMPLE_HELD
} H26_BallControlSample_t;

void H26_BallControl_Init(void);
void H26_BallControl_Reset(void);
void H26_BallControl_Start(void);
void H26_BallControl_Stop(void);

/* Consumes K230 position data without changing the rod command.  Task 3
 * uses this during its open-loop motion so the final PID shares its O origin. */
H26_BallControlSample_t H26_BallControl_Observe10ms(uint32_t nowMs);

/*
 * Runs one K230/rod-control iteration.  targetCm is expressed relative to
 * the O point automatically latched from the first valid frame after Start.
 */
H26_BallControlSample_t H26_BallControl_Task10ms(uint32_t nowMs,
                                                    float targetCm);

/*
 * PID variant for the stationary task-4 K230 closed-loop test.
 */
H26_BallControlSample_t H26_BallControl_Task10msWithPid(
    uint32_t nowMs,
    float targetCm,
    float positionKpMmPerCm,
    float positionKiMmPerCmS,
    float speedKdMmPerCmps,
    float integralLimitCmS,
    float tiltCommandLimitMm);

/* Adds a rod-position feed-forward command to the K230 PID result. */
H26_BallControlSample_t H26_BallControl_Task10msWithPidFeedForward(
    uint32_t nowMs,
    float targetCm,
    float positionKpMmPerCm,
    float positionKiMmPerCmS,
    float speedKdMmPerCmps,
    float integralLimitCmS,
    float tiltCommandLimitMm,
    float feedForwardTiltMm);

float H26_BallControl_GetPositionCm(void);
float H26_BallControl_GetBallSpeedCmps(void);
float H26_BallControl_GetTargetCm(void);
float H26_BallControl_GetErrorCm(void);
int32_t H26_BallControl_GetCommandHz(void);
uint8_t H26_BallControl_IsVisionValid(void);
uint8_t H26_BallControl_GetConfidence(void);
uint32_t H26_BallControl_GetFrameAgeMs(void);
uint16_t H26_BallControl_GetRawPositionCentiCm(void);
uint16_t H26_BallControl_GetOriginCentiCm(void);
uint8_t H26_BallControl_IsOriginCalibrated(void);
uint16_t H26_BallControl_GetLastSequence(void);
uint8_t H26_BallControl_GetLastFlags(void);
uint16_t H26_BallControl_GetStableSampleMs(void);
int32_t H26_BallControl_GetRodEncoderCount(void);
int32_t H26_BallControl_GetRodTargetCount(void);
float H26_BallControl_GetTiltCommandMm(void);
float H26_BallControl_GetPidTiltCommandMm(void);
float H26_BallControl_GetFeedForwardTiltMm(void);

#endif
