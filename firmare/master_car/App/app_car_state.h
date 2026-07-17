#ifndef __APP_CAR_STATE_H
#define __APP_CAR_STATE_H

#include <stdint.h>

extern volatile float g_forwardKp;
extern volatile float g_forwardKi;
extern volatile float g_forwardKd;

extern volatile float g_turnKp;
extern volatile float g_turnKi;
extern volatile float g_turnKd;

extern volatile float g_pwmLimit;
extern volatile float g_targetForwardSpeed;
extern volatile float g_targetTurnSpeed;
extern volatile uint8_t g_carEnable;

extern volatile float g_leftSpeed;
extern volatile float g_rightSpeed;
extern volatile float g_forwardSpeed;
extern volatile float g_turnSpeed;
extern volatile float g_speedPwm;
extern volatile float g_diffPwm;
extern volatile float g_forwardSpeedError;
extern volatile int16_t g_leftEncoderDelta;
extern volatile int16_t g_rightEncoderDelta;
extern volatile int16_t g_leftPwm;
extern volatile int16_t g_rightPwm;

extern volatile int32_t g_leftEncoderTotal;
extern volatile int32_t g_rightEncoderTotal;
extern volatile int32_t g_forwardEncoderTotal;
extern volatile int32_t g_turnEncoderTotal;

extern volatile float g_lineBlackLevelF;
extern volatile float g_lineReverseOrderF;
extern volatile float g_lineTurnSign;
extern volatile float g_lineKp;
extern volatile float g_lineKd;
extern volatile float g_lineTurnLimit;
extern volatile float g_lineFilterAlpha;
extern volatile int16_t g_lineError;
extern volatile uint8_t g_lineValid;
extern volatile uint8_t g_lineMask;
extern volatile uint8_t g_lineRawMask;
extern volatile uint8_t g_lineBlackCount;
extern volatile uint8_t g_lineBadMaskCount;
extern volatile uint8_t g_lineZeroMaskCount;
extern volatile int8_t g_lastLineDir;
extern volatile uint16_t g_lineLostMs;

#endif
