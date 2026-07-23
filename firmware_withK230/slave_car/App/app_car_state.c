#include "app_car_state.h"
#include "app_config.h"
#include "app_tuning.h"
#include "Board_Config.h"
#include "PWM.h"
#include <stdint.h>

volatile float g_forwardKp = 2.0f;
volatile float g_forwardKi = 0.0f;
volatile float g_forwardKd = 0.0f;

volatile float g_turnKp = 0.0f;
volatile float g_turnKi = 0.0f;
volatile float g_turnKd = 0.0f;

volatile float g_pwmLimit = (float)PWM_MAX_DUTY;
volatile float g_targetForwardSpeed = 0.0f;
volatile float g_targetTurnSpeed = 0.0f;
volatile uint8_t g_carEnable = 0U;

volatile float g_leftSpeed = 0.0f;
volatile float g_rightSpeed = 0.0f;
volatile float g_forwardSpeed = 0.0f;
volatile float g_turnSpeed = 0.0f;
volatile float g_speedPwm = 0.0f;
volatile float g_diffPwm = 0.0f;
volatile float g_forwardSpeedError = 0.0f;
volatile int16_t g_leftEncoderDelta = 0;
volatile int16_t g_rightEncoderDelta = 0;
volatile int16_t g_leftPwm = 0;
volatile int16_t g_rightPwm = 0;

volatile int32_t g_leftEncoderTotal = 0;
volatile int32_t g_rightEncoderTotal = 0;
volatile int32_t g_forwardEncoderTotal = 0;
volatile int32_t g_turnEncoderTotal = 0;

volatile float g_lineBlackLevelF = 0.0f;

#if ECAR_REAR_LINE_SENSOR_MODE
volatile float g_lineReverseOrderF = 1.0f;
volatile float g_lineTurnSign = -1.0f;
#else
volatile float g_lineReverseOrderF = 0.0f;
volatile float g_lineTurnSign = 1.0f;
#endif
volatile float g_lineKp = TUNE_LINE_KP;
volatile float g_lineKd = TUNE_LINE_KD;
volatile float g_lineTurnLimit = TUNE_LINE_TURN_LIMIT_CMPS;
volatile float g_lineFilterAlpha = TUNE_LINE_FILTER_ALPHA;
volatile int16_t g_lineError = 0;
volatile uint8_t g_lineValid = 0U;
volatile uint8_t g_lineMask = 0U;
volatile uint8_t g_lineRawMask = 0U;
volatile uint8_t g_lineBlackCount = 0U;
volatile uint8_t g_lineBadMaskCount = 0U;
volatile uint8_t g_lineZeroMaskCount = 0U;
volatile int8_t g_lastLineDir = 1;
volatile uint16_t g_lineLostMs = 0U;
