#include "app_e_car.h"
#include "app_config.h"
#include "app_tuning.h"
#include "app_control.h"
#include "app_line.h"
#include "Board_Config.h"
#include "BeepLed.h"
#include "Encoder.h"
#include "Key.h"
#include "Motor.h"
#include "OLED.h"
#include "PWM.h"
#include "cmsis_compiler.h"
#include <stdint.h>

#define E_CAR_CONTROL_PERIOD_MS      ECAR_CONTROL_PERIOD_MS
#define E_CAR_TARGET_LAP_MIN         1U
#define E_CAR_TARGET_LAP_MAX         5U

/*
 * E_CAR_TURN_SIGN only affects the fixed corner turn direction.
 * Measured: rear mode (-1.0f) turned right; flipped to (+1.0f) for left turn.
 */
#if ECAR_REAR_LINE_SENSOR_MODE
#define E_CAR_TURN_SIGN              1.0f
#else
#define E_CAR_TURN_SIGN              1.0f
#endif

#define E_CAR_FAULT_NONE             0U
#define E_CAR_FAULT_LINE_LOST        1U

#define ECAR_LINE_ERROR_NORM          350.0f
#define ECAR_LINE_MIN_SPEED_CMPS      8.0f
#define ECAR_LINE_ERR_SLOW_GAIN       7.0f
#define ECAR_LINE_DERR_SLOW_GAIN      3.0f

#define ECAR_LINE_STRONG_ERR_TH          230
#define ECAR_LINE_STRONG_SPEED_CMPS      10.0f
#define ECAR_LINE_STRONG_TURN_CMPS       8.0f


ECarParam_t g_eCarParam =
{
    ECAR_DEFAULT_BASE_SPEED_CMPS,
    ECAR_DEFAULT_RECOVER_SPEED_CMPS,
    ECAR_DEFAULT_CORNER_FORWARD_CMPS,
    ECAR_DEFAULT_CORNER_TURN_CMPS,

    TUNE_LINE_KP,
    TUNE_LINE_KD,
    TUNE_LINE_TURN_LIMIT_CMPS,

    ECAR_DEFAULT_LAP_PULSE,
    160U
};

volatile float g_forwardKp = 2.0f;
volatile float g_forwardKi = 0.0f;
volatile float g_forwardKd = 0.0f;

volatile float g_turnKp = 0.0f;
volatile float g_turnKi = 0.0f;
volatile float g_turnKd = 0.0f;

volatile float g_pwmLimit = (float)PWM_MAX_DUTY * 0.55f;
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

volatile float g_lineBlackLevelF = 1.0f;

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

static volatile ECarState_t s_state = E_CAR_IDLE;
static volatile uint8_t s_targetLap = 1U;
static volatile uint8_t s_lapCount = 0U;
static volatile uint8_t s_cornerCount = 0U;
static volatile uint32_t s_runningTimeMs = 0U;
static volatile uint32_t s_stateMs = 0U;
static volatile uint32_t s_lostMs = 0U;
static volatile int32_t s_lastCornerForwardPulse = 0;
static volatile int32_t s_startForwardPulse = 0;
static volatile int32_t s_lapStartForwardPulse = 0;
static volatile int32_t s_currentLapPulse = 0;
static volatile float s_lapProgress = 0.0f;
static volatile uint8_t s_faultCode = E_CAR_FAULT_NONE;
static volatile uint16_t s_promptMs = 0U;
static volatile int32_t s_cornerTurnStartPulse = 0;
static volatile int32_t s_cornerAdvanceStartPulse = 0;

static float s_lineDError = 0.0f;
static float s_lastLineError = 0.0f;
static uint8_t s_lineDerivResetPending = 0U;

static float ECar_LimitFloat(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

static uint8_t ECar_IsMotionState(ECarState_t state)
{
    return (uint8_t)(state == E_CAR_LINE_RUN ||
                     state == E_CAR_CORNER_ENTER ||
                     state == E_CAR_CORNER_ADVANCE ||
                     state == E_CAR_CORNER_TURN);
}

static float ECar_CalcAdaptiveBaseSpeed(float error, float dError)
{
    float eAbs;
    float deAbs;
    float speed;

    eAbs = (error >= 0.0f) ? error : -error;
    deAbs = (dError >= 0.0f) ? dError : -dError;

    eAbs = eAbs / ECAR_LINE_ERROR_NORM;
    deAbs = deAbs / ECAR_LINE_ERROR_NORM;

    if (eAbs > 1.0f) eAbs = 1.0f;
    if (deAbs > 1.0f) deAbs = 1.0f;

    speed = g_eCarParam.base_speed
            - ECAR_LINE_ERR_SLOW_GAIN * eAbs
            - ECAR_LINE_DERR_SLOW_GAIN * deAbs;

    return ECar_LimitFloat(speed,
                           ECAR_LINE_MIN_SPEED_CMPS,
                           g_eCarParam.base_speed);
}

static int32_t ECar_GetForwardPulse(void)
{
    return g_forwardEncoderTotal;
}

static void ECar_SetState(ECarState_t state)
{
    s_state = state;
    s_stateMs = 0U;
}

static void ECar_PromptStart(uint16_t ms)
{
    s_promptMs = ms;
    if (ms > 0U) { BeepLed_AllOn(); }
    else         { BeepLed_AllOff(); }
}

void ECar_PromptTick1ms(void)
{
    if (s_promptMs > 0U)
    {
        s_promptMs--;
        if ((s_promptMs % 80U) == 0U) { BeepLed_AllTurn(); }
        if (s_promptMs == 0U)         { BeepLed_AllOff(); }
    }
}

static void ECar_SafeStop(void)
{
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 0U;
    App_Control_ForcePWMZero();
}

static void ECar_ClearEncoderTotals(void)
{
    __disable_irq();
    g_leftEncoderTotal = 0;
    g_rightEncoderTotal = 0;
    g_forwardEncoderTotal = 0;
    g_turnEncoderTotal = 0;
    g_leftEncoderDelta = 0;
    g_rightEncoderDelta = 0;
    Encoder_ClearAll();
    __enable_irq();
}

static void ECar_SyncLineParams(void)
{
    g_lineKp = g_eCarParam.line_kp;
    g_lineKd = g_eCarParam.line_kd;
    g_lineTurnLimit = g_eCarParam.turn_limit;
}

static void ECar_ResetLineState(void)
{
    App_Line_ResetState();
    g_lineError = 0;
    g_lineValid = 0U;
    g_lineMask = 0U;
    g_lineRawMask = 0U;
    g_lineBlackCount = 0U;
    g_lineBadMaskCount = 0U;
    g_lineZeroMaskCount = 0U;
    g_lastLineDir = 1;
    g_lineLostMs = 0U;
    s_lastLineError = 0.0f;
    s_lineDError = 0.0f;
    s_lineDerivResetPending = 1U;
}

static void ECar_ResetRunData(void)
{
    ECar_ClearEncoderTotals();
    ECar_ResetLineState();

    s_lapCount = 0U;
    s_cornerCount = 0U;
    s_runningTimeMs = 0U;
    s_stateMs = 0U;
    s_lostMs = 0U;
    s_lastCornerForwardPulse = 0;
    s_startForwardPulse = 0;
    s_lapStartForwardPulse = 0;
    s_currentLapPulse = 0;
    s_lapProgress = 0.0f;
    s_faultCode = E_CAR_FAULT_NONE;
    s_cornerAdvanceStartPulse = 0;
    s_cornerTurnStartPulse = 0;
}

static void ECar_UpdateLapProgress(void)
{
    int32_t pulse;
    int32_t lapPulse;
    int32_t defaultPulse;
    float progress;

    pulse = ECar_GetForwardPulse();
    lapPulse = pulse - s_lapStartForwardPulse;
    if (lapPulse < 0) { lapPulse = 0; }

    s_currentLapPulse = lapPulse;
    defaultPulse = g_eCarParam.lap_pulse_default;
    if (defaultPulse <= 0) { defaultPulse = 1; }

    progress = (float)lapPulse / (float)defaultPulse;
    s_lapProgress = ECar_LimitFloat(progress, 0.0f, 0.999f);
}

static float ECar_CalcLineTurnCmd(void)
{
    float error;
    float dError;
    float turn;

    error = (float)g_lineError;

    if (s_lineDerivResetPending)
    {
        dError = 0.0f;
        s_lineDerivResetPending = 0U;
    }
    else
    {
        dError = error - s_lastLineError;
    }

    s_lastLineError = error;
    s_lineDError = dError;

    turn = (-g_lineTurnSign) *
           (error * g_eCarParam.line_kp + dError * g_eCarParam.line_kd);

    return ECar_LimitFloat(turn,
                           -g_eCarParam.turn_limit,
                           g_eCarParam.turn_limit);
}

static void ECar_SetSpeedCmd(float forward, float turn)
{
    g_targetForwardSpeed = forward;
    g_targetTurnSpeed = turn;
    g_carEnable = 1U;
}

static void ECar_EnterFault(uint8_t faultCode)
{
    s_faultCode = faultCode;
    ECar_SafeStop();
    ECar_SetState(E_CAR_FAULT);
    ECar_PromptStart(900U);
}

static void ECar_EnterFinish(void)
{
    s_lapProgress = 0.999f;
    ECar_SafeStop();
    ECar_SetState(E_CAR_FINISH);
    ECar_PromptStart(600U);
}

/* ---- new corner detection utilities ---- */

static uint8_t ECar_IsAllWhiteLost(void)
{
    return (g_lineBlackCount == 0U && g_lineMask == 0U) ? 1U : 0U;
}

static int32_t ECar_GetCurrentSidePulse(void)
{
    int32_t pulse;

    pulse = ECar_GetForwardPulse() - s_lastCornerForwardPulse;
    if (pulse < 0) { pulse = -pulse; }

    return pulse;
}

static uint8_t ECar_IsCornerAllowedByDistance(void)
{
    if (s_cornerCount == 0U) { return 1U; }

    return (ECar_GetCurrentSidePulse() >= ECAR_CORNER_MIN_STRAIGHT_PULSE) ? 1U : 0U;
}

/* ---- state handlers ---- */

static void ECar_HandleLineRun(void)
{
    float turnCmd;
    float speedCmd;

    if (ECar_IsAllWhiteLost())
    {
        if (s_lostMs < 60000U)
        {
            s_lostMs += E_CAR_CONTROL_PERIOD_MS;
        }

        if (s_lostMs >= ECAR_LINE_LOST_FAULT_MS)
        {
            ECar_EnterFault(E_CAR_FAULT_LINE_LOST);
            return;
        }

        if (ECar_IsCornerAllowedByDistance() &&
            s_lostMs >= ECAR_CORNER_LOST_CONFIRM_MS)
        {
            ECar_SetState(E_CAR_CORNER_ENTER);
            return;
        }

        ECar_SetSpeedCmd(g_eCarParam.recover_speed, 0.0f);
        return;
    }

    s_lostMs = 0U;

    turnCmd = ECar_CalcLineTurnCmd();
    speedCmd = ECar_CalcAdaptiveBaseSpeed((float)g_lineError, s_lineDError);

    {
        int16_t errAbs;

        errAbs = g_lineError;
        if (errAbs < 0) { errAbs = (int16_t)(-errAbs); }

        if (errAbs >= ECAR_LINE_STRONG_ERR_TH)
        {
            speedCmd = ECAR_LINE_STRONG_SPEED_CMPS;

            if (turnCmd > ECAR_LINE_STRONG_TURN_CMPS)
            {
                turnCmd = ECAR_LINE_STRONG_TURN_CMPS;
            }
            else if (turnCmd < -ECAR_LINE_STRONG_TURN_CMPS)
            {
                turnCmd = -ECAR_LINE_STRONG_TURN_CMPS;
            }
        }
    }

    ECar_SetSpeedCmd(speedCmd, turnCmd);
}

static void ECar_HandleCornerEnter(void)
{
    int32_t pulse;

    pulse = ECar_GetForwardPulse();

    if (s_cornerCount < 250U) { s_cornerCount++; }

    if ((s_cornerCount % 4U) == 0U)
    {
        if (s_lapCount < 250U) { s_lapCount++; }

        if (s_lapCount >= s_targetLap)
        {
            ECar_EnterFinish();
            return;
        }

        s_lapStartForwardPulse = pulse;
        s_currentLapPulse = 0;
        s_lapProgress = 0.0f;
    }

    s_cornerAdvanceStartPulse = pulse;

    s_lostMs = 0U;
    App_Control_ResetPID();
    s_lineDerivResetPending = 1U;
    ECar_SetState(E_CAR_CORNER_ADVANCE);
}

static void ECar_HandleCornerAdvance(void)
{
    int32_t advancePulse;

    advancePulse = ECar_GetForwardPulse() - s_cornerAdvanceStartPulse;
    if (advancePulse < 0) { advancePulse = -advancePulse; }

    if (advancePulse >= ECAR_CORNER_ADVANCE_PULSE)
    {
        s_lastCornerForwardPulse = ECar_GetForwardPulse();
        s_cornerTurnStartPulse = g_turnEncoderTotal;

        s_lostMs = 0U;
        App_Control_ResetPID();
        s_lineDerivResetPending = 1U;
        ECar_SetState(E_CAR_CORNER_TURN);
        return;
    }

    ECar_SetSpeedCmd(g_eCarParam.recover_speed, 0.0f);
}

static void ECar_HandleCornerTurn(void)
{
    int32_t turnDelta;
    float turnCmd;

    turnDelta = g_turnEncoderTotal - s_cornerTurnStartPulse;
    if (turnDelta < 0) { turnDelta = -turnDelta; }

    if (turnDelta >= (int32_t)g_eCarParam.corner_turn_pulse)
    {
        App_Control_ResetPID();
        s_lineDerivResetPending = 1U;
        s_lostMs = 0U;
        ECar_SetState(E_CAR_LINE_RUN);
        return;
    }

    turnCmd = g_eCarParam.corner_turn_speed * E_CAR_TURN_SIGN;
    ECar_SetSpeedCmd(0.0f, turnCmd);
}

/* ---- public interface ---- */

void ECar_Init(void)
{
    ECar_SyncLineParams();
    s_targetLap = 1U;
    ECar_ResetRunData();
    ECar_SafeStop();
    ECar_SetState(E_CAR_IDLE);
}

void ECar_Reset(void)
{
    ECar_SyncLineParams();
    ECar_SafeStop();
    ECar_ResetRunData();
    ECar_SetState(E_CAR_READY);
}

void ECar_Start(void)
{
    if (s_targetLap < E_CAR_TARGET_LAP_MIN) { s_targetLap = E_CAR_TARGET_LAP_MIN; }
    if (s_targetLap > E_CAR_TARGET_LAP_MAX) { s_targetLap = E_CAR_TARGET_LAP_MAX; }

#if ECAR_BOARD_TEST_MODE
    ECar_SyncLineParams();
    ECar_SafeStop();
    ECar_ResetRunData();
    ECar_SetState(E_CAR_READY);
    ECar_PromptStart(120U);
    return;
#endif

    ECar_SyncLineParams();
    ECar_SafeStop();
    ECar_ResetRunData();

    s_startForwardPulse = ECar_GetForwardPulse();
    s_lapStartForwardPulse = s_startForwardPulse;
    s_lastCornerForwardPulse = s_startForwardPulse;
    ECar_SetState(E_CAR_LINE_RUN);
    g_carEnable = 1U;
    ECar_PromptStart(120U);
}

void ECar_Stop(void)
{
    ECar_SafeStop();
    ECar_ResetRunData();
    ECar_SetState(E_CAR_READY);
    ECar_PromptStart(220U);
}

void ECar_Control10ms(void)
{
    ECar_SyncLineParams();
    App_Line_Update();

    if (ECar_IsMotionState(s_state))
    {
        if (s_runningTimeMs <= 0xFFFFFFFFU - E_CAR_CONTROL_PERIOD_MS)
        {
            s_runningTimeMs += E_CAR_CONTROL_PERIOD_MS;
        }
    }

    if (s_stateMs <= 0xFFFFFFFFU - E_CAR_CONTROL_PERIOD_MS)
    {
        s_stateMs += E_CAR_CONTROL_PERIOD_MS;
    }

    ECar_UpdateLapProgress();
    App_Control_UpdatePIDParam();

    switch (s_state)
    {
        case E_CAR_IDLE:
        case E_CAR_READY:
        case E_CAR_FINISH:
        case E_CAR_FAULT:
            ECar_SafeStop();
            break;

        case E_CAR_LINE_RUN:
            ECar_HandleLineRun();
            break;

        case E_CAR_CORNER_ENTER:
            ECar_HandleCornerEnter();
            break;

        case E_CAR_CORNER_ADVANCE:
            ECar_HandleCornerAdvance();
            break;

        case E_CAR_CORNER_TURN:
            ECar_HandleCornerTurn();
            break;

        default:
            ECar_EnterFault(E_CAR_FAULT_LINE_LOST);
            break;
    }

    App_Control_ApplyMotorOutput();
}

void ECar_KeyProcess(void)
{
    uint8_t key;

    key = Key_GetNum();
    if (key == 0U) { return; }

    if (key == 1U)
    {
        if (!ECar_IsMotionState(s_state) && s_state != E_CAR_FAULT)
        {
            s_targetLap++;
            if (s_targetLap > E_CAR_TARGET_LAP_MAX) { s_targetLap = E_CAR_TARGET_LAP_MIN; }
            ECar_SafeStop();
            ECar_SetState(E_CAR_READY);
            LED_User_BlinkTimes(s_targetLap, 500U);
        }
        return;
    }

    if (key == 2U)
    {
        if (!ECar_IsMotionState(s_state) && s_state != E_CAR_FAULT)
        {
            ECar_Start();
        }
        return;
    }

    if (key == 3U)
    {
        ECar_Stop();
        return;
    }

    if (key == 4U)
    {
        if (s_state == E_CAR_FAULT)
        {
            ECar_Reset();
            ECar_PromptStart(180U);
        }
        else
        {
            ECar_SafeStop();
            ECar_ResetRunData();
            ECar_SetState(E_CAR_IDLE);
            ECar_PromptStart(260U);
        }
    }
}

void ECar_ShowStatus(void)
{
    uint8_t progressPercent;
    int32_t lineErr;

    if (!g_carEnable && (s_state == E_CAR_IDLE || s_state == E_CAR_READY))
    {
        OLED_Clear();
        OLED_ShowString(0, 0, "E-Car MSPM0", OLED_8X16);
        OLED_ShowString(0, 16, "OLED H8 I2C", OLED_8X16);
        OLED_ShowString(0, 32, "SCL PB9 SDA PB8", OLED_8X16);
        OLED_ShowString(0, 48, "Motor OFF", OLED_8X16);
        OLED_Update();
        return;
    }

    progressPercent = (uint8_t)(s_lapProgress * 100.0f);
    if (progressPercent > 99U) { progressPercent = 99U; }

    lineErr = (int32_t)g_lineError;
    if (lineErr > 999)  { lineErr = 999; }
    if (lineErr < -999) { lineErr = -999; }

    OLED_Clear();
    OLED_ShowString(0, 0, "E CAR", OLED_8X16);

    OLED_ShowString(0, 16, "N:", OLED_8X16);
    OLED_ShowNum(16, 16, s_targetLap, 1, OLED_8X16);
    OLED_ShowString(40, 16, "L:", OLED_8X16);
    OLED_ShowNum(56, 16, s_lapCount, 1, OLED_8X16);

    OLED_ShowString(0, 32, "C:", OLED_8X16);
    OLED_ShowNum(16, 32, s_cornerCount, 2, OLED_8X16);
    OLED_ShowString(48, 32, "S:", OLED_8X16);
    OLED_ShowNum(64, 32, (uint8_t)s_state, 1, OLED_8X16);
    OLED_ShowString(88, 32, "F:", OLED_8X16);
    OLED_ShowNum(104, 32, s_faultCode, 1, OLED_8X16);

    OLED_ShowString(0, 48, "P:", OLED_8X16);
    OLED_ShowNum(16, 48, progressPercent, 2, OLED_8X16);
    OLED_ShowString(32, 48, "% E:", OLED_8X16);
    OLED_ShowSignedNum(64, 48, lineErr, 3, OLED_8X16);

    OLED_Update();
}

float ECar_GetLapProgress(void)   { return s_lapProgress; }
uint8_t ECar_GetLapCount(void)    { return s_lapCount; }
uint8_t ECar_GetTargetLap(void)   { return s_targetLap; }
ECarState_t ECar_GetState(void)   { return s_state; }

uint8_t ECar_SetTargetLap(uint8_t lap)
{
    if (lap < E_CAR_TARGET_LAP_MIN || lap > E_CAR_TARGET_LAP_MAX) { return 0U; }
    if (ECar_IsMotionState(s_state)) { return 0U; }

    s_targetLap = lap;
    if (s_state != E_CAR_FAULT)
    {
        ECar_SafeStop();
        ECar_SetState(E_CAR_READY);
    }
    return 1U;
}

uint8_t ECar_GetCornerCount(void)  { return s_cornerCount; }
uint8_t ECar_GetFaultCode(void)    { return s_faultCode; }
uint32_t ECar_GetRunningTimeMs(void) { return s_runningTimeMs; }
