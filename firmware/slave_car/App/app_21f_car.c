#include "app_21f_car.h"
#include "app_config.h"
#include "app_car_state.h"
#include "app_control.h"
#include "app_line.h"
#include "BeepLed.h"
#include "Encoder.h"
#include "Key.h"
#include "Motor.h"
#include "OLED.h"
#include "Serial.h"
#include "cmsis_compiler.h"
#include <stdint.h>

#define F21_LED_BLINK_ON_MS             300U
#define F21_LED_BLINK_OFF_MS            200U

#define F21_CROSS_BLACK_THRESH          8U

#define F21_CROSS_CONFIRM_TICKS         ((F21_CROSS_CONFIRM_MS + CAR_CONTROL_PERIOD_MS - 1U) / CAR_CONTROL_PERIOD_MS)

#define F21_FAR_LOST_CONFIRM_MS          50U
#define F21_FAR_LOST_CONFIRM_TICKS       ((F21_FAR_LOST_CONFIRM_MS + CAR_CONTROL_PERIOD_MS - 1U) / CAR_CONTROL_PERIOD_MS)

#define F21_RETURN_MODE_NONE            0U
#define F21_RETURN_MODE_SIMPLE          1U
#define F21_RETURN_MODE_FAR             2U

#define F21_VISION_START_DELAY_MS        500U
#define F21_VISION_UNLOCK_QUIET_MS       800U
#define F21_VISION_BUF_SIZE              32U

static void F21_Vision_Process(void);
static void F21_Vision_Tick10ms(void);
static uint8_t F21_Vision_DrainRx(void);

/*
 * Route table for rooms 1-8.
 * Rooms 1-4: SIMPLE, one cross + one turn -> final room run.
 * Rooms 5-8: FAR, first cross at 240cm, second cross after 70cm branch.
 *   secondTurn for rooms 5-8 are initial placeholders; verify on real track.
 */
static const F21Route_t s_routes[8] =
{
    { F21_ROUTE_SIMPLE,  15.0f, F21_TURN_LEFT,   0.0f, F21_TURN_LEFT,   35.0f },
    { F21_ROUTE_SIMPLE,  15.0f, F21_TURN_RIGHT,  0.0f, F21_TURN_RIGHT,  35.0f },
    { F21_ROUTE_SIMPLE, 120.0f, F21_TURN_LEFT,   0.0f, F21_TURN_LEFT,   35.0f },
    { F21_ROUTE_SIMPLE, 120.0f, F21_TURN_RIGHT,  0.0f, F21_TURN_RIGHT,  35.0f },
    /* room 5: first LEFT, second RIGHT */
    { F21_ROUTE_FAR,    180.0f, F21_TURN_LEFT,   70.0f, F21_TURN_RIGHT,  35.0f },
    /* room 6: first RIGHT, second LEFT */
    { F21_ROUTE_FAR,    180.0f, F21_TURN_RIGHT,  70.0f, F21_TURN_LEFT,   35.0f },
    /* room 7: first LEFT, second LEFT */
    { F21_ROUTE_FAR,    180.0f, F21_TURN_LEFT,   70.0f, F21_TURN_LEFT,   35.0f },
    /* room 8: first RIGHT, second RIGHT */
    { F21_ROUTE_FAR,    180.0f, F21_TURN_RIGHT,  70.0f, F21_TURN_RIGHT,  35.0f }
};

static const F21ReturnRoute_t s_returnRoutes[4] =
{
    /* room 1: outbound left, return right */
    { 20.0f, F21_TURN_RIGHT,  45.0f },

    /* room 2: outbound right, return left */
    { 20.0f, F21_TURN_LEFT,   45.0f },

    /* room 3: outbound left, return right */
    { 20.0f, F21_TURN_RIGHT, 135.0f },

    /* room 4: outbound right, return left */
    { 20.0f, F21_TURN_LEFT,  135.0f },
};

static const F21FarReturnRoute_t s_farReturnRoutes[4] =
{
    /* room 5: outbound first LEFT, second RIGHT -> return RIGHT, LEFT */
    { 10.0f, F21_TURN_RIGHT, 60.0f, F21_TURN_LEFT,  215.0f },

    /* room 6: outbound first RIGHT, second LEFT -> return LEFT, RIGHT */
    { 10.0f, F21_TURN_LEFT,  60.0f, F21_TURN_RIGHT, 215.0f },

    /* room 7: outbound first LEFT, second LEFT -> return RIGHT, RIGHT */
    { 10.0f, F21_TURN_RIGHT, 60.0f, F21_TURN_RIGHT, 215.0f },

    /* room 8: outbound first RIGHT, second RIGHT -> return LEFT, LEFT */
    { 10.0f, F21_TURN_LEFT,  60.0f, F21_TURN_LEFT,  215.0f },
};

static volatile F21CarState_t s_state = F21_CAR_IDLE;
static volatile uint8_t s_targetRoom = 1U;
static volatile uint8_t s_faultCode = 0U;

static volatile int32_t s_stateStartPulse = 0;
static volatile int32_t s_crossPulse = 0;
static volatile int32_t s_turnStartPulse = 0;
static volatile uint32_t s_stateMs = 0U;
static volatile uint8_t s_crossConfirmCnt = 0U;
static volatile uint8_t s_crossMonitoring = 0U;
static volatile int32_t s_detectStartPulse = 0;
static volatile int32_t s_secondDetectStartPulse = 0;
static volatile int32_t s_finalRunPulse = 0;
static volatile F21TurnDir_t s_firstTurnDir = F21_TURN_LEFT;
static volatile F21TurnDir_t s_secondTurnDir = F21_TURN_LEFT;
static volatile F21RouteType_t s_routeType = F21_ROUTE_SIMPLE;
static volatile uint8_t s_firstTurnDone = 0U;
static volatile uint8_t s_farLostConfirmCnt = 0U;

static volatile int32_t s_returnDetectStartPulse = 0;
static volatile int32_t s_returnFinalRunPulse = 0;
static volatile F21TurnDir_t s_returnTurnDir = F21_TURN_LEFT;

static volatile uint8_t s_returnMode = F21_RETURN_MODE_NONE;
static volatile uint8_t s_returnStage = 0U;
static volatile uint8_t s_returnSideConfirmCnt = 0U;
static volatile int32_t s_farReturnSecondDetectStartPulse = 0;
static volatile F21TurnDir_t s_farReturnSecondTurnDir = F21_TURN_LEFT;

static volatile uint8_t s_visionRoom = 0U;
static volatile uint8_t s_visionConfirmedRoom = 0U;
static volatile uint8_t s_visionStartPending = 0U;
static volatile uint32_t s_visionStartTick = 0U;
static volatile uint32_t s_visionMs = 0U;
static volatile uint8_t s_visionUnlockSent = 0U;
static volatile uint16_t s_visionUnlockQuietMs = 0U;

static char s_visionRxBuf[F21_VISION_BUF_SIZE];
static uint8_t s_visionRxIdx = 0U;

static volatile uint8_t s_visionLastFinishedRoom = 0U;
static volatile uint8_t s_visionBlockLastRoom = 0U;

static volatile uint8_t s_ledBlinkTarget = 0U;
static volatile uint8_t s_ledBlinkCount  = 0U;
static volatile uint16_t s_ledTimer      = 0U;
static volatile uint8_t s_ledActive      = 0U;
static volatile uint8_t s_ledOnPhase     = 0U;

static void F21_StartLedDisplay(uint8_t room)
{
    if (room < 1U || room > 8U) return;

    s_ledBlinkTarget = room;
    s_ledBlinkCount  = 0U;
    s_ledTimer       = 0U;
    s_ledActive      = 1U;
    s_ledOnPhase     = 1U;
    LED_User_On();
}

void F21Car_Tick1ms(void)
{
    if (!s_ledActive) return;

    s_ledTimer++;

    if (s_ledOnPhase)
    {
        if (s_ledTimer >= F21_LED_BLINK_ON_MS)
        {
            s_ledTimer = 0U;
            LED_User_Off();
            s_ledOnPhase = 0U;
            s_ledBlinkCount++;

            if (s_ledBlinkCount >= s_ledBlinkTarget)
            {
                s_ledActive = 0U;
                LED_User_Off();
            }
        }
    }
    else
    {
        if (s_ledTimer >= F21_LED_BLINK_OFF_MS)
        {
            s_ledTimer = 0U;
            s_ledOnPhase = 1U;
            LED_User_On();
        }
    }
}

/* ---- distance helpers ---- */

static int32_t F21_CmToPulse(float cm)
{
    return (int32_t)(cm / ECAR_CM_PER_PULSE + 0.5f);
}

static int32_t F21_GetDistanceFromPulse(int32_t startPulse)
{
    int32_t delta = g_forwardEncoderTotal - startPulse;
    return (delta >= 0) ? delta : -delta;
}

/* ---- safety ---- */

static void F21_SafeStop(void)
{
    App_Control_ForcePWMZero();
    Motor_StopAll();
}

static void F21_EnterFault(uint8_t code, const char *msg)
{
    (void)msg;
    F21_SafeStop();
    s_faultCode = code;
    s_state = F21_CAR_FAULT;
}

/* ---- encoder total clearing ---- */

static void F21_ClearEncoderTotals(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    g_leftEncoderDelta = 0;
    g_rightEncoderDelta = 0;
    g_leftEncoderTotal = 0;
    g_rightEncoderTotal = 0;
    g_forwardEncoderTotal = 0;
    g_turnEncoderTotal = 0;

    g_leftSpeed = 0.0f;
    g_rightSpeed = 0.0f;
    g_forwardSpeed = 0.0f;
    g_turnSpeed = 0.0f;

    g_leftPwm = 0;
    g_rightPwm = 0;
    g_speedPwm = 0.0f;
    g_diffPwm = 0.0f;
    g_forwardSpeedError = 0.0f;

    Encoder_ClearAll();

    if (primask == 0U)
    {
        __enable_irq();
    }
}

/* ---- cross detection ---- */

static uint8_t F21_IsCrossDetected(void)
{
    return (g_lineBlackCount >= F21_CROSS_BLACK_THRESH) ? 1U : 0U;
}

static uint8_t F21_CountMaskBits(uint8_t mask)
{
    uint8_t count = 0U;

    while (mask != 0U)
    {
        count += (uint8_t)(mask & 1U);
        mask >>= 1U;
    }

    return count;
}

static uint8_t F21_IsSideCrossDetected(F21TurnDir_t turnDir)
{
    uint8_t sideMask;

    if (turnDir == F21_TURN_RIGHT)
    {
        sideMask = (uint8_t)(g_lineMask & F21_SIDE_CROSS_RIGHT_MASK);
    }
    else
    {
        sideMask = (uint8_t)(g_lineMask & F21_SIDE_CROSS_LEFT_MASK);
    }

    return (F21_CountMaskBits(sideMask) >= F21_SIDE_CROSS_BLACK_MIN) ? 1U : 0U;
}

static float F21_GetCurrentCrossAdvanceCm(void)
{
    return (s_routeType == F21_ROUTE_FAR) ? F21_FAR_CROSS_ADVANCE_CM : F21_CROSS_ADVANCE_CM;
}

static float F21_GetCurrentLineBaseSpeed(void)
{
    return (s_routeType == F21_ROUTE_FAR) ? F21_FAR_LINE_BASE_SPEED_CMPS : F21_LINE_BASE_SPEED_CMPS;
}

/* ---- route config ---- */

static void F21_ApplyRoute(uint8_t room)
{
    if (room < 1U || room > 8U) return;

    const F21Route_t *r = &s_routes[room - 1U];
    s_routeType = r->type;
    s_detectStartPulse = F21_CmToPulse(r->firstDetectStartCm);
    s_firstTurnDir = r->firstTurn;
    s_secondDetectStartPulse = F21_CmToPulse(r->secondDetectStartCm);
    s_secondTurnDir = r->secondTurn;
    s_finalRunPulse = F21_CmToPulse(r->finalRunCm);
    s_firstTurnDone = 0U;
}

static uint8_t F21_ApplyReturnRoute(uint8_t room)
{
    if (room < 1U || room > 4U) return 0U;

    const F21ReturnRoute_t *r = &s_returnRoutes[room - 1U];
    s_returnDetectStartPulse = F21_CmToPulse(r->detectStartCm);
    s_returnTurnDir = r->turnDir;
    s_returnFinalRunPulse = F21_CmToPulse(r->finalRunCm);
    s_returnMode = F21_RETURN_MODE_SIMPLE;
    s_returnStage = 0U;
    return 1U;
}

static uint8_t F21_ApplyFarReturnRoute(uint8_t room)
{
    if (room < 5U || room > 8U) return 0U;

    const F21FarReturnRoute_t *r = &s_farReturnRoutes[room - 5U];

    s_returnMode = F21_RETURN_MODE_FAR;
    s_returnStage = 1U;

    s_returnDetectStartPulse = F21_CmToPulse(r->firstDetectStartCm);
    s_returnTurnDir = r->firstTurn;

    s_farReturnSecondDetectStartPulse = F21_CmToPulse(r->secondDetectStartCm);
    s_farReturnSecondTurnDir = r->secondTurn;

    s_returnFinalRunPulse = F21_CmToPulse(r->finalRunCm);

    s_returnSideConfirmCnt = 0U;

    return 1U;
}

/* ---- speed closed-loop motion command ---- */

static float F21_TurnDirToSign(F21TurnDir_t dir)
{
    return (dir == F21_TURN_LEFT) ? 1.0f : -1.0f;
}

static void F21_SetMotionCmd(float forward, float turn)
{
    g_targetForwardSpeed = forward;
    g_targetTurnSpeed = turn;
    g_carEnable = 1U;
    App_Control_ApplyMotorOutput();
}

/* ---- turn completion ---- */

static uint8_t F21_IsTurnPulseComplete(uint16_t targetPulse)
{
    if (targetPulse == 0U) return 0U;

    int32_t delta = g_turnEncoderTotal - s_turnStartPulse;
    if (delta < 0) delta = -delta;

    return (delta >= (int32_t)targetPulse) ? 1U : 0U;
}

static uint8_t F21_IsTurnComplete(void)
{
    return F21_IsTurnPulseComplete(F21_TURN_90_PULSE);
}

/* ---- public API ---- */

void F21Car_Init(void)
{
    s_state = F21_CAR_IDLE;
    s_targetRoom = 1U;
    s_faultCode = 0U;
    F21_SafeStop();
    App_Line_ResetState();
    F21_ClearEncoderTotals();
    s_firstTurnDone = 0U;
    s_ledActive = 0U;
    LED_User_Off();
    s_visionUnlockSent = 0U;
    s_visionUnlockQuietMs = 0U;
    s_visionRxIdx = 0U;
}

static void F21_ResetRunData(void)
{
    F21_SafeStop();
    F21_ClearEncoderTotals();
    App_Line_ResetState();
    App_Control_ResetPID();
    s_stateMs = 0U;
    s_crossConfirmCnt = 0U;
    s_farLostConfirmCnt = 0U;
    s_crossMonitoring = 0U;
    s_stateStartPulse = 0;
    s_crossPulse = 0;
    s_turnStartPulse = 0;
    s_firstTurnDone = 0U;
    s_returnMode = F21_RETURN_MODE_NONE;
    s_returnStage = 0U;
    s_returnSideConfirmCnt = 0U;
    s_farReturnSecondDetectStartPulse = 0;
    s_farReturnSecondTurnDir = F21_TURN_LEFT;
}

static void F21_StartSelectedRoomTask(const char *source)
{
    (void)source;
    if (s_targetRoom < 1U || s_targetRoom > 8U) return;

    F21_ApplyRoute(s_targetRoom);
    F21_ResetRunData();
    s_stateStartPulse = g_forwardEncoderTotal;
    s_state = F21_CAR_MAIN_LINE_RUN;
    s_visionStartPending = 0U;
    s_visionUnlockSent = 0U;
    s_visionUnlockQuietMs = 0U;
    s_visionRxIdx = 0U;
}

void F21Car_KeyProcess(void)
{
    uint8_t key = Key_GetNum();
    if (key == 0U) return;

    switch (key)
    {
    case 1U:
        s_visionStartPending = 0U;
        if (s_state == F21_CAR_IDLE || s_state == F21_CAR_WAIT_START)
        {
            s_targetRoom++;
            if (s_targetRoom > 8U) s_targetRoom = 1U;
            s_state = F21_CAR_WAIT_START;
            F21_StartLedDisplay(s_targetRoom);
        }
        break;
    case 2U:
        s_visionStartPending = 0U;
        if (s_state == F21_CAR_WAIT_START || s_state == F21_CAR_IDLE)
        {
            F21_StartSelectedRoomTask("key");
        }
        break;
    case 3U:
        s_visionStartPending = 0U;
        if (s_state != F21_CAR_IDLE && s_state != F21_CAR_WAIT_START &&
            s_state != F21_CAR_FINISH && s_state != F21_CAR_FAULT)
        {
            F21_SafeStop();
            s_state = F21_CAR_STOP;
        }
        break;
    case 4U:
        s_visionStartPending = 0U;
        s_visionUnlockSent = 0U;
    s_visionUnlockQuietMs = 0U;
    s_visionRxIdx = 0U;
    s_visionLastFinishedRoom = 0U;
    s_visionBlockLastRoom = 0U;
        s_targetRoom = 1U;
        F21_ResetRunData();
        s_state = F21_CAR_IDLE;
        s_faultCode = 0U;
        s_ledActive = 0U;
        LED_User_Off();
        break;
    default:
        break;
    }
}

/* ---- line run handler (shared) ---- */

static uint8_t F21_HandleLineRunCommon(uint8_t enableCross, int32_t detectStartPulse)
{
    App_Line_Update();
    if (!g_lineValid)
    {
        App_Control_ForcePWMZero();
        return 0U;
    }

    g_targetForwardSpeed = F21_GetCurrentLineBaseSpeed();
    g_targetTurnSpeed = App_Line_CalcTurnCmd();
    g_carEnable = 1U;
    App_Control_ApplyMotorOutput();

    if (!enableCross) return 0U;

    if (!s_crossMonitoring)
    {
        if (F21_GetDistanceFromPulse(s_stateStartPulse) >= detectStartPulse)
        {
            s_crossMonitoring = 1U;
            s_crossConfirmCnt = 0U;
        }
        return 0U;
    }

    if (F21_IsCrossDetected())
    {
        s_crossConfirmCnt++;
        if (s_crossConfirmCnt >= F21_CROSS_CONFIRM_TICKS)
        {
            s_crossPulse = g_forwardEncoderTotal;
            return 1U;
        }
    }
    else
    {
        s_crossConfirmCnt = 0U;
    }

    return 0U;
}

static uint8_t F21_HandleFarLineRunLostCross(int32_t detectStartPulse)
{
    App_Line_Update();

    if (!s_crossMonitoring)
    {
        if (F21_GetDistanceFromPulse(s_stateStartPulse) >= detectStartPulse)
        {
            s_crossMonitoring = 1U;
            s_farLostConfirmCnt = 0U;
        }

        if (g_lineValid)
        {
            g_targetForwardSpeed = F21_FAR_FAST_LINE_SPEED_CMPS;
            g_targetTurnSpeed = App_Line_CalcTurnCmd();
            g_carEnable = 1U;
            App_Control_ApplyMotorOutput();
        }
        else
        {
            App_Control_ForcePWMZero();
        }

        return 0U;
    }

    if (g_lineValid)
    {
        s_farLostConfirmCnt = 0U;

        g_targetForwardSpeed = F21_GetCurrentLineBaseSpeed();
        g_targetTurnSpeed = App_Line_CalcTurnCmd();
        g_carEnable = 1U;
        App_Control_ApplyMotorOutput();
        return 0U;
    }

    App_Control_ForcePWMZero();

    s_farLostConfirmCnt++;
    if (s_farLostConfirmCnt >= F21_FAR_LOST_CONFIRM_TICKS)
    {
        s_crossPulse = g_forwardEncoderTotal;
        s_farLostConfirmCnt = 0U;
        return 1U;
    }

    return 0U;
}

static float F21_GetReturnCrossAdvanceCm(void)
{
    return (s_returnMode == F21_RETURN_MODE_FAR) ? F21_FAR_CROSS_ADVANCE_CM : F21_CROSS_ADVANCE_CM;
}

static uint8_t F21_HandleFarReturnSideCrossRun(int32_t detectStartPulse, F21TurnDir_t turnDir)
{
    App_Line_Update();

    if (!g_lineValid)
    {
        App_Control_ForcePWMZero();
        return 0U;
    }

    g_targetForwardSpeed = F21_GetCurrentLineBaseSpeed();
    g_targetTurnSpeed = App_Line_CalcTurnCmd();
    g_carEnable = 1U;
    App_Control_ApplyMotorOutput();

    if (!s_crossMonitoring)
    {
        if (F21_GetDistanceFromPulse(s_stateStartPulse) >= detectStartPulse)
        {
            s_crossMonitoring = 1U;
            s_returnSideConfirmCnt = 0U;
        }
        return 0U;
    }

    if (F21_IsSideCrossDetected(turnDir))
    {
        s_returnSideConfirmCnt++;

        if (s_returnSideConfirmCnt >= F21_CROSS_CONFIRM_TICKS)
        {
            s_crossPulse = g_forwardEncoderTotal;
            s_returnSideConfirmCnt = 0U;
            return 1U;
        }
    }
    else
    {
        s_returnSideConfirmCnt = 0U;
    }

    return 0U;
}

/* ---- state handlers ---- */

static void F21_HandleMainLineRun(void)
{
    uint8_t crossed;

    if (s_routeType == F21_ROUTE_FAR)
    {
        crossed = F21_HandleFarLineRunLostCross(s_detectStartPulse);
    }
    else
    {
        crossed = F21_HandleLineRunCommon(1U, s_detectStartPulse);
    }

    if (crossed)
    {
        s_state = F21_CAR_FIRST_CROSS_ADVANCE;
    }
}

static void F21_HandleFirstCrossAdvance(void)
{
    static uint8_t s_firstAdvanceEntered = 0U;

    g_targetForwardSpeed = F21_CROSS_ADVANCE_SPEED_CMPS;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 1U;
    App_Control_ApplyMotorOutput();

    if (s_firstAdvanceEntered == 0U)
    {
        s_firstAdvanceEntered = 1U;
    }

    if (F21_GetDistanceFromPulse(s_crossPulse) >= F21_CmToPulse(F21_GetCurrentCrossAdvanceCm()))
    {
        s_firstAdvanceEntered = 0U;
        s_state = F21_CAR_FIRST_TURN;
        s_turnStartPulse = g_turnEncoderTotal;
    }
}

static void F21_HandleFirstTurn(void)
{
    if (F21_TURN_90_PULSE == 0)
    {
        F21_EnterFault(1U, "turn90_not_set");
        return;
    }

    F21_SetMotionCmd(0.0f,
        F21_TurnDirToSign(s_firstTurnDir) * F21_TURN_SPEED_CMPS);

    if (F21_IsTurnComplete())
    {
        F21_SafeStop();
        s_firstTurnDone = 1U;
        if (s_routeType == F21_ROUTE_SIMPLE)
        {
            s_stateStartPulse = g_forwardEncoderTotal;
            s_state = F21_CAR_FINAL_ROOM_RUN;
        }
        else
        {
            s_stateStartPulse = g_forwardEncoderTotal;
            s_crossMonitoring = 0U;
            s_crossConfirmCnt = 0U;
            s_farLostConfirmCnt = 0U;
            s_state = F21_CAR_AFTER_FIRST_TURN_RUN;
        }
    }
}

static void F21_HandleAfterFirstTurnRun(void)
{
    if (F21_HandleFarLineRunLostCross(s_secondDetectStartPulse))
    {
        s_state = F21_CAR_SECOND_CROSS_ADVANCE;
    }
}

static void F21_HandleSecondCrossAdvance(void)
{
    static uint8_t s_secondAdvanceEntered = 0U;

    g_targetForwardSpeed = F21_CROSS_ADVANCE_SPEED_CMPS;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 1U;
    App_Control_ApplyMotorOutput();

    if (s_secondAdvanceEntered == 0U)
    {
        s_secondAdvanceEntered = 1U;
    }

    if (F21_GetDistanceFromPulse(s_crossPulse) >= F21_CmToPulse(F21_GetCurrentCrossAdvanceCm()))
    {
        s_secondAdvanceEntered = 0U;
        s_state = F21_CAR_SECOND_TURN;
        s_turnStartPulse = g_turnEncoderTotal;
    }
}

static void F21_HandleSecondTurn(void)
{
    if (F21_TURN_90_PULSE == 0)
    {
        F21_EnterFault(1U, "turn90_not_set");
        return;
    }

    F21_SetMotionCmd(0.0f,
        F21_TurnDirToSign(s_secondTurnDir) * F21_TURN_SPEED_CMPS);

    if (F21_IsTurnComplete())
    {
        F21_SafeStop();
        s_stateStartPulse = g_forwardEncoderTotal;
        s_state = F21_CAR_FINAL_ROOM_RUN;
    }
}

static void F21_HandleFinalRoomRun(void)
{
    if (F21_GetDistanceFromPulse(s_stateStartPulse) >= s_finalRunPulse)
    {
        F21_SafeStop();
        s_state = F21_CAR_ARRIVED_ROOM;
        s_stateMs = 0U;
        return;
    }

    App_Line_Update();
    if (!g_lineValid)
    {
        App_Control_ForcePWMZero();
        return;
    }

    g_targetForwardSpeed = F21_GetCurrentLineBaseSpeed();
    g_targetTurnSpeed = App_Line_CalcTurnCmd();
    g_carEnable = 1U;
    App_Control_ApplyMotorOutput();
}

static void F21_HandleTurnAround(void)
{
    if (F21_TURN_180_PULSE == 0U)
    {
        F21_EnterFault(2U, "turn180_not_set");
        return;
    }

    F21_SetMotionCmd(0.0f,
        F21_TurnDirToSign(F21_TURN_RIGHT) * F21_TURN_SPEED_CMPS);

    if (F21_IsTurnPulseComplete(F21_TURN_180_PULSE))
    {
        F21_SafeStop();

        if (s_targetRoom >= 1U && s_targetRoom <= 4U)
        {
            if (F21_ApplyReturnRoute(s_targetRoom))
            {
                s_crossMonitoring = 0U;
                s_crossConfirmCnt = 0U;
                s_returnSideConfirmCnt = 0U;
                s_stateStartPulse = g_forwardEncoderTotal;
                s_state = F21_CAR_RETURN_MAIN_LINE_RUN;
            }
            else
            {
                s_state = F21_CAR_FINISH;
            }
        }
        else if (s_targetRoom >= 5U && s_targetRoom <= 8U)
        {
            if (F21_ApplyFarReturnRoute(s_targetRoom))
            {
                s_crossMonitoring = 0U;
                s_crossConfirmCnt = 0U;
                s_returnSideConfirmCnt = 0U;
                s_stateStartPulse = g_forwardEncoderTotal;
                s_state = F21_CAR_RETURN_MAIN_LINE_RUN;
            }
            else
            {
                s_state = F21_CAR_FINISH;
            }
        }
        else
        {
            s_state = F21_CAR_FINISH;
        }
    }
}

static void F21_HandleReturnMainLineRun(void)
{
    uint8_t crossed = 0U;

    if (s_returnMode == F21_RETURN_MODE_FAR)
    {
        crossed = F21_HandleFarReturnSideCrossRun(s_returnDetectStartPulse, s_returnTurnDir);
    }
    else
    {
        crossed = F21_HandleLineRunCommon(1U, s_returnDetectStartPulse);
    }

    if (crossed)
    {
        s_state = F21_CAR_RETURN_CROSS_ADVANCE;

        if (s_returnMode == F21_RETURN_MODE_FAR)
        {
        }
        else
        {
        }
    }
}

static void F21_HandleReturnCrossAdvance(void)
{
    static uint8_t s_returnAdvanceEntered = 0U;

    g_targetForwardSpeed = F21_CROSS_ADVANCE_SPEED_CMPS;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 1U;
    App_Control_ApplyMotorOutput();

    if (s_returnAdvanceEntered == 0U)
    {
        s_returnAdvanceEntered = 1U;
    }

    if (F21_GetDistanceFromPulse(s_crossPulse) >= F21_CmToPulse(F21_GetReturnCrossAdvanceCm()))
    {
        s_returnAdvanceEntered = 0U;
        s_state = F21_CAR_RETURN_TURN;
        s_turnStartPulse = g_turnEncoderTotal;
    }
}

static void F21_HandleReturnTurn(void)
{
    if (F21_TURN_90_PULSE == 0U)
    {
        F21_EnterFault(3U, "return_turn90_not_set");
        return;
    }

    F21_SetMotionCmd(0.0f,
        F21_TurnDirToSign(s_returnTurnDir) * F21_TURN_SPEED_CMPS);

    if (F21_IsTurnComplete())
    {
        F21_SafeStop();

        if (s_returnMode == F21_RETURN_MODE_FAR && s_returnStage == 1U)
        {
            s_returnStage = 2U;
            s_returnDetectStartPulse = s_farReturnSecondDetectStartPulse;
            s_returnTurnDir = s_farReturnSecondTurnDir;

            s_stateStartPulse = g_forwardEncoderTotal;
            s_crossMonitoring = 0U;
            s_crossConfirmCnt = 0U;
            s_returnSideConfirmCnt = 0U;

            s_state = F21_CAR_RETURN_MAIN_LINE_RUN;
        }
        else
        {
            s_stateStartPulse = g_forwardEncoderTotal;
            s_state = F21_CAR_RETURN_FINAL_RUN;
        }
    }
}

static void F21_HandleReturnFinalRun(void)
{
    App_Line_Update();
    if (!g_lineValid)
    {
        App_Control_ForcePWMZero();
        return;
    }

    if (F21_GetDistanceFromPulse(s_stateStartPulse) >= s_returnFinalRunPulse)
    {
        F21_SafeStop();
        s_turnStartPulse = g_turnEncoderTotal;
        s_state = F21_CAR_RETURN_FINAL_TURN_AROUND;
    }
    else
    {
        if (s_returnMode == F21_RETURN_MODE_FAR)
        {
            g_targetForwardSpeed = F21_FAR_FAST_LINE_SPEED_CMPS;
        }
        else
        {
            g_targetForwardSpeed = F21_LINE_BASE_SPEED_CMPS;
        }
        g_targetTurnSpeed = App_Line_CalcTurnCmd();
        g_carEnable = 1U;
        App_Control_ApplyMotorOutput();
    }
}

static void F21_HandleReturnFinalTurnAround(void)
{
    if (F21_TURN_180_PULSE == 0U)
    {
        F21_EnterFault(4U, "final_turn180_not_set");
        return;
    }

    F21_SetMotionCmd(0.0f,
        F21_TurnDirToSign(F21_TURN_RIGHT) * F21_TURN_SPEED_CMPS);

    if (F21_IsTurnPulseComplete(F21_TURN_180_PULSE))
    {
        F21_SafeStop();
        s_state = F21_CAR_FINISH;
    }
}

/* ---- main 10ms task ---- */

void F21Car_Task10ms(void)
{
    s_stateMs += CAR_CONTROL_PERIOD_MS;

    if (s_state == F21_CAR_FINISH)
    {
        F21_SafeStop();
#if ENABLE_K230
        {
            uint8_t drained;

            if (s_visionUnlockSent == 0U)
            {
                Serial_SendString("[num,unlock]\r\n");
                s_ledActive = 0U;
                LED_User_BlinkTimes(2U, 150U);
                s_visionUnlockSent = 1U;
                s_visionUnlockQuietMs = 0U;

                if (s_targetRoom >= 1U && s_targetRoom <= 8U)
                {
                    s_visionLastFinishedRoom = s_targetRoom;
                    s_visionBlockLastRoom = 1U;
                }
            }

            drained = F21_Vision_DrainRx();

            s_visionStartPending = 0U;
            s_visionRoom = 0U;
            s_visionConfirmedRoom = 0U;

            if (drained)
            {
                s_visionUnlockQuietMs = 0U;
            }
            else if (s_visionUnlockQuietMs < F21_VISION_UNLOCK_QUIET_MS)
            {
                s_visionUnlockQuietMs += CAR_CONTROL_PERIOD_MS;
            }
            else
            {
                s_visionUnlockQuietMs = 0U;
                s_state = F21_CAR_IDLE;
            }
        }
#else
        s_state = F21_CAR_IDLE;
#endif
        return;
    }

#if ENABLE_K230
    F21_Vision_Process();
    F21_Vision_Tick10ms();
#endif

    switch (s_state)
    {
    case F21_CAR_IDLE:
    case F21_CAR_WAIT_START:
        App_Control_ForcePWMZero();
        break;

    case F21_CAR_MAIN_LINE_RUN:
        F21_HandleMainLineRun();
        break;

    case F21_CAR_FIRST_CROSS_ADVANCE:
        F21_HandleFirstCrossAdvance();
        break;

    case F21_CAR_FIRST_TURN:
        F21_HandleFirstTurn();
        break;

    case F21_CAR_AFTER_FIRST_TURN_RUN:
        F21_HandleAfterFirstTurnRun();
        break;

    case F21_CAR_SECOND_CROSS_ADVANCE:
        F21_HandleSecondCrossAdvance();
        break;

    case F21_CAR_SECOND_TURN:
        F21_HandleSecondTurn();
        break;

    case F21_CAR_FINAL_ROOM_RUN:
        F21_HandleFinalRoomRun();
        break;

    case F21_CAR_ARRIVED_ROOM:
        F21_SafeStop();
        s_stateMs = 0U;
        s_state = F21_CAR_UNLOAD_WAIT;
        break;

    case F21_CAR_UNLOAD_WAIT:
        F21_SafeStop();
        if (s_stateMs >= F21_UNLOAD_WAIT_MS)
        {
            s_turnStartPulse = g_turnEncoderTotal;
            s_state = F21_CAR_TURN_AROUND;
        }
        break;

    case F21_CAR_TURN_AROUND:
        F21_HandleTurnAround();
        break;

    case F21_CAR_RETURN_MAIN_LINE_RUN:
        F21_HandleReturnMainLineRun();
        break;

    case F21_CAR_RETURN_CROSS_ADVANCE:
        F21_HandleReturnCrossAdvance();
        break;

    case F21_CAR_RETURN_TURN:
        F21_HandleReturnTurn();
        break;

    case F21_CAR_RETURN_FINAL_RUN:
        F21_HandleReturnFinalRun();
        break;

    case F21_CAR_RETURN_FINAL_TURN_AROUND:
        F21_HandleReturnFinalTurnAround();
        break;

    case F21_CAR_FINISH:
        F21_SafeStop();
        break;

    case F21_CAR_STOP:
        F21_SafeStop();
        break;

    case F21_CAR_FAULT:
        F21_SafeStop();
        break;

    default:
        break;
    }
}

void F21Car_Task100ms(void)
{
}

void F21Car_Task200ms(void)
{
#if CAR_OLED_ENABLE
    OLED_Clear();
    OLED_ShowString(0, 0, "21F Car", OLED_6X8);
    OLED_ShowSignedNum(0, 16, (int32_t)s_state, 2, OLED_6X8);
    OLED_ShowNum(0, 32, s_targetRoom, 2, OLED_6X8);
#endif
}

/* ---- K230 vision serial ---- */

static uint8_t F21_Vision_DrainRx(void)
{
    uint8_t byte;
    uint8_t drained = 0U;

    s_visionRxIdx = 0U;

    while (Serial_ReadByte(&byte))
    {
        drained = 1U;
    }

    return drained;
}

static void F21_Vision_ParseCommand(const char *buf)
{
    int32_t num = 0;
    const char *p;

    if (buf[0] != '[') return;

    if (buf[1] == 'n' && buf[2] == 'u' && buf[3] == 'm' && buf[4] == ',')
    {
        p = buf + 5;
        while (*p >= '0' && *p <= '9') { num = num * 10 + (int32_t)(*p - '0'); p++; }

        if (*p == ']')
        {
            if (num >= 1 && num <= 8)
            {
                s_visionRoom = (uint8_t)num;

                if (s_state == F21_CAR_WAIT_START && s_visionStartPending
                    && s_visionConfirmedRoom == (uint8_t)num)
                {
                    Serial_SendString("[num,");
                    Serial_SendByte((uint8_t)('0' + num));
                    Serial_SendString("]\r\n");
                    return;
                }

                if (s_visionBlockLastRoom
                    && s_visionLastFinishedRoom == (uint8_t)num)
                {
                    return;
                }

                if (s_state == F21_CAR_IDLE || s_state == F21_CAR_WAIT_START)
                {
                    s_visionBlockLastRoom = 0U;
                    s_visionConfirmedRoom = (uint8_t)num;
                    s_targetRoom = (uint8_t)num;
                    s_state = F21_CAR_WAIT_START;
                    F21_StartLedDisplay(s_targetRoom);

                    Serial_SendString("[num,");
                    Serial_SendByte((uint8_t)('0' + num));
                    Serial_SendString("]\r\n");

                    s_visionStartPending = 1U;
                    s_visionStartTick = s_visionMs;
                    s_visionUnlockSent = 0U;
                }
            }
        }
        else if (p[0] == ',' && p[1] == 'c' && p[2] == 'o' && p[3] == 'n'
            && p[4] == 'f' && p[5] == 'i' && p[6] == 'r' && p[7] == 'm'
            && p[8] == 'e' && p[9] == 'd' && p[10] == ',')
        {
            p += 11;
            num = 0;
            while (*p >= '0' && *p <= '9') { num = num * 10 + (int32_t)(*p - '0'); p++; }

            if (*p == ']' && num >= 1 && num <= 8)
            {
            }
        }
    }
}

static void F21_Vision_Process(void)
{
    uint8_t byte;

    while (Serial_ReadByte(&byte))
    {
        if (byte == '[')
        {
            s_visionRxIdx = 0U;
            if (s_visionRxIdx < sizeof(s_visionRxBuf) - 1U)
                s_visionRxBuf[s_visionRxIdx++] = (char)byte;
        }
        else if (s_visionRxIdx > 0U)
        {
            if (s_visionRxIdx < sizeof(s_visionRxBuf) - 1U)
                s_visionRxBuf[s_visionRxIdx++] = (char)byte;
            if (byte == ']')
            {
                s_visionRxBuf[s_visionRxIdx] = '\0';
                F21_Vision_ParseCommand(s_visionRxBuf);
                s_visionRxIdx = 0U;
            }
        }
    }
}

static void F21_Vision_Tick10ms(void)
{
    s_visionMs += 10U;

    if (s_visionStartPending)
    {
        if (s_visionMs - s_visionStartTick >= F21_VISION_START_DELAY_MS)
        {
            s_visionStartPending = 0U;

            if (s_visionConfirmedRoom >= 1U && s_visionConfirmedRoom <= 8U)
            {
                s_targetRoom = s_visionConfirmedRoom;
                F21_StartSelectedRoomTask("vision");
            }
        }
    }
}
