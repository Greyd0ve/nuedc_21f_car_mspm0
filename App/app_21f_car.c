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

#define F21_CROSS_CONFIRM_TICKS         (F21_CROSS_CONFIRM_MS / CAR_CONTROL_PERIOD_MS)

/*
 * Route table for rooms 1-8.
 * Rooms 1-4: SIMPLE, one cross + one turn -> final room run.
 * Rooms 5-8: FAR, first cross at 240cm, second cross after 70cm branch.
 *   secondTurn for rooms 5-8 are initial placeholders; verify on real track.
 */
static const F21Route_t s_routes[8] =
{
    { F21_ROUTE_SIMPLE,  50.0f, F21_TURN_LEFT,   0.0f, F21_TURN_LEFT,   40.0f },
    { F21_ROUTE_SIMPLE,  50.0f, F21_TURN_RIGHT,  0.0f, F21_TURN_RIGHT,  40.0f },
    { F21_ROUTE_SIMPLE, 150.0f, F21_TURN_LEFT,   0.0f, F21_TURN_LEFT,   40.0f },
    { F21_ROUTE_SIMPLE, 150.0f, F21_TURN_RIGHT,  0.0f, F21_TURN_RIGHT,  40.0f },
    /* room 5 secondTurn = LEFT  (placeholder, verify on track) */
    { F21_ROUTE_FAR,    240.0f, F21_TURN_LEFT,   70.0f, F21_TURN_LEFT,   50.0f },
    /* room 6 secondTurn = RIGHT (placeholder, verify on track) */
    { F21_ROUTE_FAR,    240.0f, F21_TURN_RIGHT,  70.0f, F21_TURN_RIGHT,  50.0f },
    /* room 7 secondTurn = RIGHT (placeholder, verify on track) */
    { F21_ROUTE_FAR,    240.0f, F21_TURN_LEFT,   70.0f, F21_TURN_RIGHT,  50.0f },
    /* room 8 secondTurn = LEFT  (placeholder, verify on track) */
    { F21_ROUTE_FAR,    240.0f, F21_TURN_RIGHT,  70.0f, F21_TURN_LEFT,   50.0f }
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
    F21_SafeStop();
    s_faultCode = code;
    s_state = F21_CAR_FAULT;
    Serial_Printf("[f21,fault,%s]\r\n", msg);
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

static uint8_t F21_IsTurnComplete(void)
{
    if (F21_TURN_90_PULSE == 0) return 0U;
    int32_t delta = g_turnEncoderTotal - s_turnStartPulse;
    if (delta < 0) delta = -delta;
    return (delta >= (int32_t)F21_TURN_90_PULSE) ? 1U : 0U;
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
}

static void F21_ResetRunData(void)
{
    F21_SafeStop();
    F21_ClearEncoderTotals();
    App_Line_ResetState();
    App_Control_ResetPID();
    s_stateMs = 0U;
    s_crossConfirmCnt = 0U;
    s_crossMonitoring = 0U;
    s_stateStartPulse = 0;
    s_crossPulse = 0;
    s_turnStartPulse = 0;
    s_firstTurnDone = 0U;
}

void F21Car_KeyProcess(void)
{
    uint8_t key = Key_GetNum();
    if (key == 0U) return;

    switch (key)
    {
    case 1U:
        if (s_state == F21_CAR_IDLE || s_state == F21_CAR_WAIT_START)
        {
            s_targetRoom++;
            if (s_targetRoom > 8U) s_targetRoom = 1U;
            s_state = F21_CAR_WAIT_START;
            F21_StartLedDisplay(s_targetRoom);
            Serial_Printf("[f21,room,%u]\r\n", (unsigned int)s_targetRoom);
        }
        break;
    case 2U:
        if (s_state == F21_CAR_WAIT_START || s_state == F21_CAR_IDLE)
        {
            if (s_targetRoom < 1U || s_targetRoom > 8U) break;
            F21_ApplyRoute(s_targetRoom);
            F21_ResetRunData();
            s_stateStartPulse = g_forwardEncoderTotal;
            s_state = F21_CAR_MAIN_LINE_RUN;
            Serial_Printf("[f21,start,room=%u]\r\n", (unsigned int)s_targetRoom);
        }
        break;
    case 3U:
        if (s_state != F21_CAR_IDLE && s_state != F21_CAR_WAIT_START &&
            s_state != F21_CAR_FINISH && s_state != F21_CAR_FAULT)
        {
            F21_SafeStop();
            s_state = F21_CAR_STOP;
            Serial_Printf("[f21,stop]\r\n");
        }
        break;
    case 4U:
        s_targetRoom = 1U;
        F21_ResetRunData();
        s_state = F21_CAR_IDLE;
        s_faultCode = 0U;
        s_ledActive = 0U;
        LED_User_Off();
        Serial_Printf("[f21,reset]\r\n");
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

    g_targetForwardSpeed = F21_LINE_BASE_SPEED_CMPS;
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

/* ---- state handlers ---- */

static void F21_HandleMainLineRun(void)
{
    if (F21_HandleLineRunCommon(1U, s_detectStartPulse))
    {
        s_state = F21_CAR_FIRST_CROSS_ADVANCE;
        Serial_Printf("[f21,cross,first]\r\n");
    }
}

static void F21_HandleFirstCrossAdvance(void)
{
    App_Line_Update();
    if (g_lineValid)
    {
        g_targetForwardSpeed = F21_CROSS_ADVANCE_SPEED_CMPS;
        g_targetTurnSpeed = 0.0f;
        g_carEnable = 1U;
        App_Control_ApplyMotorOutput();
    }
    else
    {
        App_Control_ForcePWMZero();
    }

    if (F21_GetDistanceFromPulse(s_crossPulse) >= F21_CmToPulse(F21_CROSS_ADVANCE_CM))
    {
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
            s_state = F21_CAR_AFTER_FIRST_TURN_RUN;
        }
    }
}

static void F21_HandleAfterFirstTurnRun(void)
{
    if (F21_HandleLineRunCommon(1U, s_secondDetectStartPulse))
    {
        s_state = F21_CAR_SECOND_CROSS_ADVANCE;
        Serial_Printf("[f21,cross,second]\r\n");
    }
}

static void F21_HandleSecondCrossAdvance(void)
{
    App_Line_Update();
    if (g_lineValid)
    {
        g_targetForwardSpeed = F21_CROSS_ADVANCE_SPEED_CMPS;
        g_targetTurnSpeed = 0.0f;
        g_carEnable = 1U;
        App_Control_ApplyMotorOutput();
    }
    else
    {
        App_Control_ForcePWMZero();
    }

    if (F21_GetDistanceFromPulse(s_crossPulse) >= F21_CmToPulse(F21_CROSS_ADVANCE_CM))
    {
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
    App_Line_Update();
    if (!g_lineValid)
    {
        App_Control_ForcePWMZero();
        return;
    }

    if (F21_GetDistanceFromPulse(s_stateStartPulse) >= s_finalRunPulse)
    {
        F21_SafeStop();
        s_state = F21_CAR_ARRIVED_ROOM;
        s_stateMs = 0U;
        Serial_Printf("[f21,arrived,room=%u]\r\n", (unsigned int)s_targetRoom);
    }
    else
    {
        g_targetForwardSpeed = F21_LINE_BASE_SPEED_CMPS;
        g_targetTurnSpeed = App_Line_CalcTurnCmd();
        g_carEnable = 1U;
        App_Control_ApplyMotorOutput();
    }
}

/* ---- main 10ms task ---- */

void F21Car_Task10ms(void)
{
    s_stateMs += CAR_CONTROL_PERIOD_MS;

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
            s_state = F21_CAR_FINISH;
        }
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
    Serial_Printf("[f21,state=%u,room=%u,dist=%ld,line=%d,mask=%u,black=%u]\r\n",
        (unsigned int)s_state,
        (unsigned int)s_targetRoom,
        (long)F21_GetDistanceFromPulse(s_stateStartPulse),
        (int)g_lineError,
        (unsigned int)g_lineMask,
        (unsigned int)g_lineBlackCount);
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
