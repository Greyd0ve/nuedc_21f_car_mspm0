#include "app_f_car.h"
#include "app_config.h"
#include "app_tuning.h"
#include "app_control.h"
#include "app_line.h"
#include "app_load_detect.h"
#include "app_room_detect.h"
#include "Board_Config.h"
#include "BeepLed.h"
#include "Encoder.h"
#include "Key.h"
#include "Motor.h"
#include "OLED.h"
#include "PWM.h"
#include "Serial.h"
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

static volatile FCarState_t s_state = F_CAR_IDLE;
static volatile uint8_t s_targetRoom = 0U;
static volatile uint32_t s_runningTimeMs = 0U;
static volatile uint8_t s_faultCode = FCAR_FAULT_NONE;

static volatile uint16_t s_promptTimer = 0U;
static volatile uint8_t s_promptActive = 0U;

static void FCar_SetState(FCarState_t state)
{
    s_state = state;
}

static void FCar_SafeStop(void)
{
    App_Control_ForcePWMZero();
    Motor_StopAll();
}

static void FCar_PromptStart(uint16_t ms)
{
    s_promptTimer = ms;
    s_promptActive = 1U;
    BeepLed_AllOn();
}

static void FCar_EnterFault(uint8_t faultCode)
{
    FCar_SafeStop();
    s_faultCode = faultCode;
    FCar_SetState(F_CAR_FAULT);
    Serial_Printf("[fcar,fault,%u]\r\n", (unsigned int)faultCode);
    FCar_PromptStart(900U);
}

void FCar_PromptTick1ms(void)
{
    if (s_promptActive)
    {
        if (s_promptTimer > 0U)
        {
            s_promptTimer--;
        }
        else
        {
            s_promptActive = 0U;
            BeepLed_AllOff();
        }
    }
}

void FCar_Init(void)
{
    FCar_SafeStop();
    FCar_SetState(F_CAR_IDLE);
    s_targetRoom = 0U;
    s_runningTimeMs = 0U;
    s_faultCode = FCAR_FAULT_NONE;
    s_promptActive = 0U;
    s_promptTimer = 0U;
}

void FCar_Reset(void)
{
    FCar_SafeStop();
    FCar_SetState(F_CAR_IDLE);
    s_targetRoom = 0U;
    s_runningTimeMs = 0U;
    s_faultCode = FCAR_FAULT_NONE;
    s_promptActive = 0U;
    s_promptTimer = 0U;
}

void FCar_Start(void)
{
    if (s_targetRoom == 0U)
    {
        return;
    }
    if (!LoadDetect_IsLoaded())
    {
        return;
    }
    if (s_state != F_CAR_WAIT_LOAD)
    {
        return;
    }
    FCar_SetState(F_CAR_DELIVER_START);
}

void FCar_Stop(void)
{
    FCar_SafeStop();
    FCar_SetState(F_CAR_IDLE);
}

void FCar_SetRedLed(uint8_t on)
{
    if (on)
    {
        LED_User_On();
    }
    else
    {
        LED_User_Off();
    }
}

void FCar_SetGreenLed(uint8_t on)
{
    if (on)
    {
        LED_User_On();
    }
    else
    {
        LED_User_Off();
    }
}

void FCar_SetYellowLed(uint8_t on)
{
    if (on)
    {
        LED_User_On();
    }
    else
    {
        LED_User_Off();
    }
}

void FCar_AllLedOff(void)
{
    LED_User_Off();
    Beep_Off();
}

uint8_t FCar_GetTargetRoom(void)
{
    return s_targetRoom;
}

FCarState_t FCar_GetState(void)
{
    return s_state;
}

uint32_t FCar_GetRunningTimeMs(void)
{
    return s_runningTimeMs;
}

uint8_t FCar_GetFaultCode(void)
{
    return s_faultCode;
}

void FCar_SetTargetRoom(uint8_t room)
{
    if (room < 1U || room > 8U)
    {
        return;
    }
    if (s_state != F_CAR_IDLE && s_state != F_CAR_WAIT_ROOM_ID)
    {
        return;
    }
    s_targetRoom = room;
    FCar_SetState(F_CAR_WAIT_LOAD);
    Serial_Printf("[fcar,room,set,%u]\r\n", (unsigned int)room);
}

void FCar_SimulateLoadComplete(void)
{
    if (s_state == F_CAR_WAIT_LOAD)
    {
        LoadDetect_SetLoaded(1U);
    }
}

void FCar_SimulateUnloadComplete(void)
{
    if (s_state == F_CAR_WAIT_UNLOAD)
    {
        FCar_SetRedLed(0U);
        FCar_SetState(F_CAR_RETURN_START);
        Serial_Printf("[fcar,unload,done]\r\n");
    }
}

static void FCar_HandleDeliverStart(void)
{
    s_runningTimeMs = 0U;
    App_Control_ResetPID();
    App_Line_ResetState();
    Encoder_ClearAll();
    FCar_SetState(F_CAR_LINE_RUN);
    Serial_Printf("[fcar,deliver,start]\r\n");
}

static void FCar_HandleLineRun(void)
{
    App_Line_Update();
    if (!g_lineValid)
    {
        App_Control_ForcePWMZero();
        return;
    }
    App_Control_ApplyMotorOutput();
}

static void FCar_HandleRoomApproach(void)
{
    FCar_SafeStop();
    FCar_SetRedLed(1U);
    FCar_SetState(F_CAR_WAIT_UNLOAD);
    Serial_Printf("[fcar,room,arrived,%u]\r\n", (unsigned int)s_targetRoom);
}

static void FCar_HandleFinish(void)
{
    FCar_SafeStop();
    FCar_SetGreenLed(1U);
    Serial_Printf("[fcar,finish]\r\n");
}

static void FCar_HandleFault(void)
{
    FCar_SafeStop();
}

void FCar_Task10ms(void)
{
    switch (s_state)
    {
    case F_CAR_IDLE:
        break;
    case F_CAR_WAIT_ROOM_ID:
        break;
    case F_CAR_WAIT_LOAD:
        break;
    case F_CAR_DELIVER_START:
        FCar_HandleDeliverStart();
        break;
    case F_CAR_LINE_RUN:
        FCar_HandleLineRun();
        break;
    case F_CAR_INTERSECTION_HANDLE:
        break;
    case F_CAR_ROOM_APPROACH:
        FCar_HandleRoomApproach();
        break;
    case F_CAR_WAIT_UNLOAD:
        break;
    case F_CAR_RETURN_START:
        break;
    case F_CAR_RETURN_RUN:
        break;
    case F_CAR_FINISH:
        FCar_HandleFinish();
        break;
    case F_CAR_FAULT:
        FCar_HandleFault();
        break;
    default:
        break;
    }
}

void FCar_KeyProcess(void)
{
    uint8_t key = Key_GetNum();
    if (key == 0U)
    {
        return;
    }

    switch (key)
    {
    case 1U:
        break;
    case 2U:
        break;
    case 3U:
        break;
    case 4U:
        break;
    default:
        break;
    }
}

void FCar_ShowStatus(void)
{
#if FCAR_OLED_ENABLE
    OLED_Clear();
    OLED_ShowString(0, 0, "F Car", OLED_6X8);
    OLED_ShowNum(0, 16, (uint16_t)s_state, 2, OLED_6X8);
    OLED_ShowNum(0, 32, s_targetRoom, 2, OLED_6X8);
#endif
}
