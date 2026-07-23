#include "app_task_mode.h"
#include "app_21f_car.h"
#include "app_coop.h"
#include "app_control.h"
#include "app_radio.h"
#include "BeepLed.h"
#include "DebugSerial.h"
#include "Key.h"
#include "Motor.h"
#include <stdint.h>

#define TASK_MODE_BLINK_INTERVAL_MS     150U

static volatile F21TaskMode_t s_taskMode = F21_TASK_MODE_BASIC;

static void App_TaskMode_SafeStop(void)
{
    App_Control_ForcePWMZero();
    Motor_StopAll();
}

static void App_TaskMode_CancelPb04Outputs(void)
{
    LED_User_CancelBlink();
    F21Car_CancelLedDisplay();
    F21Coop_CancelYellowWait();
    LED_User_Off();
}

void App_TaskMode_Init(void)
{
    s_taskMode = F21_TASK_MODE_BASIC;
    App_TaskMode_CancelPb04Outputs();
    DebugSerial_Printf("[mode,basic]\r\n");
}

F21TaskMode_t App_TaskMode_Get(void)
{
    return s_taskMode;
}

void App_TaskMode_Tick1ms(void)
{
    if (s_taskMode == F21_TASK_MODE_BASIC)
    {
        F21Car_Tick1ms();
    }
    else
    {
        F21Coop_Tick1ms();
    }
}

static uint8_t App_TaskMode_CanSwitchNow(void)
{
    if (s_taskMode == F21_TASK_MODE_BASIC)
    {
        return F21Car_IsModeSwitchAllowed();
    }

    return F21Coop_IsModeSwitchAllowed();
}

static void App_TaskMode_SwitchTo(F21TaskMode_t nextMode, uint8_t blinkTimes)
{
    App_TaskMode_SafeStop();
    App_Radio_ClearPendingCommands();

    F21Car_ResetTask();
    F21Coop_ResetTask();

    App_TaskMode_CancelPb04Outputs();
    s_taskMode = nextMode;

    if (nextMode == F21_TASK_MODE_BASIC)
    {
        DebugSerial_Printf("[mode,basic]\r\n");
    }
    else
    {
        DebugSerial_Printf("[mode,coop]\r\n");
    }

    LED_User_BlinkTimes(blinkTimes, TASK_MODE_BLINK_INTERVAL_MS);
}

void App_TaskMode_KeyProcess(void)
{
    uint8_t key = Key_GetNum();

    if (key == 0U) return;

    if (key == 3U && App_TaskMode_CanSwitchNow())
    {
        if (s_taskMode == F21_TASK_MODE_BASIC)
        {
            App_TaskMode_SwitchTo(F21_TASK_MODE_COOP, 2U);
        }
        else
        {
            App_TaskMode_SwitchTo(F21_TASK_MODE_BASIC, 1U);
        }
        return;
    }

    if (s_taskMode == F21_TASK_MODE_BASIC)
    {
        F21Car_HandleKey(key);
    }
    else
    {
        F21Coop_HandleKey(key);
    }
}
