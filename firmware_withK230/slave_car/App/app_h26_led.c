#include "app_h26_led.h"
#include "app_h26_config.h"
#include "BeepLed.h"
#include <stdint.h>

/* LED_User_* is PB04 in this SysConfig/Board_Config mapping, never PB22/K1. */
static volatile H26_LedMode_t s_mode = H26_LED_OFF;
static volatile H26_LedMode_t s_restoreMode = H26_LED_OFF;
static volatile uint8_t s_taskBlinkTarget = 0U;
static volatile uint8_t s_taskBlinkDone = 0U;
static volatile uint8_t s_onPhase = 0U;
static volatile uint16_t s_phaseMs = 0U;

static void H26_LedSetOutput(uint8_t on)
{
    if (on != 0U)
    {
        LED_User_On();
    }
    else
    {
        LED_User_Off();
    }
}

static void H26_LedStartContinuous(H26_LedMode_t mode)
{
    s_mode = mode;
    s_phaseMs = 0U;
    s_onPhase = 0U;

    if (mode == H26_LED_RUNNING)
    {
        H26_LedSetOutput(1U);
    }
    else if (mode == H26_LED_FINISHED || mode == H26_LED_FAULT)
    {
        s_onPhase = 1U;
        H26_LedSetOutput(1U);
    }
    else
    {
        H26_LedSetOutput(0U);
    }
}

void H26_LedInit(void)
{
    LED_User_CancelBlink();
    s_mode = H26_LED_OFF;
    s_restoreMode = H26_LED_OFF;
    s_taskBlinkTarget = 0U;
    s_taskBlinkDone = 0U;
    s_onPhase = 0U;
    s_phaseMs = 0U;
    H26_LedSetOutput(0U);
}

void H26_LedSetMode(H26_LedMode_t mode)
{
    if (mode > H26_LED_FAULT)
    {
        mode = H26_LED_FAULT;
    }

    s_taskBlinkTarget = 0U;
    s_taskBlinkDone = 0U;
    s_restoreMode = H26_LED_OFF;
    H26_LedStartContinuous(mode);
}

void H26_LedShowTask(uint8_t taskNumber, H26_LedMode_t restoreMode)
{
    if (taskNumber < 2U || taskNumber > 6U)
    {
        H26_LedSetMode(H26_LED_FAULT);
        return;
    }

    if (restoreMode > H26_LED_FAULT || restoreMode == H26_LED_SHOW_TASK)
    {
        restoreMode = H26_LED_OFF;
    }

    LED_User_CancelBlink();
    s_mode = H26_LED_SHOW_TASK;
    s_restoreMode = restoreMode;
    s_taskBlinkTarget = taskNumber;
    s_taskBlinkDone = 0U;
    s_onPhase = 1U;
    s_phaseMs = 0U;
    H26_LedSetOutput(1U);
}

H26_LedMode_t H26_LedGetMode(void)
{
    return s_mode;
}

void H26_LedTick1ms(void)
{
    uint16_t intervalMs;

    if (s_mode == H26_LED_OFF || s_mode == H26_LED_RUNNING)
    {
        return;
    }

    if (s_phaseMs < 0xFFFFU)
    {
        s_phaseMs++;
    }

    if (s_mode == H26_LED_SHOW_TASK)
    {
        intervalMs = s_onPhase ? H26_LED_SHOW_ON_MS : H26_LED_SHOW_OFF_MS;
        if (s_phaseMs < intervalMs)
        {
            return;
        }

        s_phaseMs = 0U;
        if (s_onPhase != 0U)
        {
            s_onPhase = 0U;
            H26_LedSetOutput(0U);
            if (s_taskBlinkDone < 0xFFU)
            {
                s_taskBlinkDone++;
            }
            if (s_taskBlinkDone >= s_taskBlinkTarget)
            {
                H26_LedStartContinuous(s_restoreMode);
            }
        }
        else
        {
            s_onPhase = 1U;
            H26_LedSetOutput(1U);
        }
        return;
    }

    intervalMs = (s_mode == H26_LED_FINISHED) ?
        H26_LED_FINISHED_TOGGLE_MS : H26_LED_FAULT_TOGGLE_MS;
    if (s_phaseMs >= intervalMs)
    {
        s_phaseMs = 0U;
        s_onPhase = (s_onPhase == 0U) ? 1U : 0U;
        H26_LedSetOutput(s_onPhase);
    }
}
