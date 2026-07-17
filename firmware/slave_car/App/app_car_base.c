#include "app_car_base.h"
#include "app_config.h"
#include "app_car_state.h"
#include "app_control.h"
#include "app_line.h"
#include "BeepLed.h"
#include "Encoder.h"
#include "Motor.h"
#include "OLED.h"
#include "Serial.h"
#include <stdint.h>

static volatile uint16_t s_promptTimer = 0U;
static volatile uint8_t s_promptActive = 0U;

void CarBase_PromptTick1ms(void)
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

static void CarBase_PromptStart(uint16_t ms)
{
#if CAR_BASE_BOOT_PROMPT_ENABLE
    s_promptTimer = ms;
    s_promptActive = 1U;
    BeepLed_AllOn();
#else
    (void)ms;
#endif
}

void CarBase_Init(void)
{
    App_Control_Init();
    App_Line_ResetState();
    CarBase_StopAll();
    BeepLed_AllOff();
    CarBase_PromptStart(100U);
}

void CarBase_StopAll(void)
{
    App_Control_ForcePWMZero();
    Motor_StopAll();
    BeepLed_AllOff();
}

void CarBase_Task10ms(void)
{
    App_Control_ForcePWMZero();
}

void CarBase_KeyProcess(void)
{
}

void CarBase_Task100ms(void)
{
#if CAR_BASE_SERIAL_MONITOR_ENABLE
    Serial_Printf("[base,state,idle]\r\n");
    Serial_Printf("[base,enc,l=%ld,r=%ld]\r\n",
        (long)g_leftEncoderTotal, (long)g_rightEncoderTotal);
    Serial_Printf("[base,line,valid=%u,err=%d,mask=%u]\r\n",
        (unsigned int)g_lineValid, (int)g_lineError,
        (unsigned int)g_lineMask);
#endif
}

void CarBase_ShowStatus(void)
{
#if CAR_OLED_ENABLE
    OLED_Clear();
    OLED_ShowString(0, 0, "Car Base", OLED_6X8);
    OLED_ShowSignedNum(0, 16, (int32_t)g_lineError, 4, OLED_6X8);
    OLED_ShowNum(0, 32, (uint16_t)g_lineMask, 4, OLED_6X8);
#endif
}

void CarBase_Task200ms(void)
{
#if CAR_OLED_ENABLE
    CarBase_ShowStatus();
#endif
}
