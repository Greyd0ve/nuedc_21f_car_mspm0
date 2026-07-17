#include "BeepLed.h"
#include "Board_Config.h"

static volatile uint16_t s_beepRemainMs = 0U;
static volatile uint8_t  s_ledBlinkRemainToggles = 0U;
static volatile uint16_t s_ledBlinkIntervalMs = 0U;
static volatile uint16_t s_ledBlinkTimerMs = 0U;

void Beep_On(void)
{
    DL_GPIO_setPins(BEEP_PORT, BEEP_PIN);
}

void Beep_Off(void)
{
    DL_GPIO_clearPins(BEEP_PORT, BEEP_PIN);
}

void Beep_BeepMs(uint16_t ms)
{
    s_beepRemainMs = ms;
    if (ms > 0U)
    {
        Beep_On();
    }
    else
    {
        Beep_Off();
    }
}

void BeepLed_Tick1ms(void)
{
    if (s_beepRemainMs > 0U)
    {
        s_beepRemainMs--;
        if (s_beepRemainMs == 0U)
        {
            Beep_Off();
        }
    }

    if (s_ledBlinkRemainToggles == 0U)
    {
        return;
    }

    if (s_ledBlinkTimerMs > 0U)
    {
        s_ledBlinkTimerMs--;
    }
    if (s_ledBlinkTimerMs == 0U)
    {
        s_ledBlinkRemainToggles--;
        if (s_ledBlinkRemainToggles > 0U)
        {
            LED_User_Toggle();
            s_ledBlinkTimerMs = s_ledBlinkIntervalMs;
        }
        else
        {
            LED_User_Off();
        }
    }
}

void LED_User_On(void)
{
    DL_GPIO_setPins(LED_USER_PORT, LED_USER_PIN);
}

void LED_User_Off(void)
{
    DL_GPIO_clearPins(LED_USER_PORT, LED_USER_PIN);
}

void LED_User_Toggle(void)
{
    DL_GPIO_togglePins(LED_USER_PORT, LED_USER_PIN);
}

void LED_User_BlinkTimes(uint8_t times, uint16_t intervalMs)
{
    if (times == 0U || intervalMs == 0U)
    {
        s_ledBlinkRemainToggles = 0U;
        LED_User_Off();
        return;
    }

    s_ledBlinkRemainToggles = (uint8_t)(times * 2U);
    s_ledBlinkIntervalMs = intervalMs;
    s_ledBlinkTimerMs = intervalMs;
    LED_User_On();
}

void BeepLed_Init(void)
{
    s_beepRemainMs = 0U;
    Beep_Off();
    LED_User_Off();
}

void BeepLed_AllOn(void)
{
    Beep_On();
    LED_User_On();
}

void BeepLed_AllOff(void)
{
    s_beepRemainMs = 0U;
    s_ledBlinkRemainToggles = 0U;
    Beep_Off();
    LED_User_Off();
}

void BeepLed_AllTurn(void)
{
    DL_GPIO_togglePins(BEEP_PORT, BEEP_PIN);
    LED_User_Toggle();
}
