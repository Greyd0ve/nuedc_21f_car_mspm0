#include "Timer.h"
#include "app_config.h"
#include "Board_Config.h"
#include "Key.h"
#include "BeepLed.h"
#include "app_car_base.h"
#include "app_task_mode.h"
#include <stdint.h>

extern volatile uint8_t g_task_1ms_count;
extern volatile uint8_t g_task_5ms_count;
extern volatile uint8_t g_task_10ms_count;
extern volatile uint8_t g_task_100ms_count;
extern volatile uint8_t g_task_200ms_count;

static void Timer_SaturatingInc(volatile uint8_t *counter)
{
    if (*counter < CAR_TASK_COUNT_MAX)
    {
        (*counter)++;
    }
}

void Timer_Init(void)
{
    DL_TimerG_clearInterruptStatus(SYSTEM_TIMER_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(SYSTEM_TIMER_IRQN);
    NVIC_EnableIRQ(SYSTEM_TIMER_IRQN);
    DL_TimerG_startCounter(SYSTEM_TIMER_INST);
}

void TIMG6_IRQHandler(void)
{
#if !CAR_ENCODER_MINIMAL_DEBUG
    static uint8_t div5ms = 0U;
    static uint8_t div10ms = 0U;
    static uint8_t div100ms = 0U;
    static uint8_t div200ms = 0U;
#endif

    switch (DL_TimerG_getPendingInterrupt(SYSTEM_TIMER_INST))
    {
        case DL_TIMER_IIDX_ZERO:
#if CAR_ENCODER_MINIMAL_DEBUG
            Timer_SaturatingInc(&g_task_1ms_count);
#else
            Key_Tick();
            BeepLed_Tick1ms();
            CarBase_PromptTick1ms();
            App_TaskMode_Tick1ms();

            Timer_SaturatingInc(&g_task_1ms_count);

            div5ms++;
            if (div5ms >= CAR_ENCODER_SPEED_PERIOD_MS)
            {
                div5ms = 0U;
                Timer_SaturatingInc(&g_task_5ms_count);
            }

            div10ms++;
            if (div10ms >= CAR_CONTROL_PERIOD_MS)
            {
                div10ms = 0U;
                Timer_SaturatingInc(&g_task_10ms_count);
            }

            div100ms++;
            if (div100ms >= CAR_SERIAL_PLOT_PERIOD_MS)
            {
                div100ms = 0U;
                Timer_SaturatingInc(&g_task_100ms_count);
            }

            div200ms++;
            if (div200ms >= CAR_OLED_REFRESH_PERIOD_MS)
            {
                div200ms = 0U;
                Timer_SaturatingInc(&g_task_200ms_count);
            }
#endif
            break;

        default:
            break;
    }
}
