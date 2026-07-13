#include "Timer.h"
#include "app_config.h"
#include "Board_Config.h"
#include "Key.h"
#include "BeepLed.h"
#include "app_e_car.h"
#include <stdint.h>

extern volatile uint8_t g_task_1ms_count;
extern volatile uint8_t g_task_5ms_count;
extern volatile uint8_t g_task_10ms_count;
extern volatile uint8_t g_task_100ms_count;
extern volatile uint8_t g_task_200ms_count;

/* 饱和计数器只表示“有任务待处理”，避免 ISR 中无限累加。 */
static void Timer_SaturatingInc(volatile uint8_t *counter)
{
    if (*counter < ECAR_TASK_COUNT_MAX)
    {
        (*counter)++;
    }
}

void Timer_Init(void)
{
    /* 前台模块初始化完成后再启动 1ms 系统定时器。 */
    DL_TimerG_clearInterruptStatus(SYSTEM_TIMER_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(SYSTEM_TIMER_IRQN);
    NVIC_EnableIRQ(SYSTEM_TIMER_IRQN);
    DL_TimerG_startCounter(SYSTEM_TIMER_INST);
}

void TIMG6_IRQHandler(void)
{
    /* 分频计数器由 1ms tick 派生出各前台任务周期。 */
#if !ECAR_ENCODER_MINIMAL_DEBUG
    static uint8_t div5ms = 0U;
    static uint8_t div10ms = 0U;
    static uint8_t div100ms = 0U;
    static uint8_t div200ms = 0U;
#endif

    switch (DL_TimerG_getPendingInterrupt(SYSTEM_TIMER_INST))
    {
        case DL_TIMER_IIDX_ZERO:
#if ECAR_ENCODER_MINIMAL_DEBUG
            Timer_SaturatingInc(&g_task_1ms_count);
#else
            /* ISR 保持短小：只做按键消抖、提示 tick 和任务计数。 */
            Key_Tick();
            BeepLed_Tick1ms();
            ECar_PromptTick1ms();

            Timer_SaturatingInc(&g_task_1ms_count);

            div5ms++;
            if (div5ms >= ECAR_ENCODER_SPEED_PERIOD_MS)
            {
                div5ms = 0U;
                Timer_SaturatingInc(&g_task_5ms_count);
            }

            div10ms++;
            if (div10ms >= ECAR_CONTROL_PERIOD_MS)
            {
                div10ms = 0U;
                Timer_SaturatingInc(&g_task_10ms_count);
            }

            div100ms++;
            if (div100ms >= ECAR_SERIAL_PLOT_PERIOD_MS)
            {
                div100ms = 0U;
                Timer_SaturatingInc(&g_task_100ms_count);
            }

            div200ms++;
            if (div200ms >= ECAR_OLED_REFRESH_PERIOD_MS)
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
