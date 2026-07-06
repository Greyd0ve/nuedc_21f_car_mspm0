#include "ti_msp_dl_config.h"
#include "app_config.h"
#include "OLED.h"
#include "Timer.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "Grayscale.h"
#include "BeepLed.h"
#include "app_control.h"
#include "app_line.h"
#include "app_e_car.h"
#include "app_e_serial.h"
#include "cmsis_compiler.h"
#include <stdint.h>

volatile uint8_t g_task_1ms_count = 0U;
volatile uint8_t g_task_5ms_count = 0U;
volatile uint8_t g_task_10ms_count = 0U;
volatile uint8_t g_task_100ms_count = 0U;
volatile uint8_t g_task_200ms_count = 0U;

static uint8_t Main_TakeTaskCounter(volatile uint8_t *counter)
{
    uint8_t hasTask = 0U;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (*counter > 0U)
    {
        (*counter)--;
        hasTask = 1U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    return hasTask;
}

int main(void)
{
    SYSCFG_DL_init();

    OLED_Init();
    Key_Init();
    Grayscale_Init();
    Motor_Init();
    Motor_StopAll();
    Encoder_Init();
    App_Line_GPIOForceInit();
    BeepLed_Init();
    Serial_Init();
    App_Control_Init();
    ECar_Init();
    ECar_Serial_Init();
    Timer_Init();

    OLED_Clear();
    ECar_ShowStatus();

    while (1)
    {
        if (Main_TakeTaskCounter(&g_task_1ms_count))
        {
            /* Reserved for light 1 ms services. Full grayscale processing stays out of ISR. */
        }

        if (Main_TakeTaskCounter(&g_task_5ms_count))
        {
            App_Control_UpdateEncoderSpeed();
        }

        if (Main_TakeTaskCounter(&g_task_10ms_count))
        {
#if ECAR_BOARD_TEST_MODE
            App_Line_Update();
#else
            ECar_Control10ms();
#endif
        }

        ECar_KeyProcess();
        ECar_SerialProcess();

        if (Main_TakeTaskCounter(&g_task_100ms_count))
        {
            ECar_SerialPlot100ms();
        }

        if (Main_TakeTaskCounter(&g_task_200ms_count))
        {
            ECar_ShowStatus();
        }
    }
}
