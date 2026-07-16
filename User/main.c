#include "ti_msp_dl_config.h"
#include "app_config.h"
#include "app_board_test.h"
#include "app_car_base.h"
#include "app_control.h"
#include "app_line.h"
#include "BeepLed.h"
#include "Encoder.h"
#include "Grayscale.h"
#include "IMU.h"
#include "Key.h"
#include "Motor.h"
#include "OLED.h"
#include "Serial.h"
#include "Servo.h"
#include "Timer.h"
#include "cmsis_compiler.h"
#include <stdint.h>

volatile uint8_t g_task_1ms_count = 0U;
volatile uint8_t g_task_5ms_count = 0U;
volatile uint8_t g_task_10ms_count = 0U;
volatile uint8_t g_task_100ms_count = 0U;
volatile uint8_t g_task_200ms_count = 0U;

/*
 * mspm0g3507.sct places this section at the end of flash with +Last.
 * Keep this section, but keep it small to avoid MDK Lite 32KB limit.
 */
__attribute__((used, section(".rodata.flash_tail_pad")))
static const uint8_t s_flashTailPadForKeilFlm[0x10] = { 0U };

static uint8_t Main_TakeTaskCounterAll(volatile uint8_t *counter)
{
    uint8_t count;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    count = *counter;
    *counter = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return count;
}

#if CAR_ENCODER_MINIMAL_DEBUG
static void Main_PrintfSingleFieldTest(void)
{
    Serial_Printf("[printf-test]\r\n");
    Serial_Printf("risr=%lu\r\n", (unsigned long)Encoder_GetRightIsrCount());
    Serial_Printf("rign=%lu\r\n", (unsigned long)Encoder_GetRightSameAIgnored());
    Serial_Printf("rstat=%lu\r\n", (unsigned long)Encoder_GetRightStatusCount());
    Serial_Printf("rraw=%ld\r\n", (long)Encoder_GetRightLastRawDeltaBeforeLimit());
    Serial_Printf("rlim=%lu\r\n", (unsigned long)Encoder_GetRightLimitHitCount());
    Serial_Printf("rget=%lu\r\n", (unsigned long)Encoder_GetRightGetDeltaCount());
    Serial_Printf("rnz=%lu\r\n", (unsigned long)Encoder_GetRightNonZeroGetCount());
    Serial_Printf("rmax=%ld\r\n", (long)Encoder_GetRightMaxRawDelta());
}
#endif

int main(void)
{
    SYSCFG_DL_init();

#if CAR_ENCODER_MINIMAL_DEBUG
    Serial_Init();
    Encoder_Init();
    Encoder_DebugPrintDirectNoPrintf("[enc-direct-before-timer]");
    Timer_Init();
    Encoder_DebugPrintDirectNoPrintf("[enc-direct-after-timer]");

    while (1)
    {
        static uint16_t printMs = 0U;
        uint8_t taskCount = Main_TakeTaskCounterAll(&g_task_1ms_count);

        while (taskCount > 0U)
        {
            taskCount--;
            printMs++;
            if (printMs >= 1000U)
            {
                printMs = 0U;
                Encoder_DebugPrintDirectNoPrintf("[enc-direct-before-getdelta]");

				{
					int16_t rd = Encoder_GetRightDelta();
					Serial_Printf("[getdelta-test]\r\n");
					Serial_Printf("rd=%d\r\n", (int)rd);
				}

				Encoder_DebugPrintDirectNoPrintf("[enc-direct-after-getdelta]");
				Encoder_DebugPrintGetterNoPrintf("[enc-getter-after-getdelta]");
				Main_PrintfSingleFieldTest();
            }
        }
    }
#else
    Key_Init();
    Grayscale_Init();
    Motor_Init();
    Motor_StopAll();
    Encoder_Init();
    BeepLed_Init();
    Serial_Init();
    Servo_Init();
    CarBase_Init();

#if CAR_OLED_ENABLE
    OLED_Init();
    OLED_Clear();
#endif

#if CAR_TEST_IMU_ENABLE
    IMU_Init();
#endif

#if CAR_BOARD_TEST_MODE
    BoardTest_Init();
#else
    App_Line_GPIOForceInit();
#if CAR_OLED_ENABLE
    CarBase_ShowStatus();
#endif
#endif

    Timer_Init();

    while (1)
    {
        uint8_t taskCount;

        (void)Main_TakeTaskCounterAll(&g_task_1ms_count);

        taskCount = Main_TakeTaskCounterAll(&g_task_5ms_count);
        if (taskCount > 0U)
        {
            App_Control_UpdateEncoderSpeed((uint16_t)taskCount * CAR_ENCODER_SPEED_PERIOD_MS);
        }

        taskCount = Main_TakeTaskCounterAll(&g_task_10ms_count);
        if (taskCount > 2U)
        {
            taskCount = 2U;
        }
        while (taskCount > 0U)
        {
#if CAR_BOARD_TEST_MODE
            BoardTest_Task10ms();
#else
            CarBase_Task10ms();
#endif
            taskCount--;
        }

#if !CAR_BOARD_TEST_MODE
        CarBase_KeyProcess();
#endif

        if (Main_TakeTaskCounterAll(&g_task_100ms_count) > 0U)
        {
#if CAR_BOARD_TEST_MODE
            BoardTest_Task100ms();
#else
            CarBase_Task100ms();
#endif
        }

        if (Main_TakeTaskCounterAll(&g_task_200ms_count) > 0U)
        {
#if CAR_BOARD_TEST_MODE
            BoardTest_Task200ms();
#else
            CarBase_Task200ms();
#endif
        }
    }
#endif
}
