#include "app_board_test.h"
#include "app_config.h"
#include "app_line.h"
#include "app_car_state.h"
#include "Board_Config.h"
#include "BeepLed.h"
#include "DebugSerial.h"
#include "Grayscale.h"
#include "Key.h"
#include "Motor.h"
#include <stdint.h>

#define BOARD_TEST_MOTOR_PWM 120

void BoardTest_Init(void)
{
    Motor_StopAll();

    DebugSerial_SendString("[board-test,start]\r\n");
    DebugSerial_SendString("[board-test,motor=enabled,pwm=120]\r\n");
    DebugSerial_SendString("[board-test,gray=enabled]\r\n");
    DebugSerial_SendString("[board-test,key,k1=left,k2=right,k3=both,k4=stop]\r\n");
}

static void BoardTest_PrintGray(void)
{
    uint8_t raw[8];
    uint8_t i;

    Grayscale_ReadAll(raw);
    App_Line_Update();

    DebugSerial_Printf("[gray,ch=");
    for (i = 0U; i < 8U; i++)
    {
        DebugSerial_Printf("%u", (unsigned int)raw[i]);
        if (i < 7U) DebugSerial_SendByte(',');
    }
    DebugSerial_SendString("]\r\n");

    DebugSerial_Printf("[gray,raw=0x%02X,mask=0x%02X,valid=%u,error=%d,count=%u]\r\n",
        (unsigned int)g_lineRawMask,
        (unsigned int)g_lineMask,
        (unsigned int)g_lineValid,
        (int)g_lineError,
        (unsigned int)g_lineBlackCount);
}

void BoardTest_Task10ms(void)
{
    uint8_t key = Key_GetNum();

    if (key != 0U)
    {
        switch (key)
        {
        case 1U:
            Motor_SetPWM((int16_t)BOARD_TEST_MOTOR_PWM, 0);
            DebugSerial_SendString("[test,motor,left,pwm=120]\r\n");
            break;
        case 2U:
            Motor_SetPWM(0, (int16_t)BOARD_TEST_MOTOR_PWM);
            DebugSerial_SendString("[test,motor,right,pwm=120]\r\n");
            break;
        case 3U:
            Motor_SetPWM((int16_t)BOARD_TEST_MOTOR_PWM, (int16_t)BOARD_TEST_MOTOR_PWM);
            DebugSerial_SendString("[test,motor,both,pwm=120]\r\n");
            break;
        case 4U:
            Motor_StopAll();
            DebugSerial_SendString("[test,motor,stop]\r\n");
            break;
        default:
            break;
        }
    }
}

void BoardTest_Task100ms(void)
{
    LED_User_Off();
}

void BoardTest_Task200ms(void)
{
    BoardTest_PrintGray();
}
