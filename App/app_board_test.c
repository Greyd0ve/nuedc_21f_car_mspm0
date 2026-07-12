#include "app_board_test.h"
#include "app_config.h"
#include "Board_Config.h"
#include "BeepLed.h"
#include "Grayscale.h"
#include "IMU.h"
#include "Key.h"
#include "Motor.h"
#include "OLED.h"
#include "Serial.h"
#include "Servo.h"
#include <stdint.h>

extern volatile int32_t g_leftEncoderTotal;
extern volatile int32_t g_rightEncoderTotal;

static uint16_t s_ledBlinkMs = 0U;

static uint8_t BoardTest_ReadKeyLevel(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0U) ? 1U : 0U;
}

static int16_t BoardTest_CalcGrayError(const uint8_t raw[GRAYSCALE_CHANNELS])
{
    static const int8_t weight[GRAYSCALE_CHANNELS] = {-7, -5, -3, -1, 1, 3, 5, 7};
    int16_t sum = 0;
    int16_t count = 0;
    uint8_t i;

    for (i = 0U; i < GRAYSCALE_CHANNELS; i++)
    {
        if (raw[i] != 0U)
        {
            sum = (int16_t)(sum + weight[i]);
            count++;
        }
    }

    if (count == 0)
    {
        return 0;
    }

    return (int16_t)(sum / count);
}

static void BoardTest_PrintKey(void)
{
#if BOARD_OLED_H8_SPI_OWNS_KEY12
    Serial_Printf("[key] k1=H8 k2=H8 k3=%u k4=%u\r\n",
        (unsigned int)BoardTest_ReadKeyLevel(KEY3_PORT, KEY3_PIN),
        (unsigned int)BoardTest_ReadKeyLevel(KEY4_PORT, KEY4_PIN));
#else
    Serial_Printf("[key] k1=%u k2=%u k3=%u k4=%u\r\n",
        (unsigned int)BoardTest_ReadKeyLevel(KEY1_PORT, KEY1_PIN),
        (unsigned int)BoardTest_ReadKeyLevel(KEY2_PORT, KEY2_PIN),
        (unsigned int)BoardTest_ReadKeyLevel(KEY3_PORT, KEY3_PIN),
        (unsigned int)BoardTest_ReadKeyLevel(KEY4_PORT, KEY4_PIN));
#endif
}

static void BoardTest_PrintGray(void)
{
#if BOARD_OLED_H8_SPI_OWNS_GRAY_AD1
    Serial_SendString("[gray] skipped: H8 OLED uses PB10/GRAY_AD1\r\n");
#else
    uint8_t raw[GRAYSCALE_CHANNELS];
    int16_t error;

    Grayscale_ReadAll(raw);
    error = BoardTest_CalcGrayError(raw);

    Serial_Printf("[gray] raw=%u,%u,%u,%u,%u,%u,%u,%u err=%d\r\n",
        (unsigned int)raw[0],
        (unsigned int)raw[1],
        (unsigned int)raw[2],
        (unsigned int)raw[3],
        (unsigned int)raw[4],
        (unsigned int)raw[5],
        (unsigned int)raw[6],
        (unsigned int)raw[7],
        (int)error);
#endif
}

static void BoardTest_PrintEncoder(void)
{
    Serial_Printf("[enc] left=%ld right=%ld\r\n",
        (long)g_leftEncoderTotal,
        (long)g_rightEncoderTotal);
}

static void BoardTest_PrintOptionalStatus(void)
{
#if ECAR_TEST_MOTOR_ENABLE
    Serial_SendString("[pwm] armed-by-command\r\n");
#else
    Serial_SendString("[pwm] disabled\r\n");
#endif

#if ECAR_TEST_SERVO_ENABLE
    Serial_SendString("[servo] center\r\n");
#else
    Serial_SendString("[servo] disabled\r\n");
#endif

#if ECAR_TEST_IMU_ENABLE
    {
        uint8_t who = 0U;
        if (IMU_ReadWhoAmI(&who))
        {
            Serial_Printf("[imu] ok who=0x%02X\r\n", (unsigned int)who);
        }
        else
        {
            Serial_SendString("[imu] stub or fail\r\n");
        }
    }
#else
    Serial_SendString("[imu] stub\r\n");
#endif

#if ECAR_TEST_OLED_ENABLE
#if BOARD_OLED_USE_H8_SPI
    Serial_SendString("[oled] h8-spi enabled\r\n");
#else
    Serial_SendString("[oled] ok\r\n");
#endif
#else
    Serial_SendString("[oled] disabled\r\n");
#endif
}

void BoardTest_Init(void)
{
    s_ledBlinkMs = 0U;
    Motor_StopAll();

#if ECAR_TEST_SERVO_ENABLE
    Servo_SetPulseUs(0U, SERVO_MID_PULSE_US);
    Servo_SetPulseUs(1U, SERVO_MID_PULSE_US);
    Servo_SetPulseUs(2U, SERVO_MID_PULSE_US);
    Servo_SetPulseUs(3U, SERVO_MID_PULSE_US);
#else
    Servo_DisableAll();
#endif

#if ECAR_TEST_BEEP_ENABLE
    Beep_BeepMs(80U);
#endif

    Serial_SendString("[boot] mspm0 e-car board test\r\n");
    BoardTest_PrintOptionalStatus();
}

void BoardTest_Task10ms(void)
{
    uint8_t key = Key_GetNum();

    if (key != 0U)
    {
        Serial_Printf("[key-event] key=%u\r\n", (unsigned int)key);
    }

#if !ECAR_TEST_MOTOR_ENABLE
    Motor_StopAll();
#endif
}

void BoardTest_Task100ms(void)
{
    s_ledBlinkMs = (uint16_t)(s_ledBlinkMs + 100U);
    if (s_ledBlinkMs >= 500U)
    {
        s_ledBlinkMs = 0U;
        LED_User_Toggle();
    }
}

void BoardTest_Task200ms(void)
{
    BoardTest_PrintKey();
    BoardTest_PrintGray();
    BoardTest_PrintEncoder();
    BoardTest_PrintOptionalStatus();
}
