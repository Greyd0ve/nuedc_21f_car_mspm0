#include "app_board_test.h"
#include "app_config.h"
#include "Board_Config.h"
#include "BeepLed.h"
#include "Encoder.h"
#include "Grayscale.h"
#include "IMU.h"
#include "Key.h"
#include "Motor.h"
#include "OLED.h"
#include "Serial.h"
#include "Servo.h"
#include "app_control.h"
#include <stdint.h>

extern volatile int32_t g_leftEncoderTotal;
extern volatile int32_t g_rightEncoderTotal;
extern volatile int16_t g_leftEncoderDelta;
extern volatile int16_t g_rightEncoderDelta;
extern volatile int16_t g_rightLastNonZeroDelta;
extern volatile uint32_t g_rightNonZeroDeltaCount;
extern volatile uint32_t g_rightLimitDeltaCount;
extern volatile float g_leftSpeed;
extern volatile float g_rightSpeed;
extern volatile float g_forwardSpeed;
extern volatile float g_turnSpeed;
extern volatile float g_targetForwardSpeed;
extern volatile float g_targetTurnSpeed;
extern volatile float g_speedPwm;
extern volatile float g_diffPwm;
extern volatile int16_t g_leftPwm;
extern volatile int16_t g_rightPwm;
extern volatile uint8_t g_carEnable;


static uint16_t s_ledBlinkMs = 0U;

static int32_t BoardTest_FloatToX10(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value * 10.0f + 0.5f);
    }
    return (int32_t)(value * 10.0f - 0.5f);
}

static void BoardTest_PrintSpeedLoop(void)
{
    Serial_Printf("[speed] target=%ld turn=%ld left=%ld right=%ld forward=%ld turnNow=%ld pwmL=%d pwmR=%d spwm=%ld dpwm=%ld\r\n",
        (long)BoardTest_FloatToX10(g_targetForwardSpeed),
        (long)BoardTest_FloatToX10(g_targetTurnSpeed),
        (long)BoardTest_FloatToX10(g_leftSpeed),
        (long)BoardTest_FloatToX10(g_rightSpeed),
        (long)BoardTest_FloatToX10(g_forwardSpeed),
        (long)BoardTest_FloatToX10(g_turnSpeed),
        (int)g_leftPwm,
        (int)g_rightPwm,
        (long)BoardTest_FloatToX10(g_speedPwm),
        (long)BoardTest_FloatToX10(g_diffPwm));
}

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
    Serial_Printf("[enc] left=%ld right=%ld ld=%d rd=%d LA=%u LB=%u RA=%u RB=%u risr=%lu rign=%lu",
        (long)g_leftEncoderTotal,
        (long)g_rightEncoderTotal,
        (int)g_leftEncoderDelta,
        (int)g_rightEncoderDelta,
        (unsigned int)BoardTest_ReadKeyLevel(ENC_L_A_PORT, ENC_L_A_PIN),
        (unsigned int)BoardTest_ReadKeyLevel(ENC_L_B_PORT, ENC_L_B_PIN),
        (unsigned int)BoardTest_ReadKeyLevel(ENC_R_A_PORT, ENC_R_A_PIN),
        (unsigned int)BoardTest_ReadKeyLevel(ENC_R_B_PORT, ENC_R_B_PIN),
        (unsigned long)Encoder_GetRightIsrCount(),
        (unsigned long)Encoder_GetRightSameAIgnored());
    Serial_Printf(" rstat=%lu rraw=%ld rlim=%lu rget=%lu rnz=%lu rmax=%ld rnzd=%lu rlmd=%lu rlast=%d\r\n",
        (unsigned long)Encoder_GetRightStatusCount(),
        (long)Encoder_GetRightLastRawDeltaBeforeLimit(),
        (unsigned long)Encoder_GetRightLimitHitCount(),
        (unsigned long)Encoder_GetRightGetDeltaCount(),
        (unsigned long)Encoder_GetRightNonZeroGetCount(),
        (long)Encoder_GetRightMaxRawDelta(),
        (unsigned long)g_rightNonZeroDeltaCount,
        (unsigned long)g_rightLimitDeltaCount,
        (int)g_rightLastNonZeroDelta);
}

static void BoardTest_PrintIMU(void)
{
#if ECAR_TEST_IMU_ENABLE
    uint8_t who = 0U;
    uint8_t addr;

    addr = IMU_GetAddr();

    if (!IMU_ReadWhoAmI(&who))
    {
        uint8_t stage = IMU_GetLastErrorStage();
        uint8_t ack68 = IMU_ProbeAddressAck(0x68U);
        uint8_t ack69 = IMU_ProbeAddressAck(0x69U);
        uint8_t foundAddr = 0U;

        Serial_Printf("[imu-ack] 68=%u 69=%u\r\n",
                      (unsigned int)ack68, (unsigned int)ack69);
        Serial_Printf("[imu-bus] sda=%u scl=%u\r\n",
                      (unsigned int)IMU_GetSdaLevel(),
                      (unsigned int)IMU_GetSclLevel());

        if (IMU_Scan(&foundAddr))
        {
            Serial_Printf("[imu-scan] found 0x%02X\r\n", (unsigned int)foundAddr);

            {
                uint8_t retryWho = 0U;
                if (IMU_ReadWhoAmI(&retryWho))
                {
                    Serial_Printf("[imu] retry who=0x%02X addr=0x%02X ready=%u healthy=%u stage=%u name=%s\r\n",
                                  (unsigned int)retryWho, (unsigned int)foundAddr,
                                  (unsigned int)IMU_IsReady(),
                                  (unsigned int)IMU_IsHealthy(),
                                  (unsigned int)IMU_GetLastErrorStage(),
                                  IMU_GetErrorStageName(IMU_GetLastErrorStage()));
                }
                else
                {
                    uint8_t retryStage = IMU_GetLastErrorStage();
                    Serial_Printf("[imu] retry_fail addr=0x%02X stage=%u name=%s\r\n",
                                  (unsigned int)foundAddr,
                                  (unsigned int)retryStage,
                                  IMU_GetErrorStageName(retryStage));
                }
            }
        }
        else
        {
            uint8_t addrValid;

            addr = IMU_GetAddr();
            addrValid = IMU_IsAddrValid();

            if (!addrValid)
            {
                Serial_Printf("[imu] fail scan=none stage=%u name=%s\r\n",
                              (unsigned int)stage, IMU_GetErrorStageName(stage));
            }
            else
            {
                Serial_Printf("[imu] fail addr=0x%02X stage=%u name=%s\r\n",
                              (unsigned int)addr, (unsigned int)stage, IMU_GetErrorStageName(stage));
            }
        }
        return;
    }

    if (!IMU_IsReady())
    {
        uint8_t stage = IMU_GetLastErrorStage();

        if (who != MPU6050_WHO_AM_I_VAL)
        {
            Serial_Printf("[imu] who_mismatch who=0x%02X addr=0x%02X\r\n",
                          (unsigned int)who, (unsigned int)addr);
        }
        else
        {
            static uint8_t s_reinited = 0U;

            Serial_Printf("[imu] not_ready who=0x%02X addr=0x%02X healthy=0 init_stage=%u init_name=%s last_stage=%u last_name=%s\r\n",
                          (unsigned int)who, (unsigned int)addr,
                          (unsigned int)IMU_GetInitErrorStage(),
                          IMU_GetErrorStageName(IMU_GetInitErrorStage()),
                          (unsigned int)stage, IMU_GetErrorStageName(stage));

            if (!s_reinited)
            {
                s_reinited = 1U;
                IMU_Init();
                if (IMU_IsReady())
                {
                    Serial_SendString("[imu] reinit_ok\r\n");
                    IMU_CalibrateGyroZ(300);
                    IMU_ResetYaw();
                }
                else
                {
                    uint8_t riStage = IMU_GetLastErrorStage();
                    Serial_Printf("[imu] reinit_fail stage=%u name=%s\r\n",
                                  (unsigned int)riStage, IMU_GetErrorStageName(riStage));
                }
            }
        }
        return;
    }

    {
        int16_t gzRaw = 0;
        int16_t gzDps_x10 = 0;
        int32_t yaw_x10 = 0;
        uint8_t healthy;
        uint8_t gyroOk;

        gyroOk = IMU_GetGyroRawZ_x10(&gzRaw, &gzDps_x10);
        if (gyroOk)
        {
            yaw_x10 = IMU_GetYawDeg_x10();
        }
        healthy = IMU_IsHealthy();

        if (gyroOk)
        {
            int16_t gx = IMU_GetLastGyroXRaw();
            int16_t gy = IMU_GetLastGyroYRaw();
            int16_t gz = IMU_GetLastGyroZRaw();
            int16_t off = IMU_GetGyroZOffset();
            int32_t dyaw = IMU_GetLastYawDelta_x10();
            uint8_t r[5] = {0};
            uint8_t sb[6] = {0};

            Serial_Printf("[imu] ok addr=0x%02X who=0x%02X healthy=%u\r\n",
                          (unsigned int)addr, (unsigned int)who, (unsigned int)healthy);
            Serial_Printf("[imu-gyro] gx=%d gy=%d gz=%d off=%d gz_dps=%d.%d\r\n",
                          (int)gx, (int)gy, (int)gz, (int)off,
                          (int)(gzDps_x10 / 10), (int)((gzDps_x10 < 0) ? (-gzDps_x10 % 10) : (gzDps_x10 % 10)));
            Serial_Printf("[imu-yaw] yaw_raw=%d yaw=%d.%d dyaw=%d.%d\r\n",
                          (int)yaw_x10,
                          (int)(yaw_x10 / 10), (int)((yaw_x10 < 0) ? (-yaw_x10 % 10) : (yaw_x10 % 10)),
                          (int)(dyaw / 10), (int)((dyaw < 0) ? (-dyaw % 10) : (dyaw % 10)));

            IMU_DebugReadReg(0x6BU, &r[0]);
            IMU_DebugReadReg(0x6CU, &r[1]);
            IMU_DebugReadReg(0x19U, &r[2]);
            IMU_DebugReadReg(0x1AU, &r[3]);
            IMU_DebugReadReg(0x1BU, &r[4]);
            Serial_Printf("[imu-reg] pwr=0x%02X pwr2=0x%02X smpl=0x%02X cfg=0x%02X gyro_cfg=0x%02X\r\n",
                          (unsigned int)r[0], (unsigned int)r[1],
                          (unsigned int)r[2], (unsigned int)r[3], (unsigned int)r[4]);

            Serial_Printf("[imu-used] g=%02X %02X %02X %02X %02X %02X\r\n",
                          (unsigned int)IMU_GetLastGyroByte(0U),
                          (unsigned int)IMU_GetLastGyroByte(1U),
                          (unsigned int)IMU_GetLastGyroByte(2U),
                          (unsigned int)IMU_GetLastGyroByte(3U),
                          (unsigned int)IMU_GetLastGyroByte(4U),
                          (unsigned int)IMU_GetLastGyroByte(5U));

            IMU_DebugReadReg(0x43U, &sb[0]);
            IMU_DebugReadReg(0x44U, &sb[1]);
            IMU_DebugReadReg(0x45U, &sb[2]);
            IMU_DebugReadReg(0x46U, &sb[3]);
            IMU_DebugReadReg(0x47U, &sb[4]);
            IMU_DebugReadReg(0x48U, &sb[5]);
            Serial_Printf("[imu-single] g=%02X %02X %02X %02X %02X %02X\r\n",
                          (unsigned int)sb[0], (unsigned int)sb[1],
                          (unsigned int)sb[2], (unsigned int)sb[3],
                          (unsigned int)sb[4], (unsigned int)sb[5]);
        }
        else
        {
            Serial_Printf("[imu] fail addr=0x%02X who=0x%02X gyro_read_fail healthy=%u\r\n",
                          (unsigned int)addr, (unsigned int)who, (unsigned int)healthy);
        }
    }
#else
    Serial_SendString("[imu] stub\r\n");
#endif
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

    BoardTest_PrintIMU();

#if ECAR_TEST_OLED_ENABLE
#if BOARD_OLED_USE_H8_SPI
    Serial_SendString("[oled] h8-spi enabled\r\n");
#elif BOARD_OLED_USE_H8_I2C
    Serial_SendString("[oled] h8-i2c pb9/pb8 enabled\r\n");
#else
    Serial_SendString("[oled] i2c shared pa28/pa31 enabled\r\n");
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

#if ECAR_TEST_IMU_ENABLE
    if (IMU_IsReady())
    {
        IMU_CalibrateGyroZ(300);
        Serial_Printf("[imu] calib offset=%d\r\n", (int)IMU_GetGyroZOffset());
        IMU_ResetYaw();
    }
#endif

    BoardTest_PrintOptionalStatus();
}

void BoardTest_Task10ms(void)
{
    uint8_t key = Key_GetNum();

    if (key != 0U)
    {
        Serial_Printf("[key-event] key=%u\r\n", (unsigned int)key);
    }

#if ECAR_TEST_IMU_ENABLE
    if (IMU_IsReady())
    {
        IMU_UpdateYaw(10);
    }
#endif

#if ECAR_TEST_MOTOR_ENABLE
    if (g_carEnable)
    {
        App_Control_ApplyMotorOutput();
    }
#else
    Motor_StopAll();
#endif
}
void BoardTest_Task100ms(void)
{
    /* Keep PB04 / LED_USER low by default. */
    LED_User_Off();
}

void BoardTest_Task200ms(void)
{
    static uint8_t div = 0U;

    div++;
    if (div < 5U)
    {
        return;
    }
    div = 0U;

    //BoardTest_PrintKey();
    //BoardTest_PrintGray();
    //BoardTest_PrintEncoder();
		BoardTest_PrintSpeedLoop();
    BoardTest_PrintIMU();
  
}
