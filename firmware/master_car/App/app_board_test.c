#include "app_board_test.h"
#include "app_config.h"
#include "Board_Config.h"
#include "DebugSerial.h"
#include "IMU.h"
#include "Key.h"
#include "Motor.h"
#include <stdint.h>

static uint8_t s_printPaused = 0U;
static uint32_t s_lastErrorMs = 0U;
static uint8_t s_lastSda = 1U;
static uint8_t s_lastScl = 1U;
static uint32_t s_reinitTimer = 0U;

void BoardTest_Init(void)
{
    Motor_StopAll();

    DebugSerial_SendString("[board-test,start]\r\n");
    DebugSerial_SendString("[board-test,mode=imu]\r\n");
    DebugSerial_SendString("[board-test,motor=disabled]\r\n");
    DebugSerial_SendString("[imu,test,start]\r\n");

    if (IMU_IsReady())
    {
        uint8_t who = 0U;
        uint8_t whoOk = IMU_ReadWhoAmI(&who);

        DebugSerial_Printf("[imu,init,ok,addr=0x%02X,who=0x%02X,ready=%u,healthy=%u]\r\n",
            (unsigned int)IMU_GetAddr(),
            (unsigned int)who,
            (unsigned int)IMU_IsReady(),
            (unsigned int)IMU_IsHealthy());
        DebugSerial_Printf("[imu,who,ok=%u,value=0x%02X]\r\n", (unsigned int)whoOk, (unsigned int)who);

        DebugSerial_SendString("[imu,calib,start,keep_still]\r\n");
        if (IMU_CalibrateGyroZ(300))
        {
            DebugSerial_Printf("[imu,calib,done,offset=%d]\r\n", (int)IMU_GetGyroZOffset());
            DebugSerial_SendString("[imu,yaw,reset]\r\n");
        }
        else
        {
            DebugSerial_SendString("[imu,calib,fail]\r\n");
        }
    }
    else
    {
        uint8_t sda, scl;
        IMU_GetBusLevels(&sda, &scl);
        DebugSerial_Printf("[imu,init,fail,addr=0x%02X,addr_valid=%u,stage=%u,name=%s]\r\n",
            (unsigned int)IMU_GetAddr(),
            (unsigned int)IMU_IsAddrValid(),
            (unsigned int)IMU_GetLastErrorStage(),
            IMU_GetErrorStageName(IMU_GetLastErrorStage()));
        DebugSerial_Printf("[imu,bus,sda=%u,scl=%u]\r\n", (unsigned int)sda, (unsigned int)scl);
    }
}

static void BoardTest_ImuReinit(void)
{
    IMU_Init();
    if (IMU_IsReady())
    {
        DebugSerial_SendString("[imu,reinit,ok]\r\n");
        if (IMU_CalibrateGyroZ(300))
        {
            DebugSerial_Printf("[imu,calib,done,offset=%d]\r\n", (int)IMU_GetGyroZOffset());
        }
        else
        {
            DebugSerial_SendString("[imu,calib,fail]\r\n");
        }
    }
    else
    {
        uint8_t sda, scl;
        IMU_GetBusLevels(&sda, &scl);
        DebugSerial_Printf("[imu,reinit,fail,stage=%u,name=%s,sda=%u,scl=%u]\r\n",
            (unsigned int)IMU_GetLastErrorStage(),
            IMU_GetErrorStageName(IMU_GetLastErrorStage()),
            (unsigned int)sda, (unsigned int)scl);
    }
}

void BoardTest_Task10ms(void)
{
    Motor_StopAll();
    s_reinitTimer += 10U;

    if (IMU_IsReady())
    {
        IMU_UpdateYaw(10U);
        s_reinitTimer = 0U;
    }
    else if (s_reinitTimer >= 1000U)
    {
        s_reinitTimer = 0U;
        BoardTest_ImuReinit();
    }

    {
        uint8_t key = Key_GetNum();

        if (key != 0U)
        {
            switch (key)
            {
            case 1U:
                if (!IMU_IsReady())
                {
                    DebugSerial_SendString("[imu,calib,rejected,not_ready]\r\n");
                }
                else
                {
                    DebugSerial_SendString("[imu,calib,start,keep_still]\r\n");
                    if (IMU_CalibrateGyroZ(300))
                    {
                        DebugSerial_Printf("[imu,calib,done,offset=%d]\r\n", (int)IMU_GetGyroZOffset());
                        DebugSerial_SendString("[imu,yaw,reset]\r\n");
                    }
                    else
                    {
                        DebugSerial_SendString("[imu,calib,fail]\r\n");
                    }
                }
                break;
            case 2U:
                if (!IMU_IsReady())
                {
                    DebugSerial_SendString("[imu,yaw,rejected,not_ready]\r\n");
                }
                else
                {
                    IMU_ResetYaw();
                    DebugSerial_SendString("[imu,yaw,reset]\r\n");
                }
                break;
            case 3U:
                s_printPaused = !s_printPaused;
                if (s_printPaused)
                    DebugSerial_SendString("[imu,print,paused]\r\n");
                else
                    DebugSerial_SendString("[imu,print,resumed]\r\n");
                break;
            case 4U:
                Motor_StopAll();
                IMU_Init();
                if (IMU_IsReady())
                    DebugSerial_SendString("[imu,test,reset,ok]\r\n");
                else
                {
                    uint8_t sda, scl;
                    IMU_GetBusLevels(&sda, &scl);
                    DebugSerial_Printf("[imu,test,reset,fail,stage=%u,name=%s,sda=%u,scl=%u]\r\n",
                        (unsigned int)IMU_GetLastErrorStage(),
                        IMU_GetErrorStageName(IMU_GetLastErrorStage()),
                        (unsigned int)sda, (unsigned int)scl);
                }
                break;
            default:
                break;
            }
        }
    }
}

void BoardTest_Task100ms(void)
{
}

void BoardTest_Task200ms(void)
{
    uint8_t changed = 0U;
    uint8_t sda, scl;
    uint32_t now = s_lastErrorMs + 200U;

    if (s_printPaused) return;

    IMU_GetBusLevels(&sda, &scl);
    if (sda != s_lastSda || scl != s_lastScl) { changed = 1U; s_lastSda = sda; s_lastScl = scl; }

    if (IMU_IsReady())
    {
        int16_t gx = 0, gy = 0, gz = 0;
        int32_t yaw_x10 = 0;

        if (IMU_ReadGyroRaw(&gx, &gy, &gz))
        {
            int16_t offset = IMU_GetGyroZOffset();
            int16_t dps_x10 = (int16_t)((int32_t)(gz - offset) * 2500L / 32768L);
            yaw_x10 = IMU_GetYawDeg_x10();

            DebugSerial_Printf("[imu,gyro,gx=%d,gy=%d,gz=%d,offset=%d,z_dps_x10=%d]\r\n",
                (int)gx, (int)gy, (int)gz, (int)offset, (int)dps_x10);
            DebugSerial_Printf("[imu,yaw_x10=%ld,ready=%u,healthy=%u,stage=%u]\r\n",
                (long)yaw_x10,
                (unsigned int)IMU_IsReady(),
                (unsigned int)IMU_IsHealthy(),
                (unsigned int)IMU_GetLastErrorStage());
        }
        else
        {
            if (changed || now >= 1000U)
            {
                DebugSerial_Printf("[imu,read,fail,stage=%u,name=%s,sda=%u,scl=%u]\r\n",
                    (unsigned int)IMU_GetLastErrorStage(),
                    IMU_GetErrorStageName(IMU_GetLastErrorStage()),
                    (unsigned int)sda, (unsigned int)scl);
                s_lastErrorMs = now;
            }
        }
    }
    else
    {
        if (changed || now >= 1000U)
        {
            DebugSerial_Printf("[imu,not_ready,stage=%u,name=%s,sda=%u,scl=%u]\r\n",
                (unsigned int)IMU_GetLastErrorStage(),
                IMU_GetErrorStageName(IMU_GetLastErrorStage()),
                (unsigned int)sda, (unsigned int)scl);
            s_lastErrorMs = now;
        }
    }
}
