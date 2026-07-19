#include "app_board_test.h"
#include "app_config.h"
#include "Board_Config.h"
#include "DebugSerial.h"
#include "IMU.h"
#include "Key.h"
#include "Motor.h"
#include <stdint.h>

static uint8_t s_printPaused = 0U;

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
        IMU_ReadWhoAmI(&who);

        DebugSerial_Printf("[imu,init,ok,addr=0x%02X,who=0x%02X,ready=%u,healthy=%u]\r\n",
            (unsigned int)IMU_GetAddr(),
            (unsigned int)who,
            (unsigned int)IMU_IsReady(),
            (unsigned int)IMU_IsHealthy());

        DebugSerial_SendString("[imu,calib,start,keep_still]\r\n");
        IMU_CalibrateGyroZ(300);
        DebugSerial_Printf("[imu,calib,done,offset=%d]\r\n", (int)IMU_GetGyroZOffset());
        IMU_ResetYaw();
        DebugSerial_SendString("[imu,yaw,reset]\r\n");
    }
    else
    {
        DebugSerial_Printf("[imu,init,fail,addr=0x%02X,addr_valid=%u,stage=%u,name=%s]\r\n",
            (unsigned int)IMU_GetAddr(),
            (unsigned int)IMU_IsAddrValid(),
            (unsigned int)IMU_GetLastErrorStage(),
            IMU_GetErrorStageName(IMU_GetLastErrorStage()));

        DebugSerial_Printf("[imu,ack,68=%u,69=%u]\r\n",
            (unsigned int)IMU_ProbeAddressAck(0x68U),
            (unsigned int)IMU_ProbeAddressAck(0x69U));
        DebugSerial_Printf("[imu,bus,sda=%u,scl=%u]\r\n",
            (unsigned int)IMU_GetSdaLevel(),
            (unsigned int)IMU_GetSclLevel());
    }
}

void BoardTest_Task10ms(void)
{
    Motor_StopAll();

    if (IMU_IsReady())
    {
        IMU_UpdateYaw(10U);
    }

    {
        uint8_t key = Key_GetNum();

        if (key != 0U)
        {
            switch (key)
            {
            case 1U:
                DebugSerial_SendString("[imu,calib,start,keep_still]\r\n");
                IMU_CalibrateGyroZ(300);
                DebugSerial_Printf("[imu,calib,done,offset=%d]\r\n", (int)IMU_GetGyroZOffset());
                IMU_ResetYaw();
                DebugSerial_SendString("[imu,yaw,reset]\r\n");
                break;
            case 2U:
                IMU_ResetYaw();
                DebugSerial_SendString("[imu,yaw,reset]\r\n");
                break;
            case 3U:
                s_printPaused = !s_printPaused;
                if (s_printPaused)
                    DebugSerial_SendString("[imu,print,paused]\r\n");
                else
                    DebugSerial_SendString("[imu,print,resumed]\r\n");
                break;
            case 4U:
                IMU_ResetYaw();
                DebugSerial_SendString("[imu,test,reset]\r\n");
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
    if (s_printPaused) return;

    if (IMU_IsReady())
    {
        int16_t gx, gy, gz;
        int16_t rawZ_x10, dps_x10;
        int32_t yaw_x10;

        IMU_ReadGyroRaw(&gx, &gy, &gz);
        IMU_GetGyroRawZ_x10(&rawZ_x10, &dps_x10);
        yaw_x10 = IMU_GetYawDeg_x10();

        DebugSerial_Printf("[imu,gyro,gx=%d,gy=%d,gz=%d,offset=%d,z_dps_x10=%d]\r\n",
            (int)gx, (int)gy, (int)gz,
            (int)IMU_GetGyroZOffset(), (int)dps_x10);
        DebugSerial_Printf("[imu,yaw_x10=%ld,ready=%u,healthy=%u,stage=%u]\r\n",
            (long)yaw_x10,
            (unsigned int)IMU_IsReady(),
            (unsigned int)IMU_IsHealthy(),
            (unsigned int)IMU_GetLastErrorStage());
    }
    else
    {
        DebugSerial_Printf("[imu,read,fail,stage=%u,name=%s,sda=%u,scl=%u]\r\n",
            (unsigned int)IMU_GetLastErrorStage(),
            IMU_GetErrorStageName(IMU_GetLastErrorStage()),
            (unsigned int)IMU_GetSdaLevel(),
            (unsigned int)IMU_GetSclLevel());
    }
}
