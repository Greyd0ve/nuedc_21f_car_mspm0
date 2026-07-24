#include "JY61P.h"
#include "Serial.h"
#include <stdint.h>

/*
 * WIT-motion JY61P serial protocol:
 *   Frame: 0x55 | frame_type(1) | data[8] | checksum(1)
 *   checksum = sum of first 10 bytes, low 8 bits
 *
 *   Type 0x52: wxL wxH wyL wyH wzL wzH TL TH  (angular velocity)
 *   Type 0x53: RollL RollH PitchL PitchH YawL YawH TL TH  (angle)
 *
 * Angles in 0.01 degree, angular velocity in 0.1 dps.
 * Temperature byte pair (TL,TH) is ignored.
 *
 * Baud rate: 9600 (JY61P default; verify with module manual).
 * Logic level: 3.3V compatible. If module outputs 5V UART, add level shifter.
 */

#define JY61P_FRAME_LEN         11U
#define JY61P_SYNC_BYTE         0x55U
#define JY61P_FRAME_TYPE_GYRO   0x52U
#define JY61P_FRAME_TYPE_ANGLE  0x53U
#define JY61P_TIMEOUT_MS        500U

typedef enum
{
    JY61P_STATE_WAIT_SYNC = 0U,
    JY61P_STATE_WAIT_TYPE,
    JY61P_STATE_RECV_DATA
} JY61P_ParseState_t;

static volatile JY61P_Data_t s_data;

static JY61P_ParseState_t s_parseState = JY61P_STATE_WAIT_SYNC;
static uint8_t s_frameBuf[JY61P_FRAME_LEN];
static uint8_t s_frameIdx = 0U;
static uint8_t s_frameType = 0U;

static uint32_t s_tickMs = 0U;
static uint32_t s_lastValidMs = 0U;
static uint8_t s_initDone = 0U;

static int16_t s_yawZeroOffset_x100 = 0;
static uint8_t s_yawZeroValid = 0U;


static void JY61P_ParseAngleFrame(const uint8_t *buf)
{
    int32_t raw;

    raw = (int32_t)((int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8U)));
    s_data.roll_x100 = (int16_t)raw;

    raw = (int32_t)((int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8U)));
    s_data.pitch_x100 = (int16_t)raw;

    raw = (int32_t)((int16_t)((uint16_t)buf[6] | ((uint16_t)buf[7] << 8U)));
    s_data.yaw_x100 = (int16_t)raw;

    s_data.angle_valid = 1U;
    s_data.angle_frame_count++;

    if (s_yawZeroValid)
    {
        int32_t rel = (int32_t)s_data.yaw_x100 - (int32_t)s_yawZeroOffset_x100;

        if (rel > 18000L)
        {
            rel -= 36000L;
        }
        else if (rel < -18000L)
        {
            rel += 36000L;
        }

        s_data.relative_yaw_x100 = (int16_t)rel;
    }
    else
    {
        s_data.relative_yaw_x100 = s_data.yaw_x100;
    }
}

static void JY61P_ParseGyroFrame(const uint8_t *buf)
{
    int32_t raw;

    raw = (int32_t)((int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8U)));
    s_data.gyro_x_dps_x10 = (int16_t)raw;

    raw = (int32_t)((int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8U)));
    s_data.gyro_y_dps_x10 = (int16_t)raw;

    raw = (int32_t)((int16_t)((uint16_t)buf[6] | ((uint16_t)buf[7] << 8U)));
    s_data.gyro_z_dps_x10 = (int16_t)raw;

    s_data.gyro_valid = 1U;
    s_data.gyro_frame_count++;
}

static uint8_t JY61P_ComputeChecksum(const uint8_t *buf)
{
    uint8_t i;
    uint32_t sum = 0U;

    for (i = 0U; i < (JY61P_FRAME_LEN - 1U); i++)
    {
        sum += buf[i];
    }
    return (uint8_t)(sum & 0xFFU);
}

static void JY61P_ProcessByte(uint8_t byte)
{
    switch (s_parseState)
    {
        case JY61P_STATE_WAIT_SYNC:
            if (byte == JY61P_SYNC_BYTE)
            {
                s_frameBuf[0] = byte;
                s_frameIdx = 1U;
                s_parseState = JY61P_STATE_WAIT_TYPE;
            }
            break;

        case JY61P_STATE_WAIT_TYPE:
            if (byte == JY61P_FRAME_TYPE_GYRO || byte == JY61P_FRAME_TYPE_ANGLE)
            {
                s_frameBuf[1] = byte;
                s_frameType = byte;
                s_frameIdx = 2U;
                s_parseState = JY61P_STATE_RECV_DATA;
            }
            else if (byte == JY61P_SYNC_BYTE)
            {
                s_frameBuf[0] = byte;
                s_frameIdx = 1U;
            }
            else
            {
                s_parseState = JY61P_STATE_WAIT_SYNC;
                s_data.sync_error_count++;
            }
            break;

        case JY61P_STATE_RECV_DATA:
            s_frameBuf[s_frameIdx] = byte;
            s_frameIdx++;

            if (s_frameIdx >= JY61P_FRAME_LEN)
            {
                uint8_t calcSum = JY61P_ComputeChecksum(s_frameBuf);

                s_parseState = JY61P_STATE_WAIT_SYNC;

                if (calcSum == s_frameBuf[JY61P_FRAME_LEN - 1U])
                {
                    s_lastValidMs = s_tickMs;
                    s_data.last_valid_frame_ms = s_tickMs;

                    if (s_frameType == JY61P_FRAME_TYPE_ANGLE)
                    {
                        JY61P_ParseAngleFrame(s_frameBuf);
                    }
                    else
                    {
                        JY61P_ParseGyroFrame(s_frameBuf);
                    }
                }
                else
                {
                    s_data.checksum_error_count++;
                }
            }
            break;

        default:
            s_parseState = JY61P_STATE_WAIT_SYNC;
            break;
    }
}

void JY61P_Init(void)
{
    uint8_t i;

    for (i = 0U; i < JY61P_FRAME_LEN; i++)
    {
        s_frameBuf[i] = 0U;
    }
    s_frameIdx = 0U;
    s_frameType = 0U;
    s_parseState = JY61P_STATE_WAIT_SYNC;

    s_data.roll_x100 = 0;
    s_data.pitch_x100 = 0;
    s_data.yaw_x100 = 0;
    s_data.relative_yaw_x100 = 0;
    s_data.gyro_x_dps_x10 = 0;
    s_data.gyro_y_dps_x10 = 0;
    s_data.gyro_z_dps_x10 = 0;
    s_data.angle_frame_count = 0U;
    s_data.gyro_frame_count = 0U;
    s_data.checksum_error_count = 0U;
    s_data.sync_error_count = 0U;
    s_data.rx_overflow_count = 0U;
    s_data.last_valid_frame_ms = 0U;
    s_data.angle_valid = 0U;
    s_data.gyro_valid = 0U;
    s_data.online = 0U;

    s_tickMs = 0U;
    s_lastValidMs = 0U;
    s_yawZeroOffset_x100 = 0;
    s_yawZeroValid = 0U;

    Serial_Init();
    s_initDone = 1U;
}

void JY61P_Task10ms(void)
{
    uint8_t byte;

    if (!s_initDone)
    {
        return;
    }

    s_tickMs += 10U;
    s_data.rx_overflow_count = Serial_GetRxOverflowCount();

    while (Serial_ReadByte(&byte))
    {
        JY61P_ProcessByte(byte);
    }

    if ((s_tickMs - s_lastValidMs) >= JY61P_TIMEOUT_MS)
    {
        s_data.online = 0U;
    }
    else
    {
        s_data.online = 1U;
    }
}

uint8_t JY61P_ResetRelativeYaw(void)
{
    if (!s_data.angle_valid)
    {
        return 0U;
    }

    s_yawZeroOffset_x100 = s_data.yaw_x100;
    s_yawZeroValid = 1U;
    s_data.relative_yaw_x100 = 0;
    return 1U;
}

void JY61P_ClearStatistics(void)
{
    s_data.angle_frame_count = 0U;
    s_data.gyro_frame_count = 0U;
    s_data.checksum_error_count = 0U;
    s_data.sync_error_count = 0U;
    s_data.rx_overflow_count = 0U;
    s_data.last_valid_frame_ms = s_tickMs;
}

uint8_t JY61P_GetData(JY61P_Data_t *data)
{
    if (data == 0)
    {
        return 0U;
    }
    *data = s_data;
    return 1U;
}

uint8_t JY61P_IsOnline(void)
{
    return s_data.online;
}
