#include "JY61P.h"
#include "Serial.h"
#include <stdint.h>

/*
 * WIT-motion JY61P serial protocol:
 *   Frame: 0x55 | frame_type(1) | data[8] | checksum(1)
 *   checksum = sum of first 10 bytes, low 8 bits
 *
 *   Type 0x52: wxL wxH wyL wyH wzL wzH TL TH  (angular velocity)
 *              The last data word is temperature.
 *   Type 0x53: RollL RollH PitchL PitchH YawL YawH VL VH  (angle)
 *              The last data word is a version/reserved field, depending on
 *              the module firmware; it is not temperature.
 *
 * Wire values are signed int16_t samples.  Convert them with the WIT standard
 * ranges: angle = raw / 32768 * 180 degrees and gyro = raw / 32768 * 2000 dps.
 * The driver exposes fixed-point 0.01 degree and 0.1 dps values.
 *
 * Baud rate: 9600 (JY61P default; verify with module manual).
 * Logic level: 3.3V compatible. If module outputs 5V UART, add level shifter.
 */

#define JY61P_FRAME_LEN         11U
#define JY61P_SYNC_BYTE         0x55U
#define JY61P_FRAME_TYPE_GYRO   0x52U
#define JY61P_FRAME_TYPE_ANGLE  0x53U

typedef enum
{
    JY61P_STATE_WAIT_SYNC = 0U,
    JY61P_STATE_RECV_FRAME
} JY61P_ParseState_t;

static volatile JY61P_Data_t s_data;

static JY61P_ParseState_t s_parseState = JY61P_STATE_WAIT_SYNC;
static uint8_t s_frameBuf[JY61P_FRAME_LEN];
static uint8_t s_frameIdx = 0U;

static uint32_t s_tickMs = 0U;
static uint32_t s_lastValidMs = 0U;
static uint32_t s_lastAngleMs = 0U;
static uint32_t s_lastGyroMs = 0U;
static uint32_t s_rxOverflowBaseline = 0U;
static uint8_t s_hasValidFrame = 0U;
static uint8_t s_hasAngleFrame = 0U;
static uint8_t s_hasGyroFrame = 0U;
static uint8_t s_initDone = 0U;

static int16_t s_yawZeroOffset_x100 = 0;
static uint8_t s_yawZeroValid = 0U;

static int16_t JY61P_ReadInt16(const uint8_t *buf)
{
    return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8U));
}

static int16_t JY61P_RawToAngleX100(int16_t raw)
{
    int32_t scaled = (int32_t)raw * 18000L;

    return (int16_t)(scaled / 32768L);
}

static int16_t JY61P_RawToGyroX10(int16_t raw)
{
    int32_t scaled = (int32_t)raw * 20000L;

    return (int16_t)(scaled / 32768L);
}

static void JY61P_UpdateRelativeYaw(void)
{
    int32_t rel;

    if (!s_yawZeroValid)
    {
        s_data.relative_yaw_x100 = s_data.yaw_x100;
        return;
    }

    rel = (int32_t)s_data.yaw_x100 - (int32_t)s_yawZeroOffset_x100;
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

static void JY61P_ParseAngleFrame(const uint8_t *buf)
{
    s_data.roll_x100 = JY61P_RawToAngleX100(JY61P_ReadInt16(&buf[2]));
    s_data.pitch_x100 = JY61P_RawToAngleX100(JY61P_ReadInt16(&buf[4]));
    s_data.yaw_x100 = JY61P_RawToAngleX100(JY61P_ReadInt16(&buf[6]));

    s_data.angle_frame_count++;
    JY61P_UpdateRelativeYaw();
}

static void JY61P_ParseGyroFrame(const uint8_t *buf)
{
    s_data.gyro_x_dps_x10 = JY61P_RawToGyroX10(JY61P_ReadInt16(&buf[2]));
    s_data.gyro_y_dps_x10 = JY61P_RawToGyroX10(JY61P_ReadInt16(&buf[4]));
    s_data.gyro_z_dps_x10 = JY61P_RawToGyroX10(JY61P_ReadInt16(&buf[6]));

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

static void JY61P_ResetParser(void)
{
    s_frameIdx = 0U;
    s_parseState = JY61P_STATE_WAIT_SYNC;
}

static void JY61P_RecoverAfterInvalidFrame(void)
{
    uint8_t start;

    for (start = 1U; start < JY61P_FRAME_LEN; start++)
    {
        if (s_frameBuf[start] == JY61P_SYNC_BYTE)
        {
            uint8_t dst;
            uint8_t remaining = (uint8_t)(JY61P_FRAME_LEN - start);

            for (dst = 0U; dst < remaining; dst++)
            {
                s_frameBuf[dst] = s_frameBuf[(uint8_t)(start + dst)];
            }
            s_frameIdx = remaining;
            s_parseState = JY61P_STATE_RECV_FRAME;
            return;
        }
    }

    JY61P_ResetParser();
}

static void JY61P_ProcessValidFrame(void)
{
    uint8_t frameType = s_frameBuf[1];

    s_hasValidFrame = 1U;
    s_lastValidMs = s_tickMs;
    s_data.last_valid_frame_ms = s_tickMs;

    if (frameType == JY61P_FRAME_TYPE_ANGLE)
    {
        s_hasAngleFrame = 1U;
        s_lastAngleMs = s_tickMs;
        s_data.last_angle_frame_ms = s_tickMs;
        JY61P_ParseAngleFrame(s_frameBuf);
    }
    else if (frameType == JY61P_FRAME_TYPE_GYRO)
    {
        s_hasGyroFrame = 1U;
        s_lastGyroMs = s_tickMs;
        s_data.last_gyro_frame_ms = s_tickMs;
        JY61P_ParseGyroFrame(s_frameBuf);
    }
    else
    {
        s_data.unsupported_frame_count++;
    }
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
                s_parseState = JY61P_STATE_RECV_FRAME;
            }
            else
            {
                s_data.sync_error_count++;
            }
            break;

        case JY61P_STATE_RECV_FRAME:
            if ((s_frameIdx == 0U) || (s_frameIdx >= JY61P_FRAME_LEN))
            {
                s_data.sync_error_count++;
                JY61P_ResetParser();
                if (byte == JY61P_SYNC_BYTE)
                {
                    s_frameBuf[0] = byte;
                    s_frameIdx = 1U;
                    s_parseState = JY61P_STATE_RECV_FRAME;
                }
                break;
            }

            s_frameBuf[s_frameIdx++] = byte;

            if (s_frameIdx == JY61P_FRAME_LEN)
            {
                uint8_t calcSum = JY61P_ComputeChecksum(s_frameBuf);

                if (calcSum == s_frameBuf[JY61P_FRAME_LEN - 1U])
                {
                    JY61P_ProcessValidFrame();
                    JY61P_ResetParser();
                }
                else
                {
                    s_data.checksum_error_count++;
                    JY61P_RecoverAfterInvalidFrame();
                }
            }
            break;

        default:
            s_data.sync_error_count++;
            JY61P_ResetParser();
            break;
    }
}

static void JY61P_UpdateFreshness(void)
{
    s_data.link_age_ms = s_hasValidFrame
        ? (uint32_t)(s_tickMs - s_lastValidMs) : JY61P_AGE_UNKNOWN_MS;
    s_data.angle_age_ms = s_hasAngleFrame
        ? (uint32_t)(s_tickMs - s_lastAngleMs) : JY61P_AGE_UNKNOWN_MS;
    s_data.gyro_age_ms = s_hasGyroFrame
        ? (uint32_t)(s_tickMs - s_lastGyroMs) : JY61P_AGE_UNKNOWN_MS;

    s_data.online = (uint8_t)((s_hasValidFrame != 0U) &&
        (s_data.link_age_ms < JY61P_LINK_TIMEOUT_MS));
    s_data.angle_valid = (uint8_t)((s_hasAngleFrame != 0U) &&
        (s_data.angle_age_ms < JY61P_ANGLE_TIMEOUT_MS));
    s_data.gyro_valid = (uint8_t)((s_hasGyroFrame != 0U) &&
        (s_data.gyro_age_ms < JY61P_GYRO_TIMEOUT_MS));
}

void JY61P_Init(void)
{
    uint8_t i;

    s_initDone = 0U;

    for (i = 0U; i < JY61P_FRAME_LEN; i++)
    {
        s_frameBuf[i] = 0U;
    }
    JY61P_ResetParser();

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
    s_data.unsupported_frame_count = 0U;
    s_data.rx_overflow_count = 0U;
    s_data.last_valid_frame_ms = 0U;
    s_data.last_angle_frame_ms = 0U;
    s_data.last_gyro_frame_ms = 0U;
    s_data.link_age_ms = JY61P_AGE_UNKNOWN_MS;
    s_data.angle_age_ms = JY61P_AGE_UNKNOWN_MS;
    s_data.gyro_age_ms = JY61P_AGE_UNKNOWN_MS;
    s_data.angle_valid = 0U;
    s_data.gyro_valid = 0U;
    s_data.online = 0U;

    s_tickMs = 0U;
    s_lastValidMs = 0U;
    s_lastAngleMs = 0U;
    s_lastGyroMs = 0U;
    s_hasValidFrame = 0U;
    s_hasAngleFrame = 0U;
    s_hasGyroFrame = 0U;
    s_yawZeroOffset_x100 = 0;
    s_yawZeroValid = 0U;

    Serial_Init();
    s_rxOverflowBaseline = Serial_GetRxOverflowCount();
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
    s_data.rx_overflow_count =
        (uint32_t)(Serial_GetRxOverflowCount() - s_rxOverflowBaseline);

    while (Serial_ReadByte(&byte))
    {
        JY61P_ProcessByte(byte);
    }

    JY61P_UpdateFreshness();
}

uint8_t JY61P_ResetRelativeYaw(void)
{
    if ((s_data.online == 0U) ||
        (s_data.angle_valid == 0U) ||
        (s_hasAngleFrame == 0U) ||
        ((uint32_t)(s_tickMs - s_lastAngleMs) >= JY61P_ANGLE_TIMEOUT_MS))
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
    s_data.unsupported_frame_count = 0U;
    s_rxOverflowBaseline = Serial_GetRxOverflowCount();
    s_data.rx_overflow_count = 0U;
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
