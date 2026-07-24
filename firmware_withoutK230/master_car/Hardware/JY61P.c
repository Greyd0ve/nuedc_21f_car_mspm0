#include "JY61P.h"
#include "Serial.h"
#include "Timer.h"
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
/*
 * All freshness windows are far below 2^31 ms.  A larger wrap-safe elapsed
 * value therefore means that the timestamp is in the future or corrupted.
 */
#define JY61P_MAX_VALID_AGE_MS  ((uint32_t)0x7FFFFFFFUL)

typedef enum
{
    JY61P_STATE_WAIT_SYNC = 0U,
    JY61P_STATE_RECV_FRAME
} JY61P_ParseState_t;

static JY61P_Data_t s_data;

static JY61P_ParseState_t s_parseState = JY61P_STATE_WAIT_SYNC;
static uint8_t s_frameBuf[JY61P_FRAME_LEN];
static uint8_t s_frameIdx = 0U;

static uint32_t s_rxOverflowBaseline = 0U;
static uint8_t s_hasValidFrame = 0U;
static uint8_t s_hasAngleFrame = 0U;
static uint8_t s_hasGyroFrame = 0U;
static uint8_t s_linkTimebaseFault = 0U;
static uint8_t s_angleTimebaseFault = 0U;
static uint8_t s_gyroTimebaseFault = 0U;
static uint8_t s_discardingUnsynced = 0U;
static uint8_t s_initDone = 0U;

static int32_t s_yawZeroOffsetX100 = 0;
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

static int32_t JY61P_NormalizeYawX100(int32_t value)
{
    while (value > 18000L)
    {
        value -= 36000L;
    }
    while (value < -18000L)
    {
        value += 36000L;
    }

    return value;
}

static void JY61P_ParseAngleFrame(const uint8_t *buf)
{
    int16_t roll = JY61P_RawToAngleX100(JY61P_ReadInt16(&buf[2]));
    int16_t pitch = JY61P_RawToAngleX100(JY61P_ReadInt16(&buf[4]));
    int16_t yaw = JY61P_RawToAngleX100(JY61P_ReadInt16(&buf[6]));
    int32_t relative;

    if (s_yawZeroValid)
    {
        int32_t offset = s_yawZeroOffsetX100;

        if ((offset < -18000L) || (offset > 18000L))
        {
            s_data.yaw_state_fault_count++;
            offset %= 36000L;
            offset = JY61P_NormalizeYawX100(offset);
        }
        relative = JY61P_NormalizeYawX100((int32_t)yaw - offset);
    }
    else
    {
        relative = (int32_t)yaw;
    }

    if ((relative < -18000L) || (relative > 18000L))
    {
        s_data.yaw_state_fault_count++;
        relative = JY61P_NormalizeYawX100(relative);
    }

    s_data.roll_x100 = roll;
    s_data.pitch_x100 = pitch;
    s_data.yaw_x100 = yaw;
    s_data.relative_yaw_x100 = (int16_t)relative;
    s_data.angle_frame_count++;
}

static void JY61P_ParseGyroFrame(const uint8_t *buf)
{
    int16_t gyroX = JY61P_RawToGyroX10(JY61P_ReadInt16(&buf[2]));
    int16_t gyroY = JY61P_RawToGyroX10(JY61P_ReadInt16(&buf[4]));
    int16_t gyroZ = JY61P_RawToGyroX10(JY61P_ReadInt16(&buf[6]));

    s_data.gyro_x_dps_x10 = gyroX;
    s_data.gyro_y_dps_x10 = gyroY;
    s_data.gyro_z_dps_x10 = gyroZ;
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
    s_discardingUnsynced = 0U;
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
            s_discardingUnsynced = 0U;
            return;
        }
    }

    JY61P_ResetParser();
}

static void JY61P_ProcessValidFrame(void)
{
    uint8_t frameType = s_frameBuf[1];
    uint32_t now = Timer_GetMillis();

    s_hasValidFrame = 1U;
    s_linkTimebaseFault = 0U;
    s_data.last_valid_frame_ms = now;

    if (frameType == JY61P_FRAME_TYPE_ANGLE)
    {
        s_hasAngleFrame = 1U;
        s_angleTimebaseFault = 0U;
        s_data.last_angle_frame_ms = now;
        JY61P_ParseAngleFrame(s_frameBuf);
    }
    else if (frameType == JY61P_FRAME_TYPE_GYRO)
    {
        s_hasGyroFrame = 1U;
        s_gyroTimebaseFault = 0U;
        s_data.last_gyro_frame_ms = now;
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
                s_discardingUnsynced = 0U;
            }
            else
            {
                if (s_discardingUnsynced == 0U)
                {
                    s_data.sync_error_count++;
                    s_discardingUnsynced = 1U;
                }
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

static uint8_t JY61P_CalculateAge(uint32_t now,
                                  uint8_t hasFrame,
                                  uint32_t timestamp,
                                  uint8_t *faultLatched,
                                  uint32_t *age)
{
    uint32_t elapsed;

    if (hasFrame == 0U)
    {
        if ((timestamp != 0U) && (*faultLatched == 0U))
        {
            s_data.timebase_fault_count++;
            *faultLatched = 1U;
        }
        *age = JY61P_AGE_UNKNOWN_MS;
        return 0U;
    }

    if (*faultLatched != 0U)
    {
        *age = JY61P_AGE_UNKNOWN_MS;
        return 0U;
    }

    elapsed = (uint32_t)(now - timestamp);
    if (elapsed > JY61P_MAX_VALID_AGE_MS)
    {
        s_data.timebase_fault_count++;
        *faultLatched = 1U;
        *age = JY61P_AGE_UNKNOWN_MS;
        return 0U;
    }

    *age = elapsed;
    return 1U;
}

static void JY61P_UpdateFreshness(uint32_t now)
{
    uint8_t linkTimeValid = JY61P_CalculateAge(
        now, s_hasValidFrame, s_data.last_valid_frame_ms,
        &s_linkTimebaseFault, &s_data.link_age_ms);
    uint8_t angleTimeValid = JY61P_CalculateAge(
        now, s_hasAngleFrame, s_data.last_angle_frame_ms,
        &s_angleTimebaseFault, &s_data.angle_age_ms);
    uint8_t gyroTimeValid = JY61P_CalculateAge(
        now, s_hasGyroFrame, s_data.last_gyro_frame_ms,
        &s_gyroTimebaseFault, &s_data.gyro_age_ms);

    s_data.online = (uint8_t)((linkTimeValid != 0U) &&
        (s_data.link_age_ms < JY61P_LINK_TIMEOUT_MS));
    s_data.angle_valid = (uint8_t)((angleTimeValid != 0U) &&
        (s_data.angle_age_ms < JY61P_ANGLE_TIMEOUT_MS));
    s_data.gyro_valid = (uint8_t)((gyroTimeValid != 0U) &&
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
    s_data.timebase_fault_count = 0U;
    s_data.yaw_state_fault_count = 0U;
    s_data.last_valid_frame_ms = 0U;
    s_data.last_angle_frame_ms = 0U;
    s_data.last_gyro_frame_ms = 0U;
    s_data.link_age_ms = JY61P_AGE_UNKNOWN_MS;
    s_data.angle_age_ms = JY61P_AGE_UNKNOWN_MS;
    s_data.gyro_age_ms = JY61P_AGE_UNKNOWN_MS;
    s_data.yaw_zero_offset_x100 = 0;
    s_data.angle_valid = 0U;
    s_data.gyro_valid = 0U;
    s_data.online = 0U;
    s_data.yaw_zero_valid = 0U;

    s_hasValidFrame = 0U;
    s_hasAngleFrame = 0U;
    s_hasGyroFrame = 0U;
    s_linkTimebaseFault = 0U;
    s_angleTimebaseFault = 0U;
    s_gyroTimebaseFault = 0U;
    s_discardingUnsynced = 0U;
    s_yawZeroOffsetX100 = 0;
    s_yawZeroValid = 0U;

    Serial_Init();
    s_rxOverflowBaseline = Serial_GetRxOverflowCount();
    s_initDone = 1U;
}

void JY61P_Task10ms(void)
{
    uint8_t byte;
    uint32_t now;

    if (!s_initDone)
    {
        return;
    }

    s_data.rx_overflow_count =
        (uint32_t)(Serial_GetRxOverflowCount() - s_rxOverflowBaseline);

    while (Serial_ReadByte(&byte))
    {
        JY61P_ProcessByte(byte);
    }

    now = Timer_GetMillis();
    JY61P_UpdateFreshness(now);
}

uint8_t JY61P_ResetRelativeYaw(void)
{
    uint32_t now = Timer_GetMillis();

    JY61P_UpdateFreshness(now);
    if ((s_data.online == 0U) ||
        (s_data.angle_valid == 0U) ||
        (s_hasAngleFrame == 0U))
    {
        return 0U;
    }

    s_yawZeroOffsetX100 = (int32_t)s_data.yaw_x100;
    s_yawZeroValid = 1U;
    s_data.yaw_zero_offset_x100 = s_data.yaw_x100;
    s_data.yaw_zero_valid = 1U;
    s_data.relative_yaw_x100 = 0;
    return 1U;
}

uint8_t JY61P_GetYawZero(int16_t *offset_x100)
{
    if (offset_x100 == 0)
    {
        return 0U;
    }

    *offset_x100 = s_data.yaw_zero_offset_x100;
    return s_data.yaw_zero_valid;
}

void JY61P_ClearStatistics(void)
{
    s_data.angle_frame_count = 0U;
    s_data.gyro_frame_count = 0U;
    s_data.checksum_error_count = 0U;
    s_data.sync_error_count = 0U;
    s_data.unsupported_frame_count = 0U;
    s_data.timebase_fault_count = 0U;
    s_data.yaw_state_fault_count = 0U;
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
