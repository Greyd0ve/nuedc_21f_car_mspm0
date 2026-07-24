#ifndef __JY61P_H
#define __JY61P_H

#include <stdint.h>

/*
 * JY61P (WIT-motion) 6-axis IMU driver via UART0.
 *
 * Protocol: 11-byte fixed frame, little-endian int16_t, sum-of-first-10-bytes
 * checksum.  Frame types decoded:
 *   0x52 = angular velocity (gyro)
 *   0x53 = angle (roll/pitch/yaw)
 *
 * Hardware check: JY61P TX/RX are 3.3V tolerant. If the module is powered from
 * 5V and its UART outputs 5V, add level shifting or power it from 3.3V.
 *
 * JY61P 6-axis yaw is integrated from gyro Z; it drifts over time.
 * Suitable for short-term heading correction, NOT long-term absolute heading.
 */

#define JY61P_LINK_TIMEOUT_MS  1000U
#define JY61P_ANGLE_TIMEOUT_MS  500U
#define JY61P_GYRO_TIMEOUT_MS   500U
#define JY61P_AGE_UNKNOWN_MS    ((uint32_t)0xFFFFFFFFUL)

typedef struct
{
    int16_t roll_x100;
    int16_t pitch_x100;
    int16_t yaw_x100;
    int16_t relative_yaw_x100;

    int16_t gyro_x_dps_x10;
    int16_t gyro_y_dps_x10;
    int16_t gyro_z_dps_x10;

    uint32_t angle_frame_count;
    uint32_t gyro_frame_count;
    uint32_t checksum_error_count;
    uint32_t sync_error_count;
    uint32_t unsupported_frame_count;
    uint32_t rx_overflow_count;
    uint32_t timebase_fault_count;
    uint32_t yaw_state_fault_count;
    uint32_t last_valid_frame_ms;
    uint32_t last_angle_frame_ms;
    uint32_t last_gyro_frame_ms;
    uint32_t link_age_ms;
    uint32_t angle_age_ms;
    uint32_t gyro_age_ms;

    int16_t yaw_zero_offset_x100;
    uint8_t angle_valid;
    uint8_t gyro_valid;
    uint8_t online;
    uint8_t yaw_zero_valid;
} JY61P_Data_t;

void JY61P_Init(void);
void JY61P_Task10ms(void);
uint8_t JY61P_ResetRelativeYaw(void);
uint8_t JY61P_GetYawZero(int16_t *offset_x100);
void JY61P_ClearStatistics(void);
uint8_t JY61P_GetData(JY61P_Data_t *data);
uint8_t JY61P_IsOnline(void);

#endif
