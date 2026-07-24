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
    uint32_t rx_overflow_count;
    uint32_t last_valid_frame_ms;

    uint8_t angle_valid;
    uint8_t gyro_valid;
    uint8_t online;
} JY61P_Data_t;

void JY61P_Init(void);
void JY61P_Task10ms(void);
uint8_t JY61P_ResetRelativeYaw(void);
void JY61P_ClearStatistics(void);
uint8_t JY61P_GetData(JY61P_Data_t *data);
uint8_t JY61P_IsOnline(void);

#endif
