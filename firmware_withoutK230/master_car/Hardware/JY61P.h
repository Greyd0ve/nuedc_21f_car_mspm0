#ifndef __JY61P_H
#define __JY61P_H

#include <stdint.h>

#define JY61P_LINK_TIMEOUT_MS  1000U
#define JY61P_ANGLE_TIMEOUT_MS  500U
#define JY61P_GYRO_TIMEOUT_MS   500U
#define JY61P_AGE_UNKNOWN_MS    ((uint32_t)0xFFFFFFFFUL)

#define JY61P_RAW_TRACE_SIZE 32U

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

void JY61P_GetRawTrace(uint8_t *buf, uint8_t buf_size, uint8_t *valid_count);

#endif
