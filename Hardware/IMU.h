#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

#define MPU6050_WHO_AM_I_VAL  0x68U

void IMU_Init(void);
uint8_t IMU_ReadWhoAmI(uint8_t *whoAmI);
uint8_t IMU_IsReady(void);

uint8_t IMU_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz);
void    IMU_CalibrateGyroZ(uint16_t samples);
void    IMU_ResetYaw(void);
void    IMU_UpdateYaw(uint16_t dt_ms);
int32_t IMU_GetYawDeg_x10(void);
uint8_t IMU_IsHealthy(void);

uint8_t IMU_GetGyroRawZ_x10(int16_t *rawZ_x10, int16_t *dps_x10);

uint8_t IMU_ProbeAddress(uint8_t addr);
uint8_t IMU_ProbeAddressAck(uint8_t addr);
uint8_t IMU_Scan(uint8_t *foundAddr);
uint32_t IMU_GetLastI2CStatus(void);
uint8_t IMU_GetLastErrorStage(void);
uint8_t IMU_GetAddr(void);
uint8_t IMU_IsAddrValid(void);

#endif
