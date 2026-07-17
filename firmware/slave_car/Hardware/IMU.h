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
int16_t IMU_GetGyroZOffset(void);
int16_t IMU_GetLastGyroXRaw(void);
int16_t IMU_GetLastGyroYRaw(void);
int16_t IMU_GetLastGyroZRaw(void);
int16_t IMU_GetLastGyroZDps_x10(void);
int32_t IMU_GetLastYawDelta_x10(void);
uint8_t IMU_GetLastGyroByte(uint8_t index);
uint8_t IMU_DebugReadReg(uint8_t reg, uint8_t *value);

uint8_t IMU_GetGyroRawZ_x10(int16_t *rawZ_x10, int16_t *dps_x10);

uint8_t IMU_ProbeAddress(uint8_t addr);
uint8_t IMU_ProbeAddressAck(uint8_t addr);
uint8_t IMU_Scan(uint8_t *foundAddr);
uint32_t IMU_GetLastI2CStatus(void);
uint8_t IMU_GetLastErrorStage(void);
uint8_t IMU_GetInitErrorStage(void);
const char *IMU_GetErrorStageName(uint8_t stage);
uint8_t IMU_GetSdaLevel(void);
uint8_t IMU_GetSclLevel(void);
uint8_t IMU_GetAddr(void);
uint8_t IMU_IsAddrValid(void);
uint8_t IMU_StatusHasIdle(uint32_t status);
uint8_t IMU_StatusHasBusy(uint32_t status);
uint8_t IMU_StatusHasError(uint32_t status);

#endif
