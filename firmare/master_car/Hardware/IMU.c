#include "IMU.h"
#include "Board_Config.h"
#include "ti_msp_dl_config.h"

#define MPU6050_REG_WHO_AM_I     0x75U
#define MPU6050_REG_PWR_MGMT_1   0x6BU
#define MPU6050_REG_PWR_MGMT_2   0x6CU
#define MPU6050_REG_SMPLRT_DIV   0x19U
#define MPU6050_REG_CONFIG       0x1AU
#define MPU6050_REG_GYRO_CONFIG  0x1BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
#define MPU6050_REG_GYRO_XOUT_H  0x43U

#define IMU_ERROR_NONE              0U
#define IMU_ERROR_SOFT_START        1U
#define IMU_ERROR_ADDR_WRITE_NACK   2U
#define IMU_ERROR_REG_NACK          3U
#define IMU_ERROR_ADDR_READ_NACK    4U
#define IMU_ERROR_DATA_TIMEOUT      5U
#define IMU_ERROR_WHO_MISMATCH      6U
#define IMU_ERROR_WRITE_REG         7U
#define IMU_ERROR_WRITE_PWR_FAIL    8U
#define IMU_ERROR_WRITE_SMPR_FAIL   9U
#define IMU_ERROR_WRITE_CONF_FAIL   10U
#define IMU_ERROR_WRITE_GYRO_FAIL   11U
#define IMU_ERROR_WRITE_ACC_FAIL    12U
#define IMU_ERROR_WRITE_PWR2_FAIL   13U

#define IMU_SOFT_RETRY_COUNT        3U

#define IMU_SDA_PORT    GPIO_I2C_SHARED_SDA_PORT
#define IMU_SDA_PIN     GPIO_I2C_SHARED_SDA_PIN
#define IMU_SDA_IOMUX   GPIO_I2C_SHARED_IOMUX_SDA
#define IMU_SCL_PORT    GPIO_I2C_SHARED_SCL_PORT
#define IMU_SCL_PIN     GPIO_I2C_SHARED_SCL_PIN
#define IMU_SCL_IOMUX   GPIO_I2C_SHARED_IOMUX_SCL

static uint8_t  s_imuReady       = 0U;
static uint8_t  s_imuHealthy     = 0U;
static int16_t  s_gyroZOffset    = 0;
static int32_t  s_yawDeg_x10     = 0;
static uint8_t  s_yawValid       = 0U;
static uint8_t  s_yawFault       = 0U;
static uint8_t  s_mpuAddr        = 0x68U;
static uint8_t  s_mpuAddrValid   = 0U;
static uint8_t  s_lastErrorStage = IMU_ERROR_NONE;
static uint8_t  s_initErrorStage = IMU_ERROR_NONE;
static uint32_t s_lastI2CStatus  = 0U;
static int16_t  s_lastGyroXRaw   = 0;
static int16_t  s_lastGyroYRaw   = 0;
static int16_t  s_lastGyroZRaw   = 0;
static int16_t  s_lastGyroZDps_x10 = 0;
static int32_t  s_lastYawDelta_x10 = 0;
static uint8_t  s_lastGyroBytes[6];

static uint8_t IMU_ReadGyroRawSingle(int16_t *gx, int16_t *gy, int16_t *gz);

static void IMU_IIC_DelayUs(uint32_t us)
{
    delay_cycles((CPUCLK_FREQ / 1000000UL) * us);
}

static void IMU_SDA_RELEASE(void)
{
    DL_GPIO_initDigitalInputFeatures(IMU_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

static void IMU_SDA_LOW(void)
{
    DL_GPIO_initDigitalOutput(IMU_SDA_IOMUX);
    DL_GPIO_clearPins(IMU_SDA_PORT, IMU_SDA_PIN);
    DL_GPIO_enableOutput(IMU_SDA_PORT, IMU_SDA_PIN);
}

static void IMU_SCL_RELEASE(void)
{
    DL_GPIO_initDigitalInputFeatures(IMU_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

static void IMU_SCL_LOW(void)
{
    DL_GPIO_initDigitalOutput(IMU_SCL_IOMUX);
    DL_GPIO_clearPins(IMU_SCL_PORT, IMU_SCL_PIN);
    DL_GPIO_enableOutput(IMU_SCL_PORT, IMU_SCL_PIN);
}

static uint8_t IMU_SDA_GET(void)
{
    return (uint8_t)(((DL_GPIO_readPins(IMU_SDA_PORT, IMU_SDA_PIN) & IMU_SDA_PIN) != 0U) ? 1U : 0U);
}

static void IMU_SoftI2CInit(void)
{
    DL_I2C_disableController(I2C_SHARED_INST);

    IMU_SCL_RELEASE();
    IMU_SDA_RELEASE();

    {
        volatile uint32_t d = 10000U;
        while (d > 0U) { d--; }
    }
}

static void IMU_IIC_BusRecover(void)
{
    uint8_t i;

    IMU_SDA_RELEASE();
    IMU_SCL_RELEASE();
    IMU_IIC_DelayUs(20);

    if (IMU_SDA_GET() == 0U)
    {
        for (i = 0U; i < 9U; i++)
        {
            IMU_SCL_LOW();
            IMU_IIC_DelayUs(10);
            IMU_SCL_RELEASE();
            IMU_IIC_DelayUs(10);
        }
    }

    IMU_SCL_LOW();
    IMU_SDA_LOW();
    IMU_SCL_RELEASE();
    IMU_IIC_DelayUs(5);
    IMU_SDA_RELEASE();
    IMU_IIC_DelayUs(20);
}

static void IMU_IIC_Start(void)
{
    IMU_SCL_LOW();
    IMU_SDA_RELEASE();
    IMU_SCL_RELEASE();
    IMU_IIC_DelayUs(5);
    IMU_SDA_LOW();
    IMU_IIC_DelayUs(5);
    IMU_SCL_LOW();
    IMU_IIC_DelayUs(5);
}

static void IMU_IIC_Stop(void)
{
    IMU_SCL_LOW();
    IMU_SDA_LOW();
    IMU_IIC_DelayUs(5);
    IMU_SCL_RELEASE();
    IMU_IIC_DelayUs(5);
    IMU_SDA_RELEASE();
    IMU_IIC_DelayUs(10);
}

static void IMU_IIC_SendAck(uint8_t ack)
{
    IMU_SCL_LOW();
    IMU_IIC_DelayUs(5);
    if (ack) { IMU_SDA_RELEASE(); }
    else     { IMU_SDA_LOW(); }
    IMU_SCL_RELEASE();
    IMU_IIC_DelayUs(5);
    IMU_SCL_LOW();
    IMU_SDA_LOW();
}

static uint8_t IMU_IIC_WaitAck(void)
{
    uint8_t ack;
    uint8_t timeout = 200U;

    IMU_SCL_LOW();
    IMU_SDA_RELEASE();
    IMU_IIC_DelayUs(5);
    IMU_SCL_RELEASE();
    IMU_IIC_DelayUs(5);

    while ((IMU_SDA_GET() == 1U) && (timeout > 0U))
    {
        timeout--;
        IMU_IIC_DelayUs(5);
    }

    if (timeout == 0U) { IMU_IIC_Stop(); return 0U; }

    ack = IMU_SDA_GET();
    IMU_SCL_LOW();

    if (ack != 0U) { return 0U; }
    return 1U;
}

static void IMU_IIC_SendByte(uint8_t dat)
{
    uint8_t i;
    IMU_SCL_LOW();
    for (i = 0; i < 8U; i++)
    {
        if (dat & 0x80U) { IMU_SDA_RELEASE(); }
        else             { IMU_SDA_LOW(); }
        IMU_IIC_DelayUs(1);
        IMU_SCL_RELEASE();
        IMU_IIC_DelayUs(5);
        IMU_SCL_LOW();
        IMU_IIC_DelayUs(5);
        dat <<= 1U;
    }
}

static uint8_t IMU_IIC_ReadByte(void)
{
    uint8_t i;
    uint8_t receive = 0U;

    IMU_SDA_RELEASE();
    for (i = 0; i < 8U; i++)
    {
        IMU_SCL_LOW();
        IMU_IIC_DelayUs(5);
        IMU_SCL_RELEASE();
        IMU_IIC_DelayUs(5);
        receive <<= 1U;
        if (IMU_SDA_GET()) { receive |= 1U; }
    }
    IMU_SCL_LOW();
    return receive;
}

static uint8_t IMU_SoftWriteRegRetry(uint8_t addr, uint8_t reg, uint8_t value, uint8_t retryCount)
{
    uint8_t attempt;
    for (attempt = 0U; attempt < retryCount; attempt++)
    {
        IMU_IIC_BusRecover();
        IMU_IIC_Start();
        IMU_IIC_SendByte((uint8_t)((addr << 1U) | 0U));
        if (!IMU_IIC_WaitAck()) { IMU_IIC_Stop(); s_lastErrorStage = IMU_ERROR_ADDR_WRITE_NACK; continue; }
        IMU_IIC_SendByte(reg);
        if (!IMU_IIC_WaitAck()) { IMU_IIC_Stop(); s_lastErrorStage = IMU_ERROR_REG_NACK; continue; }
        IMU_IIC_SendByte(value);
        if (!IMU_IIC_WaitAck()) { IMU_IIC_Stop(); s_lastErrorStage = IMU_ERROR_WRITE_REG; continue; }
        IMU_IIC_Stop();
        s_lastErrorStage = IMU_ERROR_NONE;
        return 1U;
    }
    return 0U;
}

static uint8_t IMU_SoftReadRegsRetry(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, uint8_t retryCount)
{
    uint8_t i;
    uint8_t attempt;
    if (data == 0 || len == 0U) { return 0U; }
    for (attempt = 0U; attempt < retryCount; attempt++)
    {
        IMU_IIC_BusRecover();
        IMU_IIC_Start();
        IMU_IIC_SendByte((uint8_t)((addr << 1U) | 0U));
        if (!IMU_IIC_WaitAck()) { IMU_IIC_Stop(); s_lastErrorStage = IMU_ERROR_ADDR_WRITE_NACK; continue; }
        IMU_IIC_SendByte(reg);
        if (!IMU_IIC_WaitAck()) { IMU_IIC_Stop(); s_lastErrorStage = IMU_ERROR_REG_NACK; continue; }
        IMU_IIC_Start();
        IMU_IIC_SendByte((uint8_t)((addr << 1U) | 1U));
        if (!IMU_IIC_WaitAck()) { IMU_IIC_Stop(); s_lastErrorStage = IMU_ERROR_ADDR_READ_NACK; continue; }
        for (i = 0; i < len; i++)
        {
            data[i] = IMU_IIC_ReadByte();
            IMU_IIC_SendAck((uint8_t)((i == (uint8_t)(len - 1U)) ? 1U : 0U));
        }
        IMU_IIC_Stop();
        s_lastErrorStage = IMU_ERROR_NONE;
        return 1U;
    }
    return 0U;
}

static uint8_t IMU_SoftReadRegRetry(uint8_t addr, uint8_t reg, uint8_t *value, uint8_t retryCount)
{
    return IMU_SoftReadRegsRetry(addr, reg, value, 1U, retryCount);
}

uint8_t IMU_ProbeAddressAck(uint8_t addr)
{
    IMU_IIC_BusRecover();
    IMU_IIC_Start();
    IMU_IIC_SendByte((uint8_t)((addr << 1U) | 0U));
    if (!IMU_IIC_WaitAck()) { IMU_IIC_Stop(); return 0U; }
    IMU_IIC_Stop();
    return 1U;
}

static uint8_t IMU_IsValidWhoAmI(uint8_t who)
{
    return (uint8_t)((who == 0x68U || who == 0x70U) ? 1U : 0U);
}

uint8_t IMU_ReadWhoAmI(uint8_t *whoAmI)
{
    uint8_t data;
    if (whoAmI == 0) { return 0U; }
    *whoAmI = 0U;
    if (!IMU_SoftReadRegRetry(s_mpuAddr, MPU6050_REG_WHO_AM_I, &data, IMU_SOFT_RETRY_COUNT)) { return 0U; }
    *whoAmI = data;
    return 1U;
}

uint8_t IMU_ProbeAddress(uint8_t addr)
{
    uint8_t savedAddr;
    uint8_t who;
    uint8_t result;
    savedAddr = s_mpuAddr;
    s_mpuAddr = addr;
    result = IMU_ReadWhoAmI(&who);
    if (result && IMU_IsValidWhoAmI(who)) { s_mpuAddr = addr; return 1U; }
    s_mpuAddr = savedAddr;
    return 0U;
}

uint8_t IMU_Scan(uint8_t *foundAddr)
{
    uint8_t round;
    if (foundAddr == 0) { return 0U; }
    *foundAddr = 0U;
    s_mpuAddrValid = 0U;

    for (round = 0U; round < 10U; round++)
    {
        IMU_IIC_BusRecover();
        if (IMU_ProbeAddressAck(0x68U)) { *foundAddr = 0x68U; s_mpuAddr = 0x68U; s_mpuAddrValid = 1U; return 1U; }
        if (IMU_ProbeAddressAck(0x69U)) { *foundAddr = 0x69U; s_mpuAddr = 0x69U; s_mpuAddrValid = 1U; return 1U; }
        IMU_IIC_DelayUs(200);
    }
    return 0U;
}

void IMU_Init(void)
{
    uint8_t who = 0U;
    uint8_t foundAddr = 0U;
    uint8_t i;

    IMU_SoftI2CInit();

    {
        volatile uint32_t delay = 3200000U;
        while (delay > 0U) { delay--; }
    }

    s_imuReady = 0U;
    s_imuHealthy = 0U;
    s_gyroZOffset = 0;
    s_yawDeg_x10 = 0;
    s_yawValid = 0U;
    s_mpuAddr = 0x68U;
    s_mpuAddrValid = 0U;
    s_lastErrorStage = IMU_ERROR_NONE;
    s_initErrorStage = IMU_ERROR_NONE;
    s_lastI2CStatus = 0U;

    for (i = 0U; i < 5U; i++) { IMU_IIC_BusRecover(); }

    if (!IMU_ReadWhoAmI(&who) || !IMU_IsValidWhoAmI(who))
    {
        if (!IMU_Scan(&foundAddr))
        {
            s_initErrorStage = (s_lastErrorStage != IMU_ERROR_NONE) ? s_lastErrorStage : IMU_ERROR_ADDR_WRITE_NACK;
            return;
        }
        if (!IMU_ReadWhoAmI(&who) || !IMU_IsValidWhoAmI(who))
        {
            s_lastErrorStage = IMU_ERROR_WHO_MISMATCH;
            s_initErrorStage = IMU_ERROR_WHO_MISMATCH;
            return;
        }
    }
    else
    {
        s_mpuAddr = 0x68U;
        s_mpuAddrValid = 1U;
        s_lastErrorStage = IMU_ERROR_NONE;
    }

    if (!IMU_SoftWriteRegRetry(s_mpuAddr, MPU6050_REG_PWR_MGMT_1, 0x01U, IMU_SOFT_RETRY_COUNT))
    { s_lastErrorStage = IMU_ERROR_WRITE_PWR_FAIL; s_initErrorStage = IMU_ERROR_WRITE_PWR_FAIL; return; }

    if (!IMU_SoftWriteRegRetry(s_mpuAddr, MPU6050_REG_PWR_MGMT_2, 0x00U, IMU_SOFT_RETRY_COUNT))
    { s_lastErrorStage = IMU_ERROR_WRITE_PWR2_FAIL; s_initErrorStage = IMU_ERROR_WRITE_PWR2_FAIL; return; }

    if (!IMU_SoftWriteRegRetry(s_mpuAddr, MPU6050_REG_SMPLRT_DIV, 0x07U, IMU_SOFT_RETRY_COUNT))
    { s_lastErrorStage = IMU_ERROR_WRITE_SMPR_FAIL; s_initErrorStage = IMU_ERROR_WRITE_SMPR_FAIL; return; }

    if (!IMU_SoftWriteRegRetry(s_mpuAddr, MPU6050_REG_CONFIG, 0x03U, IMU_SOFT_RETRY_COUNT))
    { s_lastErrorStage = IMU_ERROR_WRITE_CONF_FAIL; s_initErrorStage = IMU_ERROR_WRITE_CONF_FAIL; return; }

    if (!IMU_SoftWriteRegRetry(s_mpuAddr, MPU6050_REG_GYRO_CONFIG, 0x00U, IMU_SOFT_RETRY_COUNT))
    { s_lastErrorStage = IMU_ERROR_WRITE_GYRO_FAIL; s_initErrorStage = IMU_ERROR_WRITE_GYRO_FAIL; return; }

    if (!IMU_SoftWriteRegRetry(s_mpuAddr, MPU6050_REG_ACCEL_CONFIG, 0x00U, IMU_SOFT_RETRY_COUNT))
    { s_lastErrorStage = IMU_ERROR_WRITE_ACC_FAIL; s_initErrorStage = IMU_ERROR_WRITE_ACC_FAIL; return; }

    s_imuReady = 1U;
    s_imuHealthy = 1U;
    s_lastErrorStage = IMU_ERROR_NONE;
    s_initErrorStage = IMU_ERROR_NONE;
}

uint8_t IMU_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz)
{
    return IMU_ReadGyroRawSingle(gx, gy, gz);
}

static uint8_t IMU_ReadGyroRawSingle(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t hi, lo;
    if (gx == 0 || gy == 0 || gz == 0) { return 0U; }
    if (!s_imuReady) { return 0U; }

    if (!IMU_SoftReadRegRetry(s_mpuAddr, 0x43U, &hi, IMU_SOFT_RETRY_COUNT)) { s_imuHealthy = 0U; return 0U; }
    if (!IMU_SoftReadRegRetry(s_mpuAddr, 0x44U, &lo, IMU_SOFT_RETRY_COUNT)) { s_imuHealthy = 0U; return 0U; }
    *gx = (int16_t)(((uint16_t)hi << 8U) | (uint16_t)lo);

    if (!IMU_SoftReadRegRetry(s_mpuAddr, 0x45U, &hi, IMU_SOFT_RETRY_COUNT)) { s_imuHealthy = 0U; return 0U; }
    if (!IMU_SoftReadRegRetry(s_mpuAddr, 0x46U, &lo, IMU_SOFT_RETRY_COUNT)) { s_imuHealthy = 0U; return 0U; }
    *gy = (int16_t)(((uint16_t)hi << 8U) | (uint16_t)lo);

    if (!IMU_SoftReadRegRetry(s_mpuAddr, 0x47U, &hi, IMU_SOFT_RETRY_COUNT)) { s_imuHealthy = 0U; return 0U; }
    if (!IMU_SoftReadRegRetry(s_mpuAddr, 0x48U, &lo, IMU_SOFT_RETRY_COUNT)) { s_imuHealthy = 0U; return 0U; }
    *gz = (int16_t)(((uint16_t)hi << 8U) | (uint16_t)lo);

    s_lastGyroXRaw = *gx; s_lastGyroYRaw = *gy; s_lastGyroZRaw = *gz;
    {
        uint16_t rawX = (uint16_t)(*gx); uint16_t rawY = (uint16_t)(*gy); uint16_t rawZ = (uint16_t)(*gz);
        s_lastGyroBytes[0] = (uint8_t)(rawX >> 8U); s_lastGyroBytes[1] = (uint8_t)(rawX & 0xFFU);
        s_lastGyroBytes[2] = (uint8_t)(rawY >> 8U); s_lastGyroBytes[3] = (uint8_t)(rawY & 0xFFU);
        s_lastGyroBytes[4] = (uint8_t)(rawZ >> 8U); s_lastGyroBytes[5] = (uint8_t)(rawZ & 0xFFU);
    }
    s_imuHealthy = 1U; s_lastErrorStage = IMU_ERROR_NONE;
    return 1U;
}

void IMU_CalibrateGyroZ(uint16_t samples)
{
    int32_t sum = 0; uint16_t i; uint16_t validCount = 0U; int16_t gz;
    if (samples == 0U) { samples = 200U; }
    for (i = 0; i < samples; i++) { int16_t dx, dy; if (IMU_ReadGyroRaw(&dx, &dy, &gz)) { sum += (int32_t)gz; validCount++; } }
    s_gyroZOffset = (validCount > 0U) ? (int16_t)(sum / (int32_t)validCount) : 0;
    s_yawDeg_x10 = 0; s_yawValid = 1U; s_yawFault = 0U;
}

void IMU_ResetYaw(void)
{ s_yawDeg_x10 = 0; s_yawValid = 1U; s_yawFault = 0U; s_lastYawDelta_x10 = 0; }

int32_t IMU_GetYawDeg_x10(void)
{
    if (s_yawDeg_x10 > 36000L || s_yawDeg_x10 < -36000L)
    { s_yawFault = 1U; s_imuHealthy = 0U; s_yawDeg_x10 = 0L; }
    return s_yawDeg_x10;
}

uint8_t IMU_IsReady(void)         { return s_imuReady; }
uint8_t IMU_IsHealthy(void)       { return (uint8_t)((s_imuHealthy != 0U && s_yawFault == 0U) ? 1U : 0U); }
int16_t IMU_GetGyroZOffset(void)   { return s_gyroZOffset; }
int16_t IMU_GetLastGyroXRaw(void)  { return s_lastGyroXRaw; }
int16_t IMU_GetLastGyroYRaw(void)  { return s_lastGyroYRaw; }
int16_t IMU_GetLastGyroZRaw(void)  { return s_lastGyroZRaw; }
int16_t IMU_GetLastGyroZDps_x10(void) { return s_lastGyroZDps_x10; }
int32_t IMU_GetLastYawDelta_x10(void) { return s_lastYawDelta_x10; }
uint8_t IMU_GetLastGyroByte(uint8_t index) { return (index < 6U) ? s_lastGyroBytes[index] : 0U; }
uint8_t IMU_DebugReadReg(uint8_t reg, uint8_t *value) { return IMU_SoftReadRegRetry(s_mpuAddr, reg, value, IMU_SOFT_RETRY_COUNT); }
uint8_t IMU_GetAddr(void)         { return s_mpuAddr; }
uint8_t IMU_IsAddrValid(void)     { return s_mpuAddrValid; }
uint8_t IMU_GetRecoverCount(void) { return 0U; }
uint32_t IMU_GetLastI2CStatus(void) { return s_lastI2CStatus; }
uint8_t IMU_GetLastErrorStage(void) { return s_lastErrorStage; }
uint8_t IMU_GetInitErrorStage(void) { return s_initErrorStage; }

uint8_t IMU_GetSdaLevel(void) { IMU_SDA_RELEASE(); return IMU_SDA_GET(); }
uint8_t IMU_GetSclLevel(void) { IMU_SCL_RELEASE(); return (uint8_t)(((DL_GPIO_readPins(IMU_SCL_PORT, IMU_SCL_PIN) & IMU_SCL_PIN) != 0U) ? 1U : 0U); }

const char *IMU_GetErrorStageName(uint8_t stage)
{
    switch (stage)
    {
        case IMU_ERROR_NONE:            return "none";
        case IMU_ERROR_SOFT_START:      return "soft_start";
        case IMU_ERROR_ADDR_WRITE_NACK: return "addr_write_nack";
        case IMU_ERROR_REG_NACK:        return "reg_nack";
        case IMU_ERROR_ADDR_READ_NACK:  return "addr_read_nack";
        case IMU_ERROR_DATA_TIMEOUT:    return "data_timeout";
        case IMU_ERROR_WHO_MISMATCH:    return "who_mismatch";
        case IMU_ERROR_WRITE_REG:       return "write_reg_fail";
        case IMU_ERROR_WRITE_PWR_FAIL:  return "write_pwr_fail";
        case IMU_ERROR_WRITE_SMPR_FAIL: return "write_smpr_fail";
        case IMU_ERROR_WRITE_CONF_FAIL: return "write_config_fail";
        case IMU_ERROR_WRITE_GYRO_FAIL: return "write_gyro_config_fail";
        case IMU_ERROR_WRITE_ACC_FAIL:  return "write_accl_fail";
        case IMU_ERROR_WRITE_PWR2_FAIL: return "write_pwr2_fail";
        default:                        return "unknown";
    }
}

uint8_t IMU_StatusHasIdle(uint32_t status)  { (void)status; return 1U; }
uint8_t IMU_StatusHasBusy(uint32_t status)  { (void)status; return 0U; }
uint8_t IMU_StatusHasError(uint32_t status) { (void)status; return 0U; }

uint8_t IMU_GetGyroRawZ_x10(int16_t *rawZ_x10, int16_t *dps_x10)
{
    int16_t gzRaw; int16_t dx, dy;
    if (rawZ_x10 == 0 || dps_x10 == 0) { return 0U; }
    if (!IMU_ReadGyroRaw(&dx, &dy, &gzRaw)) { return 0U; }
    *rawZ_x10 = gzRaw;
    *dps_x10 = (int16_t)((int32_t)(gzRaw - s_gyroZOffset) * 2500L / 32768L);
    s_lastGyroZDps_x10 = *dps_x10;
    return 1U;
}

void IMU_UpdateYaw(uint16_t dt_ms)
{
    int16_t gzRaw; int16_t dx, dy; int64_t dps_x100; int64_t delta_x10;
    if (!s_yawValid || !s_imuReady) { return; }
    if (!IMU_ReadGyroRaw(&dx, &dy, &gzRaw)) { return; }
    dps_x100 = (int64_t)(gzRaw - s_gyroZOffset) * 25000LL / 32768LL;
    delta_x10 = (dps_x100 * (int64_t)dt_ms) / 10000LL;
    s_lastYawDelta_x10 = (int32_t)delta_x10;
    if (delta_x10 > 100LL || delta_x10 < -100LL) { s_yawFault = 1U; s_imuHealthy = 0U; return; }
    s_yawDeg_x10 += (int32_t)delta_x10;
    if (s_yawDeg_x10 > 36000L || s_yawDeg_x10 < -36000L) { s_yawFault = 1U; s_imuHealthy = 0U; s_yawDeg_x10 = 0L; return; }
}
