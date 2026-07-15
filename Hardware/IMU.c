#include "IMU.h"
#include "Board_Config.h"
#include "ti_msp_dl_config.h"

#define MPU6050_REG_WHO_AM_I     0x75U
#define MPU6050_REG_PWR_MGMT_1   0x6BU
#define MPU6050_REG_SMPLRT_DIV   0x19U
#define MPU6050_REG_CONFIG       0x1AU
#define MPU6050_REG_GYRO_CONFIG  0x1BU
#define MPU6050_REG_GYRO_XOUT_H  0x43U

#define MPU6050_GYRO_FS_SEL      0U
#define MPU6050_GYRO_LSB_PER_DPS 131.0f
#define MPU6050_DLPF_CFG         2U

#define IMU_ERROR_NONE              0U
#define IMU_ERROR_SOFT_START        1U
#define IMU_ERROR_ADDR_WRITE_NACK   2U
#define IMU_ERROR_REG_NACK          3U
#define IMU_ERROR_ADDR_READ_NACK    4U
#define IMU_ERROR_DATA_TIMEOUT      5U
#define IMU_ERROR_WHO_MISMATCH      6U
#define IMU_ERROR_WRITE_REG         7U

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
static uint8_t  s_mpuAddr        = 0x68U;
static uint8_t  s_mpuAddrValid   = 0U;
static uint8_t  s_lastErrorStage = IMU_ERROR_NONE;
static uint32_t s_lastI2CStatus  = 0U;

static void IMU_IIC_DelayUs(uint32_t us)
{
    delay_cycles((CPUCLK_FREQ / 1000000UL) * us);
}

static void IMU_SDA_OUT(void)
{
    DL_GPIO_initDigitalOutput(IMU_SDA_IOMUX);
    DL_GPIO_setPins(IMU_SDA_PORT, IMU_SDA_PIN);
    DL_GPIO_enableOutput(IMU_SDA_PORT, IMU_SDA_PIN);
}

static void IMU_SDA_IN(void)
{
    DL_GPIO_initDigitalInput(IMU_SDA_IOMUX);
}

static void IMU_SDA_SET(uint8_t level)
{
    if (level) { DL_GPIO_setPins(IMU_SDA_PORT, IMU_SDA_PIN); }
    else       { DL_GPIO_clearPins(IMU_SDA_PORT, IMU_SDA_PIN); }
}

static uint8_t IMU_SDA_GET(void)
{
    return (uint8_t)(((DL_GPIO_readPins(IMU_SDA_PORT, IMU_SDA_PIN) & IMU_SDA_PIN) != 0U) ? 1U : 0U);
}

static void IMU_SCL_SET(uint8_t level)
{
    if (level) { DL_GPIO_setPins(IMU_SCL_PORT, IMU_SCL_PIN); }
    else       { DL_GPIO_clearPins(IMU_SCL_PORT, IMU_SCL_PIN); }
}

static void IMU_SoftI2CInit(void)
{
    DL_GPIO_initDigitalOutput(IMU_SCL_IOMUX);
    DL_GPIO_setPins(IMU_SCL_PORT, IMU_SCL_PIN);
    DL_GPIO_enableOutput(IMU_SCL_PORT, IMU_SCL_PIN);

    DL_GPIO_initDigitalOutput(IMU_SDA_IOMUX);
    DL_GPIO_setPins(IMU_SDA_PORT, IMU_SDA_PIN);
    DL_GPIO_enableOutput(IMU_SDA_PORT, IMU_SDA_PIN);
}

static void IMU_IIC_Start(void)
{
    IMU_SDA_OUT();
    IMU_SCL_SET(0);
    IMU_SDA_SET(1);
    IMU_SCL_SET(1);
    IMU_IIC_DelayUs(5);
    IMU_SDA_SET(0);
    IMU_IIC_DelayUs(5);
    IMU_SCL_SET(0);
    IMU_IIC_DelayUs(5);
}

static void IMU_IIC_Stop(void)
{
    IMU_SDA_OUT();
    IMU_SCL_SET(0);
    IMU_SDA_SET(0);
    IMU_SCL_SET(1);
    IMU_IIC_DelayUs(5);
    IMU_SDA_SET(1);
    IMU_IIC_DelayUs(5);
}

static void IMU_IIC_SendAck(uint8_t ack)
{
    IMU_SDA_OUT();
    IMU_SCL_SET(0);
    IMU_SDA_SET(0);
    IMU_IIC_DelayUs(5);
    if (ack) { IMU_SDA_SET(1); }
    else     { IMU_SDA_SET(0); }
    IMU_SCL_SET(1);
    IMU_IIC_DelayUs(5);
    IMU_SCL_SET(0);
    IMU_SDA_SET(0);
}

static uint8_t IMU_IIC_WaitAck(void)
{
    uint8_t ack;
    uint8_t timeout = 200U;

    IMU_SDA_IN();
    IMU_SDA_SET(1);
    IMU_IIC_DelayUs(5);
    IMU_SCL_SET(1);
    IMU_IIC_DelayUs(5);

    while ((IMU_SDA_GET() == 1U) && (timeout > 0U))
    {
        timeout--;
        IMU_IIC_DelayUs(5);
    }

    if (timeout == 0U)
    {
        IMU_IIC_Stop();
        return 0U;
    }

    ack = IMU_SDA_GET();
    IMU_SCL_SET(0);
    IMU_SDA_OUT();

    if (ack != 0U) { return 0U; }
    return 1U;
}

static void IMU_IIC_SendByte(uint8_t dat)
{
    uint8_t i;
    IMU_SDA_OUT();
    IMU_SCL_SET(0);
    for (i = 0; i < 8U; i++)
    {
        IMU_SDA_SET((dat & 0x80U) >> 7U);
        IMU_IIC_DelayUs(1);
        IMU_SCL_SET(1);
        IMU_IIC_DelayUs(5);
        IMU_SCL_SET(0);
        IMU_IIC_DelayUs(5);
        dat <<= 1U;
    }
}

static uint8_t IMU_IIC_ReadByte(void)
{
    uint8_t i;
    uint8_t receive = 0U;

    IMU_SDA_IN();
    for (i = 0; i < 8U; i++)
    {
        IMU_SCL_SET(0);
        IMU_IIC_DelayUs(5);
        IMU_SCL_SET(1);
        IMU_IIC_DelayUs(5);
        receive <<= 1U;
        if (IMU_SDA_GET())
        {
            receive |= 1U;
        }
    }
    IMU_SCL_SET(0);
    return receive;
}

static uint8_t IMU_SoftWriteReg(uint8_t addr, uint8_t reg, uint8_t value)
{
    IMU_IIC_Start();
    IMU_IIC_SendByte((uint8_t)((addr << 1U) | 0U));
    if (!IMU_IIC_WaitAck()) { IMU_IIC_Stop(); s_lastErrorStage = IMU_ERROR_ADDR_WRITE_NACK; return 0U; }
    IMU_IIC_SendByte(reg);
    if (!IMU_IIC_WaitAck()) { IMU_IIC_Stop(); s_lastErrorStage = IMU_ERROR_REG_NACK; return 0U; }
    IMU_IIC_SendByte(value);
    if (!IMU_IIC_WaitAck()) { IMU_IIC_Stop(); s_lastErrorStage = IMU_ERROR_WRITE_REG; return 0U; }
    IMU_IIC_Stop();
    return 1U;
}

static uint8_t IMU_SoftReadRegs(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i;

    if (data == 0 || len == 0U) { return 0U; }

    IMU_IIC_Start();
    IMU_IIC_SendByte((uint8_t)((addr << 1U) | 0U));
    if (!IMU_IIC_WaitAck()) { IMU_IIC_Stop(); s_lastErrorStage = IMU_ERROR_ADDR_WRITE_NACK; return 0U; }
    IMU_IIC_SendByte(reg);
    if (!IMU_IIC_WaitAck()) { IMU_IIC_Stop(); s_lastErrorStage = IMU_ERROR_REG_NACK; return 0U; }

    IMU_IIC_Start();
    IMU_IIC_SendByte((uint8_t)((addr << 1U) | 1U));
    if (!IMU_IIC_WaitAck()) { IMU_IIC_Stop(); s_lastErrorStage = IMU_ERROR_ADDR_READ_NACK; return 0U; }

    for (i = 0; i < len; i++)
    {
        data[i] = IMU_IIC_ReadByte();
        IMU_IIC_SendAck((uint8_t)((i == (uint8_t)(len - 1U)) ? 1U : 0U));
    }

    IMU_IIC_Stop();
    return 1U;
}

static uint8_t IMU_SoftReadReg(uint8_t addr, uint8_t reg, uint8_t *value)
{
    return IMU_SoftReadRegs(addr, reg, value, 1U);
}

uint8_t IMU_ProbeAddressAck(uint8_t addr)
{
    IMU_IIC_Start();
    IMU_IIC_SendByte((uint8_t)((addr << 1U) | 0U));
    if (!IMU_IIC_WaitAck())
    {
        IMU_IIC_Stop();
        return 0U;
    }
    IMU_IIC_Stop();
    return 1U;
}

uint8_t IMU_ReadWhoAmI(uint8_t *whoAmI)
{
    uint8_t data;

    if (whoAmI == 0) { return 0U; }
    *whoAmI = 0U;

    if (!IMU_SoftReadReg(s_mpuAddr, MPU6050_REG_WHO_AM_I, &data))
    {
        return 0U;
    }

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
    if (result && who == MPU6050_WHO_AM_I_VAL)
    {
        s_mpuAddr = addr;
        return 1U;
    }

    s_mpuAddr = savedAddr;
    return 0U;
}

uint8_t IMU_Scan(uint8_t *foundAddr)
{
    if (foundAddr == 0) { return 0U; }

    *foundAddr = 0U;
    s_mpuAddrValid = 0U;

    if (IMU_ProbeAddress(0x68U))
    {
        *foundAddr = 0x68U;
        s_mpuAddr = 0x68U;
        s_mpuAddrValid = 1U;
        return 1U;
    }

    if (IMU_ProbeAddress(0x69U))
    {
        *foundAddr = 0x69U;
        s_mpuAddr = 0x69U;
        s_mpuAddrValid = 1U;
        return 1U;
    }

    return 0U;
}

void IMU_Init(void)
{
    uint8_t who = 0U;
    uint8_t foundAddr = 0U;

    IMU_SoftI2CInit();

    s_imuReady = 0U;
    s_imuHealthy = 0U;
    s_gyroZOffset = 0;
    s_yawDeg_x10 = 0;
    s_yawValid = 0U;
    s_mpuAddr = 0x68U;
    s_mpuAddrValid = 0U;
    s_lastErrorStage = IMU_ERROR_NONE;
    s_lastI2CStatus = 0U;

    if (!IMU_ReadWhoAmI(&who) || who != MPU6050_WHO_AM_I_VAL)
    {
        if (!IMU_Scan(&foundAddr)) { return; }
        if (!IMU_ReadWhoAmI(&who) || who != MPU6050_WHO_AM_I_VAL)
        {
            s_lastErrorStage = IMU_ERROR_WHO_MISMATCH;
            return;
        }
    }
    else
    {
        s_mpuAddr = 0x68U;
        s_mpuAddrValid = 1U;
    }

    if (!IMU_SoftWriteReg(s_mpuAddr, MPU6050_REG_PWR_MGMT_1, 0x00U)) { return; }

    {
        uint32_t delay = 200000U;
        while (delay > 0U) { delay--; }
    }

    if (!IMU_SoftWriteReg(s_mpuAddr, MPU6050_REG_SMPLRT_DIV, 0x00U)) { return; }
    if (!IMU_SoftWriteReg(s_mpuAddr, MPU6050_REG_CONFIG, MPU6050_DLPF_CFG)) { return; }
    if (!IMU_SoftWriteReg(s_mpuAddr, MPU6050_REG_GYRO_CONFIG, (MPU6050_GYRO_FS_SEL << 3U))) { return; }

    s_imuReady = 1U;
    s_imuHealthy = 1U;
}

uint8_t IMU_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];

    if (gx == 0 || gy == 0 || gz == 0) { return 0U; }
    if (!s_imuReady) { return 0U; }

    if (!IMU_SoftReadRegs(s_mpuAddr, MPU6050_REG_GYRO_XOUT_H, buf, 6U))
    {
        s_imuHealthy = 0U;
        return 0U;
    }

    *gx = (int16_t)(((uint16_t)buf[0] << 8U) | (uint16_t)buf[1]);
    *gy = (int16_t)(((uint16_t)buf[2] << 8U) | (uint16_t)buf[3]);
    *gz = (int16_t)(((uint16_t)buf[4] << 8U) | (uint16_t)buf[5]);

    s_imuHealthy = 1U;
    return 1U;
}

void IMU_CalibrateGyroZ(uint16_t samples)
{
    int32_t sum = 0;
    uint16_t i;
    uint16_t validCount = 0U;
    int16_t gz;

    if (samples == 0U) { samples = 200U; }

    for (i = 0; i < samples; i++)
    {
        int16_t dummyX, dummyY;
        if (IMU_ReadGyroRaw(&dummyX, &dummyY, &gz))
        {
            sum += (int32_t)gz;
            validCount++;
        }
    }

    if (validCount > 0U)
    {
        s_gyroZOffset = (int16_t)(sum / (int32_t)validCount);
    }
    else
    {
        s_gyroZOffset = 0;
    }

    s_yawDeg_x10 = 0;
    s_yawValid = 1U;
}

void IMU_ResetYaw(void)
{
    s_yawDeg_x10 = 0;
    s_yawValid = 1U;
}

void IMU_UpdateYaw(uint16_t dt_ms)
{
    int16_t gzRaw;
    int16_t dummyX, dummyY;
    int32_t dps_x100;

    if (!s_yawValid || !s_imuReady) { return; }

    if (!IMU_ReadGyroRaw(&dummyX, &dummyY, &gzRaw)) { return; }

    dps_x100 = (int32_t)(gzRaw - s_gyroZOffset) * 25000L / 32768L;
    s_yawDeg_x10 += (dps_x100 * (int32_t)dt_ms) / 10000L;
}

int32_t IMU_GetYawDeg_x10(void)   { return s_yawDeg_x10; }
uint8_t IMU_IsReady(void)         { return s_imuReady; }
uint8_t IMU_IsHealthy(void)       { return s_imuHealthy; }
uint8_t IMU_GetAddr(void)         { return s_mpuAddr; }
uint8_t IMU_IsAddrValid(void)     { return s_mpuAddrValid; }
uint8_t IMU_GetRecoverCount(void) { return 0U; }
uint32_t IMU_GetLastI2CStatus(void) { return s_lastI2CStatus; }
uint8_t IMU_GetLastErrorStage(void) { return s_lastErrorStage; }

uint8_t IMU_StatusHasIdle(uint32_t status)  { (void)status; return 1U; }
uint8_t IMU_StatusHasBusy(uint32_t status)  { (void)status; return 0U; }
uint8_t IMU_StatusHasError(uint32_t status) { (void)status; return 0U; }

uint8_t IMU_GetGyroRawZ_x10(int16_t *rawZ_x10, int16_t *dps_x10)
{
    int16_t gzRaw;
    int16_t dummyX, dummyY;

    if (rawZ_x10 == 0 || dps_x10 == 0) { return 0U; }
    if (!IMU_ReadGyroRaw(&dummyX, &dummyY, &gzRaw)) { return 0U; }

    *rawZ_x10 = gzRaw;
    *dps_x10 = (int16_t)((int32_t)(gzRaw - s_gyroZOffset) * 2500L / 32768L);
    return 1U;
}
