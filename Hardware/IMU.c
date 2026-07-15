#include "IMU.h"
#include "Board_Config.h"

#define MPU6050_REG_WHO_AM_I     0x75U
#define MPU6050_REG_PWR_MGMT_1   0x6BU
#define MPU6050_REG_SMPLRT_DIV   0x19U
#define MPU6050_REG_CONFIG       0x1AU
#define MPU6050_REG_GYRO_CONFIG  0x1BU
#define MPU6050_REG_GYRO_XOUT_H  0x43U

#define MPU6050_I2C_TIMEOUT      100000U

#define MPU6050_GYRO_FS_SEL      0U
#define MPU6050_GYRO_LSB_PER_DPS 131.0f

#define MPU6050_DLPF_CFG         2U

#define IMU_ERROR_NONE                 0U
#define IMU_ERROR_WAIT_IDLE            1U
#define IMU_ERROR_TX_FIFO_FILL         2U
#define IMU_ERROR_TX_DONE              3U
#define IMU_ERROR_RX_DONE              4U
#define IMU_ERROR_RX_FIFO_EMPTY        5U
#define IMU_ERROR_WHO_MISMATCH         6U
#define IMU_ERROR_WRITE_REG            7U
#define IMU_ERROR_IDLE_AFTER_RECOVER   8U

static uint8_t  s_imuReady       = 0U;
static uint8_t  s_imuHealthy     = 0U;
static int16_t  s_gyroZOffset    = 0;
static int32_t  s_yawDeg_x10     = 0;
static uint8_t  s_yawValid       = 0U;
static uint8_t  s_mpuAddr        = 0x68U;
static uint8_t  s_mpuAddrValid   = 0U;
static uint32_t s_lastI2CStatus  = 0U;
static uint8_t  s_lastErrorStage = IMU_ERROR_NONE;

static void IMU_I2CRecover(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT;

    DL_I2C_resetControllerTransfer(MPU6050_I2C_INST);
    DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);
    DL_I2C_flushControllerRXFIFO(MPU6050_I2C_INST);

    DL_I2C_clearInterruptStatus(MPU6050_I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
        DL_I2C_INTERRUPT_CONTROLLER_NACK |
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST |
        DL_I2C_INTERRUPT_CONTROLLER_START |
        DL_I2C_INTERRUPT_CONTROLLER_STOP);

    while (timeout > 0U)
    {
        s_lastI2CStatus = DL_I2C_getControllerStatus(MPU6050_I2C_INST);
        if ((s_lastI2CStatus & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U)
        {
            break;
        }
        timeout--;
    }
}

static uint8_t IMU_WaitIdle(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT;

    while (timeout > 0U)
    {
        s_lastI2CStatus = DL_I2C_getControllerStatus(MPU6050_I2C_INST);
        if ((s_lastI2CStatus & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)
        {
            s_lastErrorStage = IMU_ERROR_WAIT_IDLE;
            return 0U;
        }
        if ((s_lastI2CStatus & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U)
        {
            return 1U;
        }
        timeout--;
    }

    s_lastErrorStage = IMU_ERROR_WAIT_IDLE;
    return 0U;
}

static uint8_t IMU_WaitDone(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT;

    while (timeout > 0U)
    {
        s_lastI2CStatus = DL_I2C_getControllerStatus(MPU6050_I2C_INST);
        if ((s_lastI2CStatus & DL_I2C_CONTROLLER_STATUS_BUSY) == 0U)
        {
            return (uint8_t)(((s_lastI2CStatus & DL_I2C_CONTROLLER_STATUS_ERROR) == 0U) ? 1U : 0U);
        }
        timeout--;
    }

    return 0U;
}

static uint8_t IMU_EnsureIdle(void)
{
    if (IMU_WaitIdle())
    {
        return 1U;
    }

    IMU_I2CRecover();

    if (!IMU_WaitIdle())
    {
        s_lastErrorStage = IMU_ERROR_IDLE_AFTER_RECOVER;
        return 0U;
    }

    return 1U;
}

static uint8_t IMU_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2];
    uint16_t written;

    if (!IMU_EnsureIdle())
    {
        return 0U;
    }

    buf[0] = reg;
    buf[1] = value;

    DL_I2C_resetControllerTransfer(MPU6050_I2C_INST);
    written = DL_I2C_fillControllerTXFIFO(MPU6050_I2C_INST, buf, 2U);
    if (written != 2U)
    {
        s_lastErrorStage = IMU_ERROR_TX_FIFO_FILL;
        return 0U;
    }
    DL_I2C_startControllerTransfer(MPU6050_I2C_INST, s_mpuAddr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2U);
    if (!IMU_WaitDone())
    {
        s_lastErrorStage = IMU_ERROR_WRITE_REG;
        IMU_I2CRecover();
        return 0U;
    }
    return 1U;
}

static uint8_t IMU_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint16_t written;
    uint8_t i;

    if (data == 0 || len == 0U)
    {
        return 0U;
    }

    if (!IMU_EnsureIdle())
    {
        return 0U;
    }

    DL_I2C_resetControllerTransfer(MPU6050_I2C_INST);
    written = DL_I2C_fillControllerTXFIFO(MPU6050_I2C_INST, &reg, 1U);
    if (written != 1U)
    {
        s_lastErrorStage = IMU_ERROR_TX_FIFO_FILL;
        return 0U;
    }
    DL_I2C_startControllerTransfer(MPU6050_I2C_INST, s_mpuAddr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);
    if (!IMU_WaitDone())
    {
        s_lastErrorStage = IMU_ERROR_TX_DONE;
        IMU_I2CRecover();
        return 0U;
    }

    DL_I2C_startFlushControllerRXFIFO(MPU6050_I2C_INST);
    DL_I2C_stopFlushControllerRXFIFO(MPU6050_I2C_INST);
    DL_I2C_startControllerTransfer(MPU6050_I2C_INST, s_mpuAddr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    if (!IMU_WaitDone())
    {
        s_lastErrorStage = IMU_ERROR_RX_DONE;
        IMU_I2CRecover();
        return 0U;
    }

    for (i = 0; i < len; i++)
    {
        if (DL_I2C_isControllerRXFIFOEmpty(MPU6050_I2C_INST))
        {
            s_lastErrorStage = IMU_ERROR_RX_FIFO_EMPTY;
            return 0U;
        }
        data[i] = DL_I2C_receiveControllerData(MPU6050_I2C_INST);
    }

    return 1U;
}

uint8_t IMU_ProbeAddressAck(uint8_t addr)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT;
    uint32_t status;

    if (!IMU_EnsureIdle())
    {
        return 0U;
    }

    DL_I2C_resetControllerTransfer(MPU6050_I2C_INST);
    DL_I2C_flushControllerRXFIFO(MPU6050_I2C_INST);

    DL_I2C_startControllerTransfer(MPU6050_I2C_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1U);

    while (timeout > 0U)
    {
        status = DL_I2C_getControllerStatus(MPU6050_I2C_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_BUSY) == 0U)
        {
            break;
        }
        timeout--;
    }

    s_lastI2CStatus = status;

    if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)
    {
        IMU_I2CRecover();
        return 0U;
    }

    if (!DL_I2C_isControllerRXFIFOEmpty(MPU6050_I2C_INST))
    {
        (void)DL_I2C_receiveControllerData(MPU6050_I2C_INST);
    }

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
    if (foundAddr == 0)
    {
        return 0U;
    }

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

    s_imuReady = 0U;
    s_imuHealthy = 0U;
    s_gyroZOffset = 0;
    s_yawDeg_x10 = 0;
    s_yawValid = 0U;
    s_mpuAddr = 0x68U;
    s_mpuAddrValid = 0U;
    s_lastI2CStatus = 0U;
    s_lastErrorStage = IMU_ERROR_NONE;

    if (!IMU_ReadWhoAmI(&who) || who != MPU6050_WHO_AM_I_VAL)
    {
        if (!IMU_Scan(&foundAddr))
        {
            return;
        }
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

    if (!IMU_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00U))
    {
        return;
    }

    {
        uint32_t delay = 200000U;
        while (delay > 0U) { delay--; }
    }

    if (!IMU_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x00U))
    {
        return;
    }

    if (!IMU_WriteReg(MPU6050_REG_CONFIG, MPU6050_DLPF_CFG))
    {
        return;
    }

    if (!IMU_WriteReg(MPU6050_REG_GYRO_CONFIG, (MPU6050_GYRO_FS_SEL << 3U)))
    {
        return;
    }

    s_imuReady = 1U;
    s_imuHealthy = 1U;
}

uint8_t IMU_ReadWhoAmI(uint8_t *whoAmI)
{
    uint8_t data;

    if (whoAmI == 0)
    {
        return 0U;
    }

    *whoAmI = 0U;

    if (!IMU_ReadRegs(MPU6050_REG_WHO_AM_I, &data, 1U))
    {
        return 0U;
    }

    *whoAmI = data;
    return 1U;
}

uint8_t IMU_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];

    if (gx == 0 || gy == 0 || gz == 0)
    {
        return 0U;
    }
    if (!s_imuReady)
    {
        return 0U;
    }

    if (!IMU_ReadRegs(MPU6050_REG_GYRO_XOUT_H, buf, 6U))
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

    if (samples == 0U)
    {
        samples = 200U;
    }

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

    if (!s_yawValid || !s_imuReady)
    {
        return;
    }

    if (!IMU_ReadGyroRaw(&dummyX, &dummyY, &gzRaw))
    {
        return;
    }

    dps_x100 = (int32_t)(gzRaw - s_gyroZOffset) * 25000L / 32768L;

    s_yawDeg_x10 += (dps_x100 * (int32_t)dt_ms) / 10000L;
}

int32_t IMU_GetYawDeg_x10(void)
{
    return s_yawDeg_x10;
}

uint8_t IMU_IsReady(void)
{
    return s_imuReady;
}

uint8_t IMU_IsHealthy(void)
{
    return s_imuHealthy;
}

uint8_t IMU_GetGyroRawZ_x10(int16_t *rawZ_x10, int16_t *dps_x10)
{
    int16_t gzRaw;
    int16_t dummyX, dummyY;

    if (rawZ_x10 == 0 || dps_x10 == 0)
    {
        return 0U;
    }

    if (!IMU_ReadGyroRaw(&dummyX, &dummyY, &gzRaw))
    {
        return 0U;
    }

    *rawZ_x10 = gzRaw;
    *dps_x10 = (int16_t)((int32_t)(gzRaw - s_gyroZOffset) * 2500L / 32768L);

    return 1U;
}

uint32_t IMU_GetLastI2CStatus(void)
{
    return s_lastI2CStatus;
}

uint8_t IMU_GetLastErrorStage(void)
{
    return s_lastErrorStage;
}

uint8_t IMU_GetAddr(void)
{
    return s_mpuAddr;
}

uint8_t IMU_IsAddrValid(void)
{
    return s_mpuAddrValid;
}
