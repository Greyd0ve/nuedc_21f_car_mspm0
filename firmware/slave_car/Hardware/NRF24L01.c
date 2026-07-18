#include "NRF24L01.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

#define NRF_CE_PORT   GPIO_NRF_NRF_CE_PORT
#define NRF_CE_PIN    GPIO_NRF_NRF_CE_PIN

#define NRF_CSN_PORT  GPIO_NRF_NRF_CSN_PORT
#define NRF_CSN_PIN   GPIO_NRF_NRF_CSN_PIN

#define NRF_SCK_PORT  GPIO_NRF_NRF_SCK_PORT
#define NRF_SCK_PIN   GPIO_NRF_NRF_SCK_PIN

#define NRF_MOSI_PORT GPIO_NRF_NRF_MOSI_PORT
#define NRF_MOSI_PIN  GPIO_NRF_NRF_MOSI_PIN

#define NRF_MISO_PORT GPIO_NRF_NRF_MISO_PORT
#define NRF_MISO_PIN  GPIO_NRF_NRF_MISO_PIN

#define NRF_CE_LOW()   DL_GPIO_clearPins(NRF_CE_PORT, NRF_CE_PIN)
#define NRF_CE_HIGH()  DL_GPIO_setPins(NRF_CE_PORT, NRF_CE_PIN)

#define NRF_CSN_LOW()  DL_GPIO_clearPins(NRF_CSN_PORT, NRF_CSN_PIN)
#define NRF_CSN_HIGH() DL_GPIO_setPins(NRF_CSN_PORT, NRF_CSN_PIN)

#define NRF_SCK_LOW()  DL_GPIO_clearPins(NRF_SCK_PORT, NRF_SCK_PIN)
#define NRF_SCK_HIGH() DL_GPIO_setPins(NRF_SCK_PORT, NRF_SCK_PIN)

#define NRF_MOSI_LOW() DL_GPIO_clearPins(NRF_MOSI_PORT, NRF_MOSI_PIN)
#define NRF_MOSI_HIGH() DL_GPIO_setPins(NRF_MOSI_PORT, NRF_MOSI_PIN)

#define NRF_MISO_READ() (DL_GPIO_readPins(NRF_MISO_PORT, NRF_MISO_PIN) ? 1U : 0U)

#define NRF_USE_IRQ 0

#define NRF_CMD_R_REGISTER    0x00U
#define NRF_CMD_W_REGISTER    0x20U
#define NRF_CMD_R_RX_PAYLOAD  0x61U
#define NRF_CMD_W_TX_PAYLOAD  0xA0U
#define NRF_CMD_FLUSH_TX      0xE1U
#define NRF_CMD_FLUSH_RX      0xE2U

#define NRF_FIFO_RX_EMPTY     0x01U
#define NRF_PAYLOAD_WIDTH      32U
#define NRF_TX_TIMEOUT_MS      20U

static void NRF_DelayUs(uint32_t us)
{
    delay_cycles((CPUCLK_FREQ / 1000000U) * us);
}

static uint8_t NRF_SPI_RW(uint8_t byte)
{
    uint8_t i;
    uint8_t rx = 0U;
    for (i = 0U; i < 8U; i++)
    {
        if (byte & 0x80U) NRF_MOSI_HIGH(); else NRF_MOSI_LOW();
        byte <<= 1U;
        NRF_DelayUs(1U);
        NRF_SCK_HIGH();
        rx <<= 1U;
        if (NRF_MISO_READ()) rx |= 1U;
        NRF_DelayUs(1U);
        NRF_SCK_LOW();
    }
    return rx;
}

void NRF24L01_CE_Set(uint8_t level)
{
    if (level) NRF_CE_HIGH(); else NRF_CE_LOW();
}

void NRF24L01_CSN_Set(uint8_t level)
{
    if (level) NRF_CSN_HIGH(); else NRF_CSN_LOW();
}

uint8_t NRF24L01_ReadReg(uint8_t reg)
{
    uint8_t val;
    NRF_CSN_LOW();
    NRF_SPI_RW((uint8_t)(NRF_CMD_R_REGISTER | (reg & 0x1FU)));
    val = NRF_SPI_RW(0xFFU);
    NRF_CSN_HIGH();
    return val;
}

uint8_t NRF24L01_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    NRF_CSN_LOW();
    status = NRF_SPI_RW((uint8_t)(NRF_CMD_W_REGISTER | (reg & 0x1FU)));
    NRF_SPI_RW(value);
    NRF_CSN_HIGH();
    return status;
}

uint8_t NRF24L01_ReadBuf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t status;
    uint8_t i;
    NRF_CSN_LOW();
    status = NRF_SPI_RW((uint8_t)(NRF_CMD_R_REGISTER | (reg & 0x1FU)));
    for (i = 0U; i < len; i++) { buf[i] = NRF_SPI_RW(0xFFU); }
    NRF_CSN_HIGH();
    return status;
}

uint8_t NRF24L01_WriteBuf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t status;
    uint8_t i;
    NRF_CSN_LOW();
    status = NRF_SPI_RW((uint8_t)(NRF_CMD_W_REGISTER | (reg & 0x1FU)));
    for (i = 0U; i < len; i++) { NRF_SPI_RW(buf[i]); }
    NRF_CSN_HIGH();
    return status;
}

void NRF24L01_FlushTX(void)
{
    NRF_CSN_LOW();
    NRF_SPI_RW(NRF_CMD_FLUSH_TX);
    NRF_CSN_HIGH();
}

void NRF24L01_FlushRX(void)
{
    NRF_CSN_LOW();
    NRF_SPI_RW(NRF_CMD_FLUSH_RX);
    NRF_CSN_HIGH();
}

void NRF24L01_ClearIRQFlags(void)
{
    uint8_t mask = NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT;
    NRF24L01_WriteReg(NRF_REG_STATUS, mask);
}

uint8_t NRF24L01_Check(void)
{
    uint8_t orig[5];
    uint8_t test[5] = {0xA5, 0x5A, 0xC3, 0x3C, 0x96};
    uint8_t back[5];
    uint8_t i;
    uint8_t ok;

    NRF24L01_ReadBuf(NRF_REG_TX_ADDR, orig, 5);
    NRF24L01_WriteBuf(NRF_REG_TX_ADDR, test, 5);
    NRF24L01_ReadBuf(NRF_REG_TX_ADDR, back, 5);
    NRF24L01_WriteBuf(NRF_REG_TX_ADDR, orig, 5);

    ok = 1U;
    for (i = 0U; i < 5U; i++) { if (back[i] != test[i]) ok = 0U; }
    return ok;
}

void NRF24L01_Init(void)
{
    NRF_CE_LOW();
    NRF_CSN_HIGH();
    NRF_SCK_LOW();
    NRF_MOSI_LOW();
}

void NRF24L01_RX_Mode(void)
{
    NRF_CE_LOW();
    NRF24L01_WriteReg(NRF_REG_CONFIG, 0x0FU);
    NRF24L01_ClearIRQFlags();
    NRF_DelayUs(2000U);
    NRF_CE_HIGH();
}

void NRF24L01_TX_Mode(void)
{
    NRF_CE_LOW();
    NRF24L01_WriteReg(NRF_REG_CONFIG, 0x0EU);
    NRF24L01_ClearIRQFlags();
    NRF_DelayUs(2000U);
}

uint8_t NRF24L01_SendPacket(const uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint8_t status;
    uint32_t timeout;

    NRF_CE_LOW();
    NRF24L01_TX_Mode();
    NRF24L01_FlushTX();
    NRF24L01_ClearIRQFlags();

    NRF_CSN_LOW();
    NRF_SPI_RW(NRF_CMD_W_TX_PAYLOAD);
    for (i = 0U; i < len && i < NRF_PAYLOAD_WIDTH; i++) { NRF_SPI_RW(buf[i]); }
    NRF_CSN_HIGH();

    NRF_CE_HIGH();
    NRF_DelayUs(20U);
    NRF_CE_LOW();

    for (timeout = 0U; timeout < NRF_TX_TIMEOUT_MS; timeout++)
    {
        status = NRF24L01_ReadReg(NRF_REG_STATUS);
        if (status & NRF_STATUS_TX_DS)
        {
            NRF24L01_ClearIRQFlags();
            return 1U;
        }
        if (status & NRF_STATUS_MAX_RT)
        {
            NRF24L01_ClearIRQFlags();
            NRF24L01_FlushTX();
            return 0U;
        }
        NRF_DelayUs(1000U);
    }

    NRF24L01_ClearIRQFlags();
    NRF24L01_FlushTX();
    return 0U;
}

uint8_t NRF24L01_ReceivePacket(uint8_t *buf, uint8_t len)
{
    uint8_t fifo;
    uint8_t i;

    fifo = NRF24L01_ReadReg(NRF_REG_FIFO_STATUS);
    if (fifo & NRF_FIFO_RX_EMPTY) return 0U;

    NRF_CSN_LOW();
    NRF_SPI_RW(NRF_CMD_R_RX_PAYLOAD);
    for (i = 0U; i < len && i < NRF_PAYLOAD_WIDTH; i++) { buf[i] = NRF_SPI_RW(0xFFU); }
    NRF_CSN_HIGH();

    NRF24L01_ClearIRQFlags();
    return 1U;
}
