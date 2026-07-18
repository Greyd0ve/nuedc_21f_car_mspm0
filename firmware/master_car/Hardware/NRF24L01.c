#include "NRF24L01.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

#define NRF_CE_PORT   GPIOB
#define NRF_CE_PIN    DL_GPIO_PIN_20

#define NRF_CSN_PORT  GPIOA
#define NRF_CSN_PIN   DL_GPIO_PIN_8

#define NRF_SCK_PORT  GPIOA
#define NRF_SCK_PIN   DL_GPIO_PIN_11

#define NRF_MOSI_PORT GPIOA
#define NRF_MOSI_PIN  DL_GPIO_PIN_9

#define NRF_MISO_PORT GPIOA
#define NRF_MISO_PIN  DL_GPIO_PIN_10

#define NRF_IRQ_PORT  GPIOB
#define NRF_IRQ_PIN   DL_GPIO_PIN_9

#define NRF_CE_LOW()   DL_GPIO_clearPins(NRF_CE_PORT, NRF_CE_PIN)
#define NRF_CE_HIGH()  DL_GPIO_setPins(NRF_CE_PORT, NRF_CE_PIN)

#define NRF_CSN_LOW()  DL_GPIO_clearPins(NRF_CSN_PORT, NRF_CSN_PIN)
#define NRF_CSN_HIGH() DL_GPIO_setPins(NRF_CSN_PORT, NRF_CSN_PIN)

#define NRF_SCK_LOW()  DL_GPIO_clearPins(NRF_SCK_PORT, NRF_SCK_PIN)
#define NRF_SCK_HIGH() DL_GPIO_setPins(NRF_SCK_PORT, NRF_SCK_PIN)

#define NRF_MOSI_LOW() DL_GPIO_clearPins(NRF_MOSI_PORT, NRF_MOSI_PIN)
#define NRF_MOSI_HIGH() DL_GPIO_setPins(NRF_MOSI_PORT, NRF_MOSI_PIN)

#define NRF_MISO_READ() (DL_GPIO_readPins(NRF_MISO_PORT, NRF_MISO_PIN) ? 1U : 0U)

#define NRF_IRQ_READ()  (DL_GPIO_readPins(NRF_IRQ_PORT, NRF_IRQ_PIN) == 0U)

/* NRF24L01 register definitions */
#define NRF_REG_CONFIG      0x00
#define NRF_REG_EN_AA       0x01
#define NRF_REG_EN_RXADDR   0x02
#define NRF_REG_SETUP_AW    0x03
#define NRF_REG_SETUP_RETR  0x04
#define NRF_REG_RF_CH       0x05
#define NRF_REG_RF_SETUP    0x06
#define NRF_REG_STATUS      0x07
#define NRF_REG_RX_ADDR_P0  0x0A
#define NRF_REG_TX_ADDR     0x10
#define NRF_REG_RX_PW_P0    0x11
#define NRF_REG_FIFO_STATUS 0x17
#define NRF_REG_DYNPD       0x1C
#define NRF_REG_FEATURE     0x1D

#define NRF_CMD_R_REGISTER    0x00
#define NRF_CMD_W_REGISTER    0x20
#define NRF_CMD_R_RX_PAYLOAD  0x61
#define NRF_CMD_W_TX_PAYLOAD  0xA0
#define NRF_CMD_FLUSH_TX      0xE1
#define NRF_CMD_FLUSH_RX      0xE2

#define NRF_STATUS_RX_DR   0x40
#define NRF_STATUS_TX_DS   0x20
#define NRF_STATUS_MAX_RT  0x10

#define NRF_PAYLOAD_WIDTH  32

static void NRF_SPI_Delay(void)
{
    volatile uint8_t i;
    for (i = 0U; i < 2U; i++) { }
}

static uint8_t NRF_SPI_RW(uint8_t byte)
{
    uint8_t i;
    uint8_t rx = 0U;

    for (i = 0U; i < 8U; i++)
    {
        if (byte & 0x80U) NRF_MOSI_HIGH(); else NRF_MOSI_LOW();
        byte <<= 1U;

        NRF_SPI_Delay();
        NRF_SCK_HIGH();

        rx <<= 1U;
        if (NRF_MISO_READ()) rx |= 1U;

        NRF_SPI_Delay();
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
    for (i = 0U; i < len; i++)
    {
        buf[i] = NRF_SPI_RW(0xFFU);
    }
    NRF_CSN_HIGH();
    return status;
}

uint8_t NRF24L01_WriteBuf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t status;
    uint8_t i;
    NRF_CSN_LOW();
    status = NRF_SPI_RW((uint8_t)(NRF_CMD_W_REGISTER | (reg & 0x1FU)));
    for (i = 0U; i < len; i++)
    {
        NRF_SPI_RW(buf[i]);
    }
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
    NRF24L01_WriteReg(NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
}

void NRF24L01_Init(void)
{
    NRF_CE_LOW();
    NRF_CSN_HIGH();

    DL_GPIO_initDigitalOutput(NRF_CE_PIN);
    DL_GPIO_initDigitalOutput(NRF_CSN_PIN);
    DL_GPIO_initDigitalOutput(NRF_SCK_PIN);
    DL_GPIO_initDigitalOutput(NRF_MOSI_PIN);
    DL_GPIO_initDigitalInput(NRF_MISO_PIN);

    NRF_SCK_LOW();
    NRF_MOSI_LOW();

    NRF_CE_LOW();

    NRF24L01_WriteReg(NRF_REG_CONFIG, 0x08);
    NRF24L01_WriteReg(NRF_REG_EN_AA, 0x01);
    NRF24L01_WriteReg(NRF_REG_EN_RXADDR, 0x01);
    NRF24L01_WriteReg(NRF_REG_SETUP_AW, 0x03);
    NRF24L01_WriteReg(NRF_REG_SETUP_RETR, 0x00);
    NRF24L01_WriteReg(NRF_REG_RF_CH, 40U);
    NRF24L01_WriteReg(NRF_REG_RF_SETUP, 0x07);
    NRF24L01_WriteReg(NRF_REG_RX_PW_P0, NRF_PAYLOAD_WIDTH);
    NRF24L01_WriteReg(NRF_REG_DYNPD, 0x00);
    NRF24L01_WriteReg(NRF_REG_FEATURE, 0x00);

    NRF24L01_FlushTX();
    NRF24L01_FlushRX();
    NRF24L01_ClearIRQFlags();

    NRF_CE_LOW();
}

uint8_t NRF24L01_Check(void)
{
    uint8_t val = NRF24L01_ReadReg(NRF_REG_CONFIG);
    return (val == 0x08U) ? 1U : 0U;
}

void NRF24L01_RX_Mode(void)
{
    NRF_CE_LOW();
    NRF24L01_WriteReg(NRF_REG_CONFIG, 0x0B);
    NRF_CE_HIGH();
}

void NRF24L01_TX_Mode(void)
{
    NRF_CE_LOW();
    NRF24L01_WriteReg(NRF_REG_CONFIG, 0x0A);
    NRF_CE_HIGH();
}

uint8_t NRF24L01_SendPacket(const uint8_t *buf, uint8_t len)
{
    uint8_t status;
    uint8_t i;

    NRF_CE_LOW();
    NRF24L01_FlushTX();
    NRF24L01_ClearIRQFlags();

    NRF_CSN_LOW();
    NRF_SPI_RW(NRF_CMD_W_TX_PAYLOAD);
    for (i = 0U; i < len && i < NRF_PAYLOAD_WIDTH; i++)
    {
        NRF_SPI_RW(buf[i]);
    }
    NRF_CSN_HIGH();

    NRF_CE_HIGH();

    for (i = 0U; i < 100U; i++)
    {
        NRF_SPI_Delay();
    }

    status = NRF24L01_ReadReg(NRF_REG_STATUS);

    if (status & NRF_STATUS_TX_DS)
    {
        NRF24L01_ClearIRQFlags();
        NRF_CE_LOW();
        return 1U;
    }
    if (status & NRF_STATUS_MAX_RT)
    {
        NRF24L01_ClearIRQFlags();
        NRF24L01_FlushTX();
        NRF_CE_LOW();
        return 0U;
    }

    NRF_CE_LOW();
    return 0U;
}

uint8_t NRF24L01_ReceivePacket(uint8_t *buf, uint8_t len)
{
    uint8_t status;
    uint8_t i;

    status = NRF24L01_ReadReg(NRF_REG_STATUS);

    if (!(status & NRF_STATUS_RX_DR))
    {
        return 0U;
    }

    NRF_CSN_LOW();
    NRF_SPI_RW(NRF_CMD_R_RX_PAYLOAD);
    for (i = 0U; i < len && i < NRF_PAYLOAD_WIDTH; i++)
    {
        buf[i] = NRF_SPI_RW(0xFFU);
    }
    NRF_CSN_HIGH();

    NRF24L01_ClearIRQFlags();
    return 1U;
}
