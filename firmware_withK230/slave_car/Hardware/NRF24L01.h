#ifndef __NRF24L01_H
#define __NRF24L01_H

#include <stdint.h>

#define NRF24L01_RXMODE  0
#define NRF24L01_TXMODE  1

#define NRF_REG_CONFIG       0x00U
#define NRF_REG_EN_AA        0x01U
#define NRF_REG_EN_RXADDR    0x02U
#define NRF_REG_SETUP_AW     0x03U
#define NRF_REG_SETUP_RETR   0x04U
#define NRF_REG_RF_CH        0x05U
#define NRF_REG_RF_SETUP     0x06U
#define NRF_REG_STATUS       0x07U
#define NRF_REG_RX_ADDR_P0   0x0AU
#define NRF_REG_TX_ADDR      0x10U
#define NRF_REG_RX_PW_P0     0x11U
#define NRF_REG_FIFO_STATUS  0x17U
#define NRF_REG_DYNPD        0x1CU
#define NRF_REG_FEATURE      0x1DU

#define NRF_STATUS_RX_DR     0x40U
#define NRF_STATUS_TX_DS     0x20U
#define NRF_STATUS_MAX_RT    0x10U

void NRF24L01_Init(void);
uint8_t NRF24L01_Check(void);

uint8_t NRF24L01_ReadReg(uint8_t reg);
uint8_t NRF24L01_WriteReg(uint8_t reg, uint8_t value);

uint8_t NRF24L01_ReadBuf(uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t NRF24L01_WriteBuf(uint8_t reg, const uint8_t *buf, uint8_t len);

void NRF24L01_RX_Mode(void);
void NRF24L01_TX_Mode(void);

uint8_t NRF24L01_SendPacket(const uint8_t *buf, uint8_t len);
uint8_t NRF24L01_ReceivePacket(uint8_t *buf, uint8_t len);

void NRF24L01_FlushTX(void);
void NRF24L01_FlushRX(void);
void NRF24L01_ClearIRQFlags(void);

void NRF24L01_CE_Set(uint8_t level);
void NRF24L01_CSN_Set(uint8_t level);

#endif
