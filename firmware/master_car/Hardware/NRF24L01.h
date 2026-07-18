#ifndef __NRF24L01_H
#define __NRF24L01_H

#include <stdint.h>

#define NRF24L01_RXMODE  0
#define NRF24L01_TXMODE  1

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
