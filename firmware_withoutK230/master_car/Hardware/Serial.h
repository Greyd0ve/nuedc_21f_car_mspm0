#ifndef __SERIAL_H
#define __SERIAL_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

void Serial_Init(void);

void Serial_SendByte(uint8_t byte);
void Serial_SendArray(const uint8_t *array, uint16_t length);
void Serial_SendString(const char *string);
void Serial_SendNumber(uint32_t number, uint8_t length);
void Serial_Printf(const char *format, ...);

uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

uint8_t Serial_ReadByte(uint8_t *byte);

uint32_t Serial_GetRxOverflowCount(void);
uint16_t Serial_GetRxPendingCount(void);
uint16_t Serial_GetRxHighWaterMark(void);

uint32_t Serial_GetIrqCount(void);
uint32_t Serial_GetRxIrqCount(void);
uint32_t Serial_GetOtherIrqCount(void);
uint32_t Serial_GetRxByteCount(void);
uint32_t Serial_GetInitDiscardCount(void);
uint32_t Serial_GetLastInterruptIndex(void);

#endif
