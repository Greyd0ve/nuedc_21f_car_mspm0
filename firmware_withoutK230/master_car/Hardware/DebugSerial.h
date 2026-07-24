#ifndef __DEBUG_SERIAL_H
#define __DEBUG_SERIAL_H

#include <stdint.h>

void DebugSerial_Init(void);
void DebugSerial_SendByte(uint8_t byte);
void DebugSerial_SendString(const char *str);
void DebugSerial_Printf(const char *format, ...);
uint8_t DebugSerial_ReadByte(uint8_t *byte);
uint32_t DebugSerial_GetRxOverflowCount(void);
uint32_t DebugSerial_GetTxOverflowCount(void);
uint16_t DebugSerial_GetTxPendingCount(void);
uint16_t DebugSerial_GetTxHighWaterMark(void);
void DebugSerial_Task10ms(void);

#endif
