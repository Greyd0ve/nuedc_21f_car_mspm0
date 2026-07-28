#ifndef __JY61P_SERIAL_H
#define __JY61P_SERIAL_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

void JY61P_Serial_Init(void);

uint8_t JY61P_Serial_ReadByte(uint8_t *byte);

uint32_t JY61P_Serial_GetRxOverflowCount(void);
uint16_t JY61P_Serial_GetRxPendingCount(void);
uint16_t JY61P_Serial_GetRxHighWaterMark(void);

uint32_t JY61P_Serial_GetIrqCount(void);
uint32_t JY61P_Serial_GetRxIrqCount(void);
uint32_t JY61P_Serial_GetOtherIrqCount(void);
uint32_t JY61P_Serial_GetRxByteCount(void);
uint32_t JY61P_Serial_GetInitDiscardCount(void);
uint32_t JY61P_Serial_GetLastInterruptIndex(void);

#endif
