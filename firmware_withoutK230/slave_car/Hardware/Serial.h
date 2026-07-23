#ifndef __SERIAL_H
#define __SERIAL_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* 初始化 UART RX 中断，并清空软件接收环形缓冲区。 */
void Serial_Init(void);

/* 阻塞式发送辅助函数，适用于前台短帧。 */
void Serial_SendByte(uint8_t byte);
void Serial_SendArray(const uint8_t *array, uint16_t length);
void Serial_SendString(const char *string);
void Serial_SendNumber(uint32_t number, uint8_t length);
void Serial_Printf(const char *format, ...);

/* 兼容旧接口；协议解析优先使用 Serial_ReadByte()。 */
uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

/* 从 ISR 填充的 RX 环形缓冲区非阻塞读取 1 字节。 */
uint8_t Serial_ReadByte(uint8_t *byte);

/* RX 环形缓冲区满时丢弃的字节累计数。 */
uint32_t Serial_GetRxOverflowCount(void);

#endif

