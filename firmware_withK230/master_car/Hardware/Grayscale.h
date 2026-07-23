#ifndef __GRAYSCALE_H
#define __GRAYSCALE_H

#include <stdint.h>

/* CD4051 多路复用器连接的灰度传感器数量。 */
#define GRAYSCALE_CHANNELS 8U /* 灰度通道数，当前为 8 路 */

/* 初始化 CD4051 地址引脚。 */
void Grayscale_Init(void);

/* 读取一个物理通道 0..7，内部使用多数表决采样。 */
uint8_t Grayscale_ReadChannel(uint8_t channel);

/* 按地址递增顺序读取全部物理通道到 raw[]。 */
void Grayscale_ReadAll(uint8_t raw[GRAYSCALE_CHANNELS]);

/* 单通道读取兼容封装。 */
uint8_t Grayscale_ReadOne(uint8_t channel);

/* 不改变 CD4051 地址，直接读取当前 OUT 引脚电平。 */
uint8_t Grayscale_RawOUT(void);

#endif
