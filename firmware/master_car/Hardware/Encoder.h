#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

/* 清空待处理增量后，使能编码器 GPIO 中断。 */
void Encoder_Init(void);

/* 返回并清零对应车轮累计的有符号 A 相边沿计数。 */
int16_t Encoder_GetLeftDelta(void);
int16_t Encoder_GetRightDelta(void);

/* 清空左右编码器待处理增量累计值。 */
void Encoder_ClearAll(void);

/*
 * Diagnostic accessors return real values only when ENCODER_DIAG_ENABLE = 1.
 * When disabled, they return 0 and debug print functions report diag disabled.
 */
uint32_t Encoder_GetRightIsrCount(void);
uint32_t Encoder_GetRightSameAIgnored(void);
uint32_t Encoder_GetRightStatusCount(void);
int32_t Encoder_GetRightLastRawDeltaBeforeLimit(void);
uint32_t Encoder_GetRightLimitHitCount(void);
uint32_t Encoder_GetRightGetDeltaCount(void);
uint32_t Encoder_GetRightNonZeroGetCount(void);
int32_t Encoder_GetRightMaxRawDelta(void);

void Encoder_DebugPrintDirectNoPrintf(const char *tag);
void Encoder_DebugPrintGetterNoPrintf(const char *tag);

#endif
