#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

/* 4× quadrature encoder counts per wheel revolution (measured). */
#define ENCODER_4X_COUNT_PER_REV 4096U

void Encoder_Init(void);

int16_t Encoder_GetLeftDelta(void);
int16_t Encoder_GetRightDelta(void);

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
