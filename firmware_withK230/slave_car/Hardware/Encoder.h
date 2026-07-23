#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

#define ENCODER_4X_COUNT_PER_REV 4096U

void Encoder_Init(void);

int16_t Encoder_GetLeftDelta(void);
int16_t Encoder_GetRightDelta(void);

void Encoder_ClearAll(void);

uint32_t Encoder_GetLeftIllegalCount(void);
uint32_t Encoder_GetRightIllegalCount(void);

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
