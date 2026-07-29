#ifndef __ROD_ENCODER_H
#define __ROD_ENCODER_H

#include <stdint.h>

typedef struct
{
    int32_t count;
    uint32_t badTransitionCount;
    uint8_t stateAB;
} RodEncoderSnapshot_t;

void RodEncoder_Init(void);
void RodEncoder_Reset(void);
void RodEncoder_ServiceISR(void);
int32_t RodEncoder_GetCount(void);
void RodEncoder_GetSnapshot(RodEncoderSnapshot_t *snapshot);

#endif
