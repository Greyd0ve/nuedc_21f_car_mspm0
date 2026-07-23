#ifndef __STEPPER_ENCODER_H
#define __STEPPER_ENCODER_H

#include <stdint.h>

typedef struct {
    int32_t  xCount;
    int32_t  yCount;
    uint32_t xBad;
    uint32_t yBad;
} StepperEncoderSnapshot_t;

void StepperEncoder_Init(void);
void StepperEncoder_ServiceISR(void);

void StepperEncoder_ResetCounts(void);
void StepperEncoder_GetSnapshot(StepperEncoderSnapshot_t *snapshot);

#endif
