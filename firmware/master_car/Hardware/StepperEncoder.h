#ifndef __STEPPER_ENCODER_H
#define __STEPPER_ENCODER_H

#include <stdint.h>

void StepperEncoder_Init(void);
void StepperEncoder_ServiceISR(void);

void StepperEncoder_ResetCounts(void);

int32_t StepperEncoder_GetXCount(void);
int32_t StepperEncoder_GetYCount(void);
uint32_t StepperEncoder_GetXBadCount(void);
uint32_t StepperEncoder_GetYBadCount(void);

#endif
