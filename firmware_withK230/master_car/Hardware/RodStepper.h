#ifndef __ROD_STEPPER_H
#define __ROD_STEPPER_H

#include <stdint.h>

typedef enum
{
    ROD_STEPPER_DIR_NEGATIVE = -1,
    ROD_STEPPER_DIR_POSITIVE = 1
} RodStepperDirection_t;

void RodStepper_Init(void);
uint8_t RodStepper_MovePulses(RodStepperDirection_t direction,
                              uint32_t pulses,
                              uint32_t frequencyHz);
/* Start or update a continuous STEP output.  Use RodStepper_Stop() for zero. */
uint8_t RodStepper_SetVelocity(RodStepperDirection_t direction,
                               uint32_t frequencyHz);
void RodStepper_Stop(void);
void RodStepper_Tick1ms(void);
uint8_t RodStepper_IsBusy(void);
uint32_t RodStepper_GetRemainingPulses(void);
int32_t RodStepper_GetSignedCommandPulseTotal(void);
uint32_t RodStepper_GetCompletedPulseCount(void);
uint8_t RodStepper_IsEnabled(void);
RodStepperDirection_t RodStepper_GetDirection(void);
uint8_t RodStepper_TakeCompletionEvent(void);

#endif
