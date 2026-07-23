#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include <stdint.h>

void StepperMotor_Init(void);

void StepperMotor_EnableLeft(uint8_t enable);
void StepperMotor_EnableRight(uint8_t enable);
void StepperMotor_EnableAll(uint8_t enable);

int32_t StepperMotor_SetLeftTargetFrequency(int32_t frequencyHz);
int32_t StepperMotor_SetRightTargetFrequency(int32_t frequencyHz);
void StepperMotor_SetTargetFrequency(int32_t leftFrequencyHz, int32_t rightFrequencyHz);

int32_t StepperMotor_GetLeftTargetFrequency(void);
int32_t StepperMotor_GetRightTargetFrequency(void);
int32_t StepperMotor_GetLeftCurrentFrequency(void);
int32_t StepperMotor_GetRightCurrentFrequency(void);

uint8_t StepperMotor_IsLeftEnabled(void);
uint8_t StepperMotor_IsRightEnabled(void);

void StepperMotor_StopLeft(void);
void StepperMotor_StopRight(void);
void StepperMotor_StopAll(void);

void StepperMotor_EmergencyStop(void);

void StepperMotor_Task1ms(void);

#endif
