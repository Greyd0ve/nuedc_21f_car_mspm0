#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

#define SERVO_CHANNEL_COUNT 4U

void Servo_Init(void);
void Servo_DisableAll(void);
void Servo_SetPulseUs(uint8_t channel, uint16_t pulseUs);
void Servo_SetAngle(uint8_t channel, int16_t angleDeg);

#endif
