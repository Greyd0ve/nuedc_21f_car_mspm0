#ifndef __IR8_H
#define __IR8_H

#include <stdint.h>

#define IR8_CHANNELS 8U

/* Direct eight-channel infrared line-sensor driver. */
void IR8_Init(void);
uint8_t IR8_ReadChannel(uint8_t channel);
void IR8_ReadAll(uint8_t raw[IR8_CHANNELS]);

#endif
