#ifndef __APP_LOAD_DETECT_H
#define __APP_LOAD_DETECT_H

#include <stdint.h>

void LoadDetect_Init(void);

uint8_t LoadDetect_IsLoaded(void);
uint8_t LoadDetect_IsUnloaded(void);

void LoadDetect_SetLoaded(uint8_t loaded);
void LoadDetect_SetUnloaded(uint8_t unloaded);

#endif
