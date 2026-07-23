#ifndef __APP_CAR_BASE_H
#define __APP_CAR_BASE_H

#include <stdint.h>

void CarBase_Init(void);
void CarBase_Task10ms(void);
void CarBase_KeyProcess(void);
void CarBase_Task100ms(void);
void CarBase_Task200ms(void);
void CarBase_PromptTick1ms(void);

void CarBase_StopAll(void);
void CarBase_ShowStatus(void);

#endif
