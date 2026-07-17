#ifndef __BEEPLED_H
#define __BEEPLED_H

#include <stdint.h>

void BeepLed_Init(void);
void BeepLed_AllOn(void);
void BeepLed_AllOff(void);
void BeepLed_AllTurn(void);

void Beep_On(void);
void Beep_Off(void);
void Beep_BeepMs(uint16_t ms);
void BeepLed_Tick1ms(void);

void LED_User_On(void);
void LED_User_Off(void);
void LED_User_Toggle(void);
void LED_User_BlinkTimes(uint8_t times, uint16_t intervalMs);

#endif
