#ifndef __APP_LINE_H
#define __APP_LINE_H

#include <stdint.h>

extern volatile int16_t g_lineError;
extern volatile uint8_t g_lineValid;
extern volatile uint8_t g_lineMask;
extern volatile uint8_t g_lineRawMask;
extern volatile uint8_t g_lineBlackCount;
extern volatile uint8_t g_lineBadMaskCount;
extern volatile uint8_t g_lineZeroMaskCount;
extern volatile uint8_t g_lineCornerMaskStableCount;
extern volatile uint16_t g_lineLostMs;

void App_Line_GPIOForceInit(void);
void App_Line_ResetState(void);
void App_Line_Update(void);
float App_Line_CalcTurnCmd(void);

#endif
