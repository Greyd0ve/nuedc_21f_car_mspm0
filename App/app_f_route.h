#ifndef __APP_F_ROUTE_H
#define __APP_F_ROUTE_H

#include <stdint.h>

void FRoute_Init(void);

void FRoute_SetTargetRoom(uint8_t room);
uint8_t FRoute_GetNextAction(void);

uint8_t FRoute_IsAtRoom(void);
uint8_t FRoute_IsAtPharmacy(void);

#endif
