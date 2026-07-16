#ifndef __APP_ROOM_DETECT_H
#define __APP_ROOM_DETECT_H

#include <stdint.h>

void RoomDetect_Init(void);

uint8_t RoomDetect_GetCurrentRoom(void);
uint8_t RoomDetect_IsTargetRoom(uint8_t targetRoom);

#endif
