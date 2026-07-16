#include "app_room_detect.h"

static uint8_t s_currentRoom = 0U;

void RoomDetect_Init(void)
{
    s_currentRoom = 0U;
}

uint8_t RoomDetect_GetCurrentRoom(void)
{
    return s_currentRoom;
}

uint8_t RoomDetect_IsTargetRoom(uint8_t targetRoom)
{
    return (s_currentRoom == targetRoom) ? 1U : 0U;
}
