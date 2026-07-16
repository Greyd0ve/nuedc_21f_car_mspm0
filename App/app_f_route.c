#include "app_f_route.h"

static uint8_t s_targetRoom = 0U;

void FRoute_Init(void)
{
    s_targetRoom = 0U;
}

void FRoute_SetTargetRoom(uint8_t room)
{
    s_targetRoom = room;
}

uint8_t FRoute_GetNextAction(void)
{
    (void)s_targetRoom;
    return 0U;
}

uint8_t FRoute_IsAtRoom(void)
{
    return 0U;
}

uint8_t FRoute_IsAtPharmacy(void)
{
    return 0U;
}
