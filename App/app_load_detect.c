#include "app_load_detect.h"

static uint8_t s_loaded = 0U;
static uint8_t s_unloaded = 0U;

void LoadDetect_Init(void)
{
    s_loaded = 0U;
    s_unloaded = 0U;
}

uint8_t LoadDetect_IsLoaded(void)
{
    return s_loaded;
}

uint8_t LoadDetect_IsUnloaded(void)
{
    return s_unloaded;
}

void LoadDetect_SetLoaded(uint8_t loaded)
{
    s_loaded = loaded;
}

void LoadDetect_SetUnloaded(uint8_t unloaded)
{
    s_unloaded = unloaded;
}
