#include "IR8.h"
#include "Board_Config.h"

static uint8_t IR8_ReadPin(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0U) ? 1U : 0U;
}

void IR8_Init(void)
{
    /* All eight inputs are configured by SysConfig before application init. */
}

uint8_t IR8_ReadChannel(uint8_t channel)
{
    switch (channel)
    {
    case 0U: return IR8_ReadPin(IR8_X1_PORT, IR8_X1_PIN);
    case 1U: return IR8_ReadPin(IR8_X2_PORT, IR8_X2_PIN);
    case 2U: return IR8_ReadPin(IR8_X3_PORT, IR8_X3_PIN);
    case 3U: return IR8_ReadPin(IR8_X4_PORT, IR8_X4_PIN);
    case 4U: return IR8_ReadPin(IR8_X5_PORT, IR8_X5_PIN);
    case 5U: return IR8_ReadPin(IR8_X6_PORT, IR8_X6_PIN);
    case 6U: return IR8_ReadPin(IR8_X7_PORT, IR8_X7_PIN);
    case 7U: return IR8_ReadPin(IR8_X8_PORT, IR8_X8_PIN);
    default: return 0U;
    }
}

void IR8_ReadAll(uint8_t raw[IR8_CHANNELS])
{
    uint8_t channel;

    if (raw == 0)
    {
        return;
    }

    for (channel = 0U; channel < IR8_CHANNELS; channel++)
    {
        raw[channel] = IR8_ReadChannel(channel);
    }
}
