#include "RodEncoder.h"
#include "Board_Config.h"
#include "cmsis_compiler.h"
#include <stdint.h>

#define ROD_ENCODER_PIN_MASK (ROD_ENCODER_A_PIN | ROD_ENCODER_B_PIN)

static volatile int32_t s_count = 0;
static volatile uint32_t s_badTransitionCount = 0U;
static volatile uint8_t s_stateAB = 0U;

static uint8_t RodEncoder_ReadStateAB(void)
{
    uint32_t pins = DL_GPIO_readPins(ROD_ENCODER_A_PORT, ROD_ENCODER_PIN_MASK);
    uint8_t a = ((pins & ROD_ENCODER_A_PIN) != 0U) ? 1U : 0U;
    uint8_t b = ((pins & ROD_ENCODER_B_PIN) != 0U) ? 1U : 0U;

    return (uint8_t)((a << 1U) | b);
}

void RodEncoder_Init(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    DL_GPIO_disableInterrupt(ROD_ENCODER_A_PORT, ROD_ENCODER_PIN_MASK);
    s_count = 0;
    s_badTransitionCount = 0U;
    s_stateAB = RodEncoder_ReadStateAB();
    DL_GPIO_clearInterruptStatus(ROD_ENCODER_A_PORT, ROD_ENCODER_PIN_MASK);
    NVIC_ClearPendingIRQ(ROD_ENCODER_IRQN);
    DL_GPIO_enableInterrupt(ROD_ENCODER_A_PORT, ROD_ENCODER_PIN_MASK);
    NVIC_EnableIRQ(ROD_ENCODER_IRQN);

    if (primask == 0U)
    {
        __enable_irq();
    }
}

void RodEncoder_Reset(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_count = 0;
    s_badTransitionCount = 0U;
    s_stateAB = RodEncoder_ReadStateAB();

    if (primask == 0U)
    {
        __enable_irq();
    }
}

void RodEncoder_ServiceISR(void)
{
    uint32_t status;
    uint8_t stateNow;
    int8_t delta;

    status = DL_GPIO_getEnabledInterruptStatus(ROD_ENCODER_A_PORT,
                                                ROD_ENCODER_PIN_MASK);
    if (status == 0U)
    {
        return;
    }
    DL_GPIO_clearInterruptStatus(ROD_ENCODER_A_PORT, status);

    stateNow = RodEncoder_ReadStateAB();
    switch ((s_stateAB << 2U) | stateNow)
    {
    case 0x0: case 0x5: case 0xAU: case 0xFU:
        delta = 0;
        break;
    case 0x1: case 0x7: case 0x8: case 0xEU:
        delta = 1;
        break;
    case 0x2: case 0x4: case 0xBU: case 0xDU:
        delta = -1;
        break;
    default:
        delta = 0;
        s_badTransitionCount++;
        break;
    }

    s_count += (int32_t)delta;
    s_stateAB = stateNow;
}

int32_t RodEncoder_GetCount(void)
{
    RodEncoderSnapshot_t snapshot;

    RodEncoder_GetSnapshot(&snapshot);
    return snapshot.count;
}

void RodEncoder_GetSnapshot(RodEncoderSnapshot_t *snapshot)
{
    uint32_t primask;

    if (snapshot == 0)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->count = s_count;
    snapshot->badTransitionCount = s_badTransitionCount;
    snapshot->stateAB = s_stateAB;

    if (primask == 0U)
    {
        __enable_irq();
    }
}
