#include "StepperEncoder.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

#define X_A_PORT  GPIOB
#define X_A_PIN   DL_GPIO_PIN_15
#define X_B_PORT  GPIOB
#define X_B_PIN   DL_GPIO_PIN_16
#define Y_A_PORT  GPIOB
#define Y_A_PIN   DL_GPIO_PIN_0
#define Y_B_PORT  GPIOB
#define Y_B_PIN   DL_GPIO_PIN_18

#define STEPPER_ENC_MASK (X_A_PIN | X_B_PIN | Y_A_PIN | Y_B_PIN)

static int32_t  s_xCount = 0;
static int32_t  s_yCount = 0;
static uint32_t s_xBad   = 0U;
static uint32_t s_yBad   = 0U;

void StepperEncoder_ServiceISR(void)
{
    uint32_t status, pins;
    uint8_t xA, xB, yA, yB;
    static uint8_t s_xPrev = 0U;
    static uint8_t s_yPrev = 0U;
    uint8_t xCurr, yCurr;
    int8_t dx, dy;

    status = DL_GPIO_getEnabledInterruptStatus(GPIOB, STEPPER_ENC_MASK);
    DL_GPIO_clearInterruptStatus(GPIOB, status);

    if (!(status & STEPPER_ENC_MASK)) return;

    pins = DL_GPIO_readPins(GPIOB, STEPPER_ENC_MASK);

    xA = (pins & X_A_PIN) ? 1U : 0U;
    xB = (pins & X_B_PIN) ? 1U : 0U;
    yA = (pins & Y_A_PIN) ? 1U : 0U;
    yB = (pins & Y_B_PIN) ? 1U : 0U;

    xCurr = (xA << 1U) | xB;
    yCurr = (yA << 1U) | yB;

    switch ((s_xPrev << 2U) | xCurr)
    {
    case 0x0: case 0x5: case 0xA: case 0xF: dx = 0; break;
    case 0x1: case 0x7: case 0x8: case 0xE: dx = +1; break;
    case 0x2: case 0x4: case 0xB: case 0xD: dx = -1; break;
    default: dx = 0; s_xBad++; break;
    }
    s_xCount += (int32_t)dx;

    switch ((s_yPrev << 2U) | yCurr)
    {
    case 0x0: case 0x5: case 0xA: case 0xF: dy = 0; break;
    case 0x1: case 0x7: case 0x8: case 0xE: dy = +1; break;
    case 0x2: case 0x4: case 0xB: case 0xD: dy = -1; break;
    default: dy = 0; s_yBad++; break;
    }
    s_yCount += (int32_t)dy;

    s_xPrev = xCurr;
    s_yPrev = yCurr;
}

void StepperEncoder_Init(void)
{
    s_xCount = 0;
    s_yCount = 0;
    s_xBad = 0U;
    s_yBad = 0U;

    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM12, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM44, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM32, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM33, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_0_EDGE_RISE_FALL);
    DL_GPIO_setUpperPinsPolarity(GPIOB, DL_GPIO_PIN_18_EDGE_RISE_FALL | DL_GPIO_PIN_15_EDGE_RISE_FALL | DL_GPIO_PIN_16_EDGE_RISE_FALL);

    DL_GPIO_clearInterruptStatus(GPIOB, STEPPER_ENC_MASK);
    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    DL_GPIO_enableInterrupt(GPIOB, STEPPER_ENC_MASK);
}

void StepperEncoder_ResetCounts(void)
{
    s_xCount = 0;
    s_yCount = 0;
    s_xBad = 0U;
    s_yBad = 0U;
}

int32_t StepperEncoder_GetXCount(void)     { return s_xCount; }
int32_t StepperEncoder_GetYCount(void)     { return s_yCount; }
uint32_t StepperEncoder_GetXBadCount(void) { return s_xBad; }
uint32_t StepperEncoder_GetYBadCount(void) { return s_yBad; }
