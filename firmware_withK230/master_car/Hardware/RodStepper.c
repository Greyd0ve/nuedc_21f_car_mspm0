#include "RodStepper.h"
#include "Board_Config.h"
#include "cmsis_compiler.h"
#include <stdint.h>

static volatile uint32_t s_remainingPulses = 0U;
static volatile uint32_t s_completedPulseCount = 0U;
static volatile int32_t s_signedCommandPulseTotal = 0;
static volatile uint32_t s_periodCounts = 2500U;
static volatile uint8_t s_busy = 0U;
static volatile uint8_t s_completionEvent = 0U;
static volatile RodStepperDirection_t s_direction = ROD_STEPPER_DIR_NEGATIVE;

static void RodStepper_ForceStepLow(void)
{
    DL_TimerG_setCaptureCompareValue(
        ROD_STEP_TIMER_INST, s_periodCounts, ROD_STEP_CC_INDEX);
}

static int32_t RodStepper_SaturatingAddSigned(int32_t value, int32_t delta)
{
    if (delta > 0 && value > (INT32_MAX - delta))
    {
        return INT32_MAX;
    }
    if (delta < 0 && value < (INT32_MIN - delta))
    {
        return INT32_MIN;
    }
    return value + delta;
}

void RodStepper_Init(void)
{
    DL_TimerG_stopCounter(ROD_STEP_TIMER_INST);
    s_periodCounts = 2500U;
    RodStepper_ForceStepLow();

    DL_GPIO_clearPins(ROD_DIR_PORT, ROD_DIR_PIN);
    DL_TimerG_clearInterruptStatus(ROD_STEP_TIMER_INST,
                                   DL_TIMERG_INTERRUPT_ZERO_EVENT);

    s_remainingPulses = 0U;
    s_completedPulseCount = 0U;
    s_signedCommandPulseTotal = 0;
    s_busy = 0U;
    s_completionEvent = 0U;
    s_direction = ROD_STEPPER_DIR_NEGATIVE;

    /* PB21 is active-low EN and is intentionally never driven high. */
    DL_GPIO_clearPins(ROD_EN_PORT, ROD_EN_PIN);

    NVIC_ClearPendingIRQ(ROD_STEP_TIMER_IRQN);
    NVIC_EnableIRQ(ROD_STEP_TIMER_IRQN);
}

uint8_t RodStepper_MovePulses(RodStepperDirection_t direction,
                              uint32_t pulses,
                              uint32_t frequencyHz)
{
    uint32_t periodCounts;
    uint32_t compareCounts;

    periodCounts = ROD_STEP_TIMER_CLK_HZ / frequencyHz;
    compareCounts = periodCounts / 2U;

    DL_TimerG_stopCounter(ROD_STEP_TIMER_INST);
    RodStepper_ForceStepLow();

    if (direction == ROD_STEPPER_DIR_POSITIVE)
    {
        DL_GPIO_setPins(ROD_DIR_PORT, ROD_DIR_PIN);
    }
    else
    {
        DL_GPIO_clearPins(ROD_DIR_PORT, ROD_DIR_PIN);
    }

    s_direction = direction;
    s_periodCounts = periodCounts;
    DL_TimerG_setLoadValue(ROD_STEP_TIMER_INST, periodCounts);
    DL_TimerG_setCaptureCompareValue(
        ROD_STEP_TIMER_INST, compareCounts, ROD_STEP_CC_INDEX);
    DL_TimerG_clearInterruptStatus(ROD_STEP_TIMER_INST,
                                   DL_TIMERG_INTERRUPT_ZERO_EVENT);

    s_remainingPulses = pulses;
    s_busy = 1U;
    s_completionEvent = 0U;
    s_signedCommandPulseTotal = RodStepper_SaturatingAddSigned(
        s_signedCommandPulseTotal,
        (direction == ROD_STEPPER_DIR_POSITIVE) ? (int32_t)pulses :
        -(int32_t)pulses);

    DL_TimerG_startCounter(ROD_STEP_TIMER_INST);

    return 1U;
}

void RodStepper_Stop(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    DL_TimerG_stopCounter(ROD_STEP_TIMER_INST);
    RodStepper_ForceStepLow();
    DL_TimerG_clearInterruptStatus(ROD_STEP_TIMER_INST,
                                   DL_TIMERG_INTERRUPT_ZERO_EVENT);

    s_remainingPulses = 0U;
    s_busy = 0U;
    s_completionEvent = 0U;
    /* Do not change PB21: the driver remains constantly enabled. */

    if (primask == 0U)
    {
        __enable_irq();
    }
}

void RodStepper_Tick1ms(void)
{
}

void TIMG7_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(ROD_STEP_TIMER_INST))
    {
    case DL_TIMER_IIDX_ZERO:
        if (s_remainingPulses != 0U)
        {
            s_remainingPulses--;
            s_completedPulseCount++;
            if (s_remainingPulses == 0U)
            {
                DL_TimerG_stopCounter(ROD_STEP_TIMER_INST);
                RodStepper_ForceStepLow();
                s_busy = 0U;
                s_completionEvent = 1U;
            }
        }
        break;

    default:
        break;
    }
}

uint8_t RodStepper_IsBusy(void)
{
    return s_busy;
}

uint32_t RodStepper_GetRemainingPulses(void)
{
    return s_remainingPulses;
}

int32_t RodStepper_GetSignedCommandPulseTotal(void)
{
    return s_signedCommandPulseTotal;
}

uint32_t RodStepper_GetCompletedPulseCount(void)
{
    return s_completedPulseCount;
}

uint8_t RodStepper_IsEnabled(void)
{
    return ((DL_GPIO_readPins(ROD_EN_PORT, ROD_EN_PIN) & ROD_EN_PIN) == 0U) ?
        1U : 0U;
}

RodStepperDirection_t RodStepper_GetDirection(void)
{
    return s_direction;
}

uint8_t RodStepper_TakeCompletionEvent(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t event;

    __disable_irq();
    event = s_completionEvent;
    s_completionEvent = 0U;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return event;
}
