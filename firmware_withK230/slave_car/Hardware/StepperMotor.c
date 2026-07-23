#include "StepperMotor.h"
#include "Board_Config.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * D36A stepper driver constants.
 * Independent TIMG7 / TIMG8 generate STEP_L / STEP_R hardware square waves.
 * Frequency changes happen via timer load/compare update every 1ms.
 */

/* Enable polarity for D36A: 0 = active low, 1 = active high. */
#ifndef STEPPER_EN_ACTIVE_LEVEL
#define STEPPER_EN_ACTIVE_LEVEL 0U
#endif

/* Accel / decel rate per 1ms tick.
 * Smaller = softer ramp, larger = stiffer response. */
#ifndef STEPPER_ACCEL_HZ_PER_TICK
#define STEPPER_ACCEL_HZ_PER_TICK 20
#endif
#ifndef STEPPER_DECEL_HZ_PER_TICK
#define STEPPER_DECEL_HZ_PER_TICK 30
#endif

/* Minimum STEP frequency the timer can produce at BUSCLK / 1 (32 MHz).
 * 32 MHz / 65535 ≈ 488 Hz.  Below this we stop the timer to keep STEP low. */
#define STEPPER_MIN_TIMER_FREQ_HZ 500

/* Maximum frequency limit (software clamp). */
#ifndef STEPPER_MAX_FREQ_HZ
#define STEPPER_MAX_FREQ_HZ 32000
#endif

#define STEPPER_TIMER_L_INST TIMG7
#define STEPPER_TIMER_R_INST TIMG8
#define STEPPER_CC_INDEX     DL_TIMER_CC_0_INDEX

/* PA3 IOMUX for TIMG7 CCP0; PA5 IOMUX for TIMG8 CCP0 (MSPM0G3507). */
#define STEPPER_STEP_L_IOMUX      (IOMUX_PINCM8)
#define STEPPER_STEP_L_IOMUX_FUNC  IOMUX_PINCM8_PF_TIMG7_CCP0
#define STEPPER_STEP_R_IOMUX      (IOMUX_PINCM10)
#define STEPPER_STEP_R_IOMUX_FUNC  IOMUX_PINCM10_PF_TIMG8_CCP0

static volatile int32_t s_leftTargetFreq  = 0;
static volatile int32_t s_rightTargetFreq = 0;
static volatile int32_t s_leftCurFreq     = 0;
static volatile int32_t s_rightCurFreq    = 0;
static volatile uint8_t s_leftEnabled     = 0U;
static volatile uint8_t s_rightEnabled    = 0U;
static volatile uint8_t s_emergencyStop   = 0U;

static void StepperMotor_SetEN(GPIO_Regs *port, uint32_t pin, uint8_t enable)
{
    uint8_t active = (uint8_t)STEPPER_EN_ACTIVE_LEVEL;

    if (enable)
    {
        if (active) { DL_GPIO_setPins(port, pin); }
        else        { DL_GPIO_clearPins(port, pin); }
    }
    else
    {
        if (active) { DL_GPIO_clearPins(port, pin); }
        else        { DL_GPIO_setPins(port, pin); }
    }
}

static void StepperMotor_SetDIR(GPIO_Regs *port, uint32_t pin, int32_t freq)
{
    if (freq >= 0)
    {
        DL_GPIO_setPins(port, pin);
    }
    else
    {
        DL_GPIO_clearPins(port, pin);
    }
}

/*
 * Configure one TIMG channel for 50 % duty PWM and route to the STEP pin.
 * Timer is left STOPPED; load = 0, compare = 0 so STEP stays low.
 */
static void StepperMotor_InitTimerChannel(
    GPTIMER_Regs *timer,
    uint32_t iomux,
    uint32_t iomuxFunc,
    GPIO_Regs *port,
    uint32_t pinMask)
{
    DL_TimerG_ClockConfig clockCfg = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = 0U,
    };
    DL_TimerG_PWMConfig pwmCfg = {
        .pwmMode          = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .period           = 0U,
        .isTimerWithFourCC = false,
        .startTimer       = DL_TIMER_STOP,
    };

    DL_GPIO_initPeripheralOutputFunction(iomux, iomuxFunc);
    DL_GPIO_enableOutput(port, pinMask);

    DL_TimerG_reset(timer);
    DL_TimerG_enablePower(timer);

    DL_TimerG_setClockConfig(timer, &clockCfg);
    DL_TimerG_initPWMMode(timer, &pwmCfg);

    DL_TimerG_setCaptureCompareOutCtl(timer,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(timer,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptureCompareValue(timer, 0U,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCCPDirection(timer, DL_TIMER_CC0_OUTPUT);
    DL_TimerG_enableClock(timer);
}

/*
 * Apply a new frequency to a channel (set load value and CC value).
 * freq == 0  → stop timer (STEP stays low).
 * freq != 0  → start / update timer at 50 % duty.
 */
static void StepperMotor_ApplyFreq(
    GPTIMER_Regs *timer, int32_t freqAbs)
{
    uint32_t period;

    if (freqAbs == 0)
    {
        DL_TimerG_stopCounter(timer);
        DL_TimerG_setCaptureCompareValue(timer, 0U,
            DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
        return;
    }

    if ((uint32_t)freqAbs > (uint32_t)STEPPER_MAX_FREQ_HZ)
    {
        freqAbs = (int32_t)STEPPER_MAX_FREQ_HZ;
    }

    period = 32000000U / (uint32_t)freqAbs;
    if (period < 2U) { period = 2U; }
    if (period > 65535U) { period = 65535U; }

    DL_TimerG_setLoadValue(timer, period - 1U);
    DL_TimerG_setCaptureCompareValue(timer,
        period / 2U, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_startCounter(timer);
}

void StepperMotor_Init(void)
{
    s_leftTargetFreq  = 0;
    s_rightTargetFreq = 0;
    s_leftCurFreq     = 0;
    s_rightCurFreq    = 0;
    s_leftEnabled     = 0U;
    s_rightEnabled    = 0U;
    s_emergencyStop   = 0U;

    /* DIR_L = PB17, DIR_R = PB19 already configured as GPIO output by
     * SYSCFG_DL_GPIO_init() (former TB6612 L_IN1 / L_IN2).
     * Set both low initially. */
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_17 | DL_GPIO_PIN_19);

    /* EN_L = PA16, EN_R = PB24, also already configured as GPIO output.
     * Force both to inactive (disabled) state. */
    StepperMotor_SetEN(GPIOA, DL_GPIO_PIN_16, 0U);
    StepperMotor_SetEN(GPIOB, DL_GPIO_PIN_24, 0U);

    /* Init STEP timer channels.
     * STEP_L = PA3 (TIMG7 CCP0), STEP_R = PA5 (TIMG8 CCP0). */
    StepperMotor_InitTimerChannel(
        STEPPER_TIMER_L_INST,
        STEPPER_STEP_L_IOMUX,
        STEPPER_STEP_L_IOMUX_FUNC,
        GPIOA, DL_GPIO_PIN_3);

    StepperMotor_InitTimerChannel(
        STEPPER_TIMER_R_INST,
        STEPPER_STEP_R_IOMUX,
        STEPPER_STEP_R_IOMUX_FUNC,
        GPIOA, DL_GPIO_PIN_5);
}

void StepperMotor_EnableLeft(uint8_t enable)
{
    s_leftEnabled = enable;
    StepperMotor_SetEN(GPIOA, DL_GPIO_PIN_16, enable);
    if (!enable)
    {
        s_leftTargetFreq = 0;
    }
}

void StepperMotor_EnableRight(uint8_t enable)
{
    s_rightEnabled = enable;
    StepperMotor_SetEN(GPIOB, DL_GPIO_PIN_24, enable);
    if (!enable)
    {
        s_rightTargetFreq = 0;
    }
}

void StepperMotor_EnableAll(uint8_t enable)
{
    StepperMotor_EnableLeft(enable);
    StepperMotor_EnableRight(enable);
}

int32_t StepperMotor_SetLeftTargetFrequency(int32_t frequencyHz)
{
    if (frequencyHz > (int32_t)STEPPER_MAX_FREQ_HZ)
        { frequencyHz = (int32_t)STEPPER_MAX_FREQ_HZ; }
    if (frequencyHz < -(int32_t)STEPPER_MAX_FREQ_HZ)
        { frequencyHz = -(int32_t)STEPPER_MAX_FREQ_HZ; }
    s_leftTargetFreq = frequencyHz;
    return frequencyHz;
}

int32_t StepperMotor_SetRightTargetFrequency(int32_t frequencyHz)
{
    if (frequencyHz > (int32_t)STEPPER_MAX_FREQ_HZ)
        { frequencyHz = (int32_t)STEPPER_MAX_FREQ_HZ; }
    if (frequencyHz < -(int32_t)STEPPER_MAX_FREQ_HZ)
        { frequencyHz = -(int32_t)STEPPER_MAX_FREQ_HZ; }
    s_rightTargetFreq = frequencyHz;
    return frequencyHz;
}

void StepperMotor_SetTargetFrequency(int32_t leftFreqHz, int32_t rightFreqHz)
{
    StepperMotor_SetLeftTargetFrequency(leftFreqHz);
    StepperMotor_SetRightTargetFrequency(rightFreqHz);
}

int32_t StepperMotor_GetLeftTargetFrequency(void)
{
    return s_leftTargetFreq;
}

int32_t StepperMotor_GetRightTargetFrequency(void)
{
    return s_rightTargetFreq;
}

int32_t StepperMotor_GetLeftCurrentFrequency(void)
{
    return s_leftCurFreq;
}

int32_t StepperMotor_GetRightCurrentFrequency(void)
{
    return s_rightCurFreq;
}

uint8_t StepperMotor_IsLeftEnabled(void)
{
    return s_leftEnabled;
}

uint8_t StepperMotor_IsRightEnabled(void)
{
    return s_rightEnabled;
}

void StepperMotor_StopLeft(void)
{
    s_leftTargetFreq = 0;
}

void StepperMotor_StopRight(void)
{
    s_rightTargetFreq = 0;
}

void StepperMotor_StopAll(void)
{
    s_leftTargetFreq = 0;
    s_rightTargetFreq = 0;
}

void StepperMotor_EmergencyStop(void)
{
    s_emergencyStop = 1U;
    s_leftTargetFreq  = 0;
    s_rightTargetFreq = 0;
    s_leftCurFreq     = 0;
    s_rightCurFreq    = 0;

    DL_TimerG_stopCounter(STEPPER_TIMER_L_INST);
    DL_TimerG_setCaptureCompareValue(STEPPER_TIMER_L_INST, 0U,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_stopCounter(STEPPER_TIMER_R_INST);
    DL_TimerG_setCaptureCompareValue(STEPPER_TIMER_R_INST, 0U,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    StepperMotor_SetEN(GPIOA, DL_GPIO_PIN_16, 0U);
    StepperMotor_SetEN(GPIOB, DL_GPIO_PIN_24, 0U);

    s_emergencyStop = 0U;
}

static int32_t StepperMotor_RampOne(
    int32_t target, int32_t current, int32_t accelStep, int32_t decelStep)
{
    int32_t diff;
    int32_t step;
    int32_t absTarget;

    if (target == current) { return target; }

    absTarget = (target >= 0) ? target : -target;
    if (absTarget < (int32_t)STEPPER_MIN_TIMER_FREQ_HZ && target != 0)
    {
        /*
         * Clamp very low non-zero targets to the minimum timer frequency.
         * Below ~500 Hz the 16-bit period overflows; force to 0 so
         * STEP stays low instead of producing a glitch frequency.
         */
        if (target > 0) { target = (int32_t)STEPPER_MIN_TIMER_FREQ_HZ; }
        else            { target = -(int32_t)STEPPER_MIN_TIMER_FREQ_HZ; }
    }

    diff = target - current;
    if (diff > 0)
    {
        step = accelStep;
    }
    else
    {
        step = decelStep;
        diff = -diff;
    }

    if ((int32_t)step >= diff)
    {
        return target;
    }

    if (target > current)
    {
        return current + (int32_t)step;
    }
    else
    {
        return current - (int32_t)step;
    }
}

void StepperMotor_Task1ms(void)
{
    int32_t targetL, targetR;
    int32_t curL, curR;
    int32_t newL, newR;
    int32_t freqAbsL, freqAbsR;
    uint8_t enableL, enableR;

    if (s_emergencyStop) { return; }

    targetL = s_leftTargetFreq;
    targetR = s_rightTargetFreq;
    curL    = s_leftCurFreq;
    curR    = s_rightCurFreq;
    enableL = s_leftEnabled;
    enableR = s_rightEnabled;

    if (!enableL && targetL != 0) { targetL = 0; }
    if (!enableR && targetR != 0) { targetR = 0; }

    newL = StepperMotor_RampOne(targetL, curL,
        (int32_t)STEPPER_ACCEL_HZ_PER_TICK,
        (int32_t)STEPPER_DECEL_HZ_PER_TICK);

    newR = StepperMotor_RampOne(targetR, curR,
        (int32_t)STEPPER_ACCEL_HZ_PER_TICK,
        (int32_t)STEPPER_DECEL_HZ_PER_TICK);

    /*  Direction change safety: keep freq at 0 until we can switch DIR. */
    if (newL != 0 && curL == 0 && targetL != 0)
    {
        StepperMotor_SetDIR(GPIOB, DL_GPIO_PIN_17, targetL);
    }
    if (newR != 0 && curR == 0 && targetR != 0)
    {
        StepperMotor_SetDIR(GPIOB, DL_GPIO_PIN_19, targetR);
    }

    if (newL != curL)
    {
        freqAbsL = (newL >= 0) ? newL : -newL;
        StepperMotor_ApplyFreq(STEPPER_TIMER_L_INST,
            (newL >= 0) ? freqAbsL : freqAbsL);
        s_leftCurFreq = newL;
    }

    if (newR != curR)
    {
        freqAbsR = (newR >= 0) ? newR : -newR;
        StepperMotor_ApplyFreq(STEPPER_TIMER_R_INST,
            (newR >= 0) ? freqAbsR : freqAbsR);
        s_rightCurFreq = newR;
    }
}
