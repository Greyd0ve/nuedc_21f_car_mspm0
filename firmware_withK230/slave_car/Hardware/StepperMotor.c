#include "StepperMotor.h"
#include "Board_Config.h"
#include "app_config.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

typedef enum
{
    DIR_STATE_NORMAL = 0,
    DIR_STATE_RAMP_TO_ZERO,
    DIR_STATE_HOLD_ZERO
} StepDirState_t;

static volatile int32_t s_leftTargetFreq    = 0;
static volatile int32_t s_rightTargetFreq   = 0;
static volatile int32_t s_leftCurFreq       = 0;
static volatile int32_t s_rightCurFreq      = 0;
static volatile int32_t s_leftPendingTarget = 0;
static volatile int32_t s_rightPendingTarget= 0;
static volatile uint8_t s_leftEnabled       = 0U;
static volatile uint8_t s_rightEnabled      = 0U;
static volatile uint8_t s_emergencyStop     = 0U;

static volatile int32_t s_leftSignedStepCount  = 0;
static volatile int32_t s_rightSignedStepCount = 0;
static volatile uint32_t s_leftPhase  = 0U;
static volatile uint32_t s_rightPhase = 0U;

static volatile StepDirState_t s_leftDirState  = DIR_STATE_NORMAL;
static volatile StepDirState_t s_rightDirState = DIR_STATE_NORMAL;

static volatile uint8_t s_startupHold = 1U;

static void StepperMotor_SetDIR(GPIO_Regs *port, uint32_t pin, int32_t freq)
{
    if (freq >= 0) { DL_GPIO_setPins(port, pin); }
    else           { DL_GPIO_clearPins(port, pin); }
}

static void StepperMotor_InitTimerChannel(
    GPTIMER_Regs *timer,
    uint32_t iomux,
    uint32_t iomuxFunc,
    GPIO_Regs *port,
    uint32_t pinMask,
    uint8_t ccIndex)
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

    DL_GPIO_initDigitalOutput(iomux);
    DL_GPIO_clearPins(port, pinMask);

    DL_TimerG_reset(timer);
    DL_TimerG_enablePower(timer);
    DL_TimerG_setClockConfig(timer, &clockCfg);
    DL_TimerG_initPWMMode(timer, &pwmCfg);

    DL_TimerG_setCaptureCompareOutCtl(timer,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        ccIndex);

    DL_TimerG_setCaptCompUpdateMethod(timer,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        ccIndex);

    DL_TimerG_setCaptureCompareValue(timer, 0U, ccIndex);

    DL_TimerG_setCCPDirection(timer,
        (ccIndex == DL_TIMER_CC_0_INDEX) ? DL_TIMER_CC0_OUTPUT
                                         : DL_TIMER_CC1_OUTPUT);
    DL_TimerG_enableClock(timer);

    DL_GPIO_initPeripheralOutputFunction(iomux, iomuxFunc);
    DL_GPIO_enableOutput(port, pinMask);
}

static void StepperMotor_ApplyFreq(
    GPTIMER_Regs *timer, int32_t freqAbs, uint8_t ccIndex)
{
    uint32_t period;

    if (freqAbs == 0)
    {
        DL_TimerG_stopCounter(timer);
        DL_TimerG_setCaptureCompareValue(timer, 0U, ccIndex);
        return;
    }

    if ((uint32_t)freqAbs > (uint32_t)STEPPER_MAX_FREQ_HZ)
        { freqAbs = (int32_t)STEPPER_MAX_FREQ_HZ; }

    if (freqAbs < (int32_t)STEPPER_MIN_TIMER_FREQ_HZ)
    {
        DL_TimerG_stopCounter(timer);
        DL_TimerG_setCaptureCompareValue(timer, 0U, ccIndex);
        return;
    }

    period = 32000000U / (uint32_t)freqAbs;
    if (period < 2U) { period = 2U; }
    if (period > 65535U) { period = 65535U; }

    DL_TimerG_setLoadValue(timer, period - 1U);
    DL_TimerG_setCaptureCompareValue(timer, period / 2U, ccIndex);
    DL_TimerG_startCounter(timer);
}

void StepperMotor_Init(void)
{
    s_leftTargetFreq   = 0;
    s_rightTargetFreq  = 0;
    s_leftCurFreq      = 0;
    s_rightCurFreq     = 0;
    s_leftPendingTarget= 0;
    s_rightPendingTarget=0;
    s_emergencyStop    = 0U;
    s_leftSignedStepCount  = 0;
    s_rightSignedStepCount = 0;
    s_leftPhase   = 0U;
    s_rightPhase  = 0U;
    s_leftDirState  = DIR_STATE_NORMAL;
    s_rightDirState = DIR_STATE_NORMAL;
    s_startupHold   = 1U;

#if STEPPER_HAS_ENABLE_GPIO
    s_leftEnabled  = 0U;
    s_rightEnabled = 0U;
#else
    s_leftEnabled  = 1U;
    s_rightEnabled = 1U;
#endif

    DL_GPIO_initDigitalOutput(STEPPER_DIR_L_IOMUX);
    DL_GPIO_initDigitalOutput(STEPPER_DIR_R_IOMUX);

    StepperMotor_SetDIR(STEPPER_DIR_L_PORT, STEPPER_DIR_L_PIN,
        (int32_t)LEFT_STEPPER_DIR_SIGN);
    StepperMotor_SetDIR(STEPPER_DIR_R_PORT, STEPPER_DIR_R_PIN,
        (int32_t)RIGHT_STEPPER_DIR_SIGN);

    DL_GPIO_enableOutput(STEPPER_DIR_L_PORT, STEPPER_DIR_L_PIN);
    DL_GPIO_enableOutput(STEPPER_DIR_R_PORT, STEPPER_DIR_R_PIN);

    StepperMotor_InitTimerChannel(
        STEPPER_STEP_L_TIMER_INST,
        STEPPER_STEP_L_IOMUX,
        STEPPER_STEP_L_IOMUX_FUNC,
        STEPPER_STEP_L_PORT,
        STEPPER_STEP_L_PIN,
        STEPPER_STEP_L_CC_INDEX);

    StepperMotor_InitTimerChannel(
        STEPPER_STEP_R_TIMER_INST,
        STEPPER_STEP_R_IOMUX,
        STEPPER_STEP_R_IOMUX_FUNC,
        STEPPER_STEP_R_PORT,
        STEPPER_STEP_R_PIN,
        STEPPER_STEP_R_CC_INDEX);
}

void StepperMotor_EnableLeft(uint8_t enable)
{
#if STEPPER_HAS_ENABLE_GPIO
    s_leftEnabled = enable;
    if (!enable) { s_leftTargetFreq = 0; }
#else
    (void)enable;
#endif
}

void StepperMotor_EnableRight(uint8_t enable)
{
#if STEPPER_HAS_ENABLE_GPIO
    s_rightEnabled = enable;
    if (!enable) { s_rightTargetFreq = 0; }
#else
    (void)enable;
#endif
}

void StepperMotor_EnableAll(uint8_t enable)
{
    StepperMotor_EnableLeft(enable);
    StepperMotor_EnableRight(enable);
}

int32_t StepperMotor_SetLeftTargetFrequency(int32_t hz)
{
    if (hz > (int32_t)STEPPER_MAX_FREQ_HZ) hz = (int32_t)STEPPER_MAX_FREQ_HZ;
    if (hz < -(int32_t)STEPPER_MAX_FREQ_HZ) hz = -(int32_t)STEPPER_MAX_FREQ_HZ;
    s_leftTargetFreq = hz;
    s_leftPendingTarget = 0;
    s_leftDirState = DIR_STATE_NORMAL;
    return hz;
}

int32_t StepperMotor_SetRightTargetFrequency(int32_t hz)
{
    if (hz > (int32_t)STEPPER_MAX_FREQ_HZ) hz = (int32_t)STEPPER_MAX_FREQ_HZ;
    if (hz < -(int32_t)STEPPER_MAX_FREQ_HZ) hz = -(int32_t)STEPPER_MAX_FREQ_HZ;
    s_rightTargetFreq = hz;
    s_rightPendingTarget = 0;
    s_rightDirState = DIR_STATE_NORMAL;
    return hz;
}

void StepperMotor_SetTargetFrequency(int32_t l, int32_t r)
{
    StepperMotor_SetLeftTargetFrequency(l);
    StepperMotor_SetRightTargetFrequency(r);
}

int32_t StepperMotor_GetLeftTargetFrequency(void) { return s_leftTargetFreq; }
int32_t StepperMotor_GetRightTargetFrequency(void){ return s_rightTargetFreq; }
int32_t StepperMotor_GetLeftCurrentFrequency(void){ return s_leftCurFreq; }
int32_t StepperMotor_GetRightCurrentFrequency(void){return s_rightCurFreq; }
int32_t StepperMotor_GetLeftSignedStepCount(void) { return s_leftSignedStepCount; }
int32_t StepperMotor_GetRightSignedStepCount(void){ return s_rightSignedStepCount; }
uint8_t StepperMotor_IsLeftEnabled(void)          { return s_leftEnabled; }
uint8_t StepperMotor_IsRightEnabled(void)         { return s_rightEnabled; }

void StepperMotor_StopLeft(void)
{
    s_leftTargetFreq = 0;
    s_leftPendingTarget = 0;
    s_leftDirState = DIR_STATE_NORMAL;
}

void StepperMotor_StopRight(void)
{
    s_rightTargetFreq = 0;
    s_rightPendingTarget = 0;
    s_rightDirState = DIR_STATE_NORMAL;
}

void StepperMotor_StopAll(void)
{
    s_leftTargetFreq = 0;
    s_leftPendingTarget = 0;
    s_leftDirState = DIR_STATE_NORMAL;
    s_rightTargetFreq = 0;
    s_rightPendingTarget = 0;
    s_rightDirState = DIR_STATE_NORMAL;
}

void StepperMotor_EmergencyStop(void)
{
    s_emergencyStop = 1U;
    s_leftTargetFreq   = 0;
    s_rightTargetFreq  = 0;
    s_leftCurFreq      = 0;
    s_rightCurFreq     = 0;
    s_leftPendingTarget= 0;
    s_rightPendingTarget=0;
    s_leftDirState  = DIR_STATE_NORMAL;
    s_rightDirState = DIR_STATE_NORMAL;

    DL_TimerG_stopCounter(STEPPER_STEP_L_TIMER_INST);
    DL_TimerG_setCaptureCompareValue(
        STEPPER_STEP_L_TIMER_INST, 0U, STEPPER_STEP_L_CC_INDEX);
    DL_TimerG_stopCounter(STEPPER_STEP_R_TIMER_INST);
    DL_TimerG_setCaptureCompareValue(
        STEPPER_STEP_R_TIMER_INST, 0U, STEPPER_STEP_R_CC_INDEX);

    /* NOTE: EN is hardware always-enabled.  EmergencyStop only
     * stops STEP pulses.  Driver power stage remains active. */
}

void StepperMotor_ClearEmergencyStop(void) { s_emergencyStop = 0U; }

uint8_t StepperMotor_IsEmergencyStopped(void) { return s_emergencyStop; }

void StepperMotor_ClearStepCount(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_leftSignedStepCount  = 0;
    s_rightSignedStepCount = 0;
    s_leftPhase  = 0U;
    s_rightPhase = 0U;
    if (primask == 0U) { __enable_irq(); }
}

/* Ramp toward target without crossing zero.
 * If current and target have opposite signs, ramps toward 0:
 * returns the next step toward 0, never past 0. */
static int32_t StepperMotor_RampOne(
    int32_t target, int32_t current, int32_t accel, int32_t decel)
{
    int32_t step, diff;

    if (target == current) return target;

    /* Opposite signs → force ramp toward 0. */
    if ((target > 0 && current < 0) || (target < 0 && current > 0))
    {
        if (current > 0)
        {
            current -= decel;
            if (current < 0) current = 0;
        }
        else
        {
            current += decel;
            if (current > 0) current = 0;
        }
        return current;
    }

    /* Same sign or one is zero — normal ramp. */
    diff = target - current;
    if (diff > 0) step = accel;  else { step = decel; diff = -diff; }
    if ((int32_t)step >= diff) return target;
    return (target > current) ? (current + (int32_t)step)
                              : (current - (int32_t)step);
}

static void StepperMotor_StepChannel(
    int32_t target, volatile int32_t *cur,
    volatile int32_t *pendingTarget, volatile StepDirState_t *dirState,
    GPTIMER_Regs *timer, uint8_t ccIndex,
    GPIO_Regs *dirPort, uint32_t dirPin,
    volatile int32_t *stepCount, volatile uint32_t *phase)
{
    int32_t effectiveTarget;
    int32_t newCur;

    effectiveTarget = target;

    /* Start from stop: set DIR based on target sign, then hold 1 tick. */
    if (*cur == 0 && target != 0 && *dirState == DIR_STATE_NORMAL)
    {
        *pendingTarget = target;
        StepperMotor_ApplyFreq(timer, 0, ccIndex);
        StepperMotor_SetDIR(dirPort, dirPin, target);
        *dirState = DIR_STATE_HOLD_ZERO;
        return;
    }

    /* In-flight direction change: opposite signs → ramp to zero first. */
    if (*cur != 0 && target != 0 &&
        ((target > 0 && *cur < 0) || (target < 0 && *cur > 0)))
    {
        if (*dirState == DIR_STATE_NORMAL)
        {
            *pendingTarget = target;
            *dirState = DIR_STATE_RAMP_TO_ZERO;
            effectiveTarget = 0;
        }
        else
        {
            effectiveTarget = 0;
        }
    }

    if (*dirState == DIR_STATE_RAMP_TO_ZERO && *cur == 0)
    {
        /* Reached zero — stop timer, switch DIR, wait 1 tick */
        StepperMotor_ApplyFreq(timer, 0, ccIndex);
        StepperMotor_SetDIR(dirPort, dirPin, *pendingTarget);
        *dirState = DIR_STATE_HOLD_ZERO;
        return;
    }

    if (*dirState == DIR_STATE_HOLD_ZERO)
    {
        /* DIR settling tick done — start ramping to pending target */
        *dirState = DIR_STATE_NORMAL;
        effectiveTarget = *pendingTarget;
        *pendingTarget = 0;
    }

    if (effectiveTarget == 0 && *cur == 0)
    {
        *dirState = DIR_STATE_NORMAL;
        return;
    }

    newCur = StepperMotor_RampOne(effectiveTarget, *cur,
        (int32_t)STEPPER_ACCEL_HZ_PER_TICK,
        (int32_t)STEPPER_DECEL_HZ_PER_TICK);

    if (newCur != *cur)
    {
        int32_t absNew = (newCur >= 0) ? newCur : -newCur;
        StepperMotor_ApplyFreq(timer, absNew, ccIndex);
        *cur = newCur;
    }

    /* Accumulate signed STEP count every 1ms regardless of whether
     * frequency changed.  steady-state rate = abs(cur) steps/ms. */
    {
        int32_t absCur = (*cur >= 0) ? *cur : -*cur;
        int32_t sign   = (*cur >= 0) ? 1 : -1;

        if (absCur > 0)
        {
            *phase += (uint32_t)absCur;
            while (*phase >= 1000U)
            {
                *stepCount += sign;
                *phase -= 1000U;
            }
        }
    }
}

void StepperMotor_Task1ms(void)
{
    if (s_emergencyStop) { return; }

    /* Startup hold: after Init(), let DIR settle at least 1ms
     * before any STEP pulse can be emitted. */
    if (s_startupHold > 0U)
    {
        s_startupHold--;
        return;
    }

#if STEPPER_HAS_ENABLE_GPIO
    if (!s_leftEnabled)  { s_leftTargetFreq = 0; }
    if (!s_rightEnabled) { s_rightTargetFreq = 0; }
#endif

    StepperMotor_StepChannel(
        s_leftTargetFreq, &s_leftCurFreq,
        &s_leftPendingTarget, &s_leftDirState,
        STEPPER_STEP_L_TIMER_INST, STEPPER_STEP_L_CC_INDEX,
        STEPPER_DIR_L_PORT, STEPPER_DIR_L_PIN,
        &s_leftSignedStepCount, &s_leftPhase);

    StepperMotor_StepChannel(
        s_rightTargetFreq, &s_rightCurFreq,
        &s_rightPendingTarget, &s_rightDirState,
        STEPPER_STEP_R_TIMER_INST, STEPPER_STEP_R_CC_INDEX,
        STEPPER_DIR_R_PORT, STEPPER_DIR_R_PIN,
        &s_rightSignedStepCount, &s_rightPhase);
}
