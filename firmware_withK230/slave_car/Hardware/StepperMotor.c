#include "StepperMotor.h"
#include "Board_Config.h"
#include "app_config.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

static volatile int32_t s_leftTargetFreq   = 0;
static volatile int32_t s_rightTargetFreq  = 0;
static volatile int32_t s_leftCurFreq      = 0;
static volatile int32_t s_rightCurFreq     = 0;
static volatile uint8_t s_leftEnabled      = 0U;
static volatile uint8_t s_rightEnabled     = 0U;
static volatile uint8_t s_emergencyStop    = 0U;

static volatile uint32_t s_leftStepCount  = 0U;
static volatile uint32_t s_rightStepCount = 0U;

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
        ccIndex);

    DL_TimerG_setCaptCompUpdateMethod(timer,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        ccIndex);

    DL_TimerG_setCaptureCompareValue(timer, 0U, ccIndex);

    DL_TimerG_setCCPDirection(timer,
        (ccIndex == DL_TIMER_CC_0_INDEX) ? DL_TIMER_CC0_OUTPUT
                                         : DL_TIMER_CC1_OUTPUT);
    DL_TimerG_enableClock(timer);
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
    {
        freqAbs = (int32_t)STEPPER_MAX_FREQ_HZ;
    }

    period = 32000000U / (uint32_t)freqAbs;
    if (period < 2U) { period = 2U; }
    if (period > 65535U) { period = 65535U; }

    /* At very low frequencies, period overflows 16-bit → stop STEP. */
    if (freqAbs < (int32_t)STEPPER_MIN_TIMER_FREQ_HZ)
    {
        DL_TimerG_stopCounter(timer);
        DL_TimerG_setCaptureCompareValue(timer, 0U, ccIndex);
        return;
    }

    DL_TimerG_setLoadValue(timer, period - 1U);
    DL_TimerG_setCaptureCompareValue(timer, period / 2U, ccIndex);
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
    s_leftStepCount   = 0U;
    s_rightStepCount  = 0U;

    DL_GPIO_clearPins(STEPPER_DIR_L_PORT, STEPPER_DIR_L_PIN);
    DL_GPIO_clearPins(STEPPER_DIR_R_PORT, STEPPER_DIR_R_PIN);

    DL_GPIO_initDigitalOutput(STEPPER_DIR_L_IOMUX);
    DL_GPIO_initDigitalOutput(STEPPER_DIR_R_IOMUX);

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
    s_leftEnabled = enable;
    if (!enable) { s_leftTargetFreq = 0; }
}

void StepperMotor_EnableRight(uint8_t enable)
{
    s_rightEnabled = enable;
    if (!enable) { s_rightTargetFreq = 0; }
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

int32_t StepperMotor_GetLeftTargetFrequency(void)  { return s_leftTargetFreq; }
int32_t StepperMotor_GetRightTargetFrequency(void) { return s_rightTargetFreq; }
int32_t StepperMotor_GetLeftCurrentFrequency(void) { return s_leftCurFreq; }
int32_t StepperMotor_GetRightCurrentFrequency(void){ return s_rightCurFreq; }
uint32_t StepperMotor_GetLeftStepCount(void)       { return s_leftStepCount; }
uint32_t StepperMotor_GetRightStepCount(void)      { return s_rightStepCount; }
uint8_t StepperMotor_IsLeftEnabled(void)           { return s_leftEnabled; }
uint8_t StepperMotor_IsRightEnabled(void)          { return s_rightEnabled; }

void StepperMotor_StopLeft(void)  { s_leftTargetFreq = 0; }
void StepperMotor_StopRight(void) { s_rightTargetFreq = 0; }
void StepperMotor_StopAll(void)   { s_leftTargetFreq = 0; s_rightTargetFreq = 0; }

void StepperMotor_EmergencyStop(void)
{
    s_emergencyStop = 1U;
    s_leftTargetFreq  = 0;
    s_rightTargetFreq = 0;
    s_leftCurFreq     = 0;
    s_rightCurFreq    = 0;

    DL_TimerG_stopCounter(STEPPER_STEP_L_TIMER_INST);
    DL_TimerG_setCaptureCompareValue(
        STEPPER_STEP_L_TIMER_INST, 0U, STEPPER_STEP_L_CC_INDEX);

    DL_TimerG_stopCounter(STEPPER_STEP_R_TIMER_INST);
    DL_TimerG_setCaptureCompareValue(
        STEPPER_STEP_R_TIMER_INST, 0U, STEPPER_STEP_R_CC_INDEX);

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
        /* Clamp to minimum timer frequency; actual freq floor is enforced
         * in ApplyFreq where sub-minimum stops the timer.  The ramp drives
         * current toward the floor so the transition into/out of motion is
         * clean. */
        if (target > 0) { target = (int32_t)STEPPER_MIN_TIMER_FREQ_HZ; }
        else            { target = -(int32_t)STEPPER_MIN_TIMER_FREQ_HZ; }
    }

    diff = target - current;
    if (diff > 0) { step = accelStep; }
    else          { step = decelStep; diff = -diff; }

    if ((int32_t)step >= diff) { return target; }

    if (target > current) { return current + (int32_t)step; }
    else                  { return current - (int32_t)step; }
}

void StepperMotor_Task1ms(void)
{
    int32_t targetL, targetR;
    int32_t curL, curR;
    int32_t newL, newR;

    if (s_emergencyStop) { return; }

    targetL = s_leftTargetFreq;
    targetR = s_rightTargetFreq;
    curL    = s_leftCurFreq;
    curR    = s_rightCurFreq;

    if (!s_leftEnabled  && targetL != 0) { targetL = 0; }
    if (!s_rightEnabled && targetR != 0) { targetR = 0; }

    newL = StepperMotor_RampOne(targetL, curL,
        (int32_t)STEPPER_ACCEL_HZ_PER_TICK,
        (int32_t)STEPPER_DECEL_HZ_PER_TICK);

    newR = StepperMotor_RampOne(targetR, curR,
        (int32_t)STEPPER_ACCEL_HZ_PER_TICK,
        (int32_t)STEPPER_DECEL_HZ_PER_TICK);

    /* Direction change: only flip DIR when current freq has reached 0.
     * DIR is set based on *target* sign, so it's correct before motion starts. */
    if (newL != 0 && curL == 0 && targetL != 0)
    {
        StepperMotor_SetDIR(STEPPER_DIR_L_PORT,
            STEPPER_DIR_L_PIN, targetL);
    }
    if (newR != 0 && curR == 0 && targetR != 0)
    {
        StepperMotor_SetDIR(STEPPER_DIR_R_PORT,
            STEPPER_DIR_R_PIN, targetR);
    }

    if (newL != curL)
    {
        int32_t absL = (newL >= 0) ? newL : -newL;
        StepperMotor_ApplyFreq(STEPPER_STEP_L_TIMER_INST,
            absL, STEPPER_STEP_L_CC_INDEX);
        s_leftCurFreq = newL;

        /* Accumulate emitted STEP count.
         * current_freq_hz = steps/second → steps/ms = freq/1000.
         * Use a fixed-point phase accumulator for precision. */
        {
            static uint32_t phaseL = 0U;
            phaseL += (uint32_t)((newL >= 0) ? newL : -newL);
            while (phaseL >= 1000U) {
                s_leftStepCount++;
                phaseL -= 1000U;
            }
        }
    }
    /* When stopped, step count is frozen; phase accumulation stops. */

    if (newR != curR)
    {
        int32_t absR = (newR >= 0) ? newR : -newR;
        StepperMotor_ApplyFreq(STEPPER_STEP_R_TIMER_INST,
            absR, STEPPER_STEP_R_CC_INDEX);
        s_rightCurFreq = newR;

        {
            static uint32_t phaseR = 0U;
            phaseR += (uint32_t)((newR >= 0) ? newR : -newR);
            while (phaseR >= 1000U) {
                s_rightStepCount++;
                phaseR -= 1000U;
            }
        }
    }
}
