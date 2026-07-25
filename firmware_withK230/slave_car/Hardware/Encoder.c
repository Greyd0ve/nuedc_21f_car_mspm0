#include "Encoder.h"
#include "app_config.h"
#include "Board_Config.h"
#include "Serial.h"
#include "cmsis_compiler.h"

#ifndef ENCODER_DIAG_ENABLE
#define ENCODER_DIAG_ENABLE 0U
#endif

#define ENCODER_ALL_PINS \
    (ENC_L_A_PIN | ENC_L_B_PIN | ENC_R_A_PIN | ENC_R_B_PIN)

/*
 * 4× quadrature state transition table.
 * Index = (prev_state << 2) | current_state
 * where state = (A_level << 1) | B_level  (0..3)
 *
 * +1 = forward step, -1 = reverse step, 0 = no change or illegal.
 */
static const int8_t s_quadTable[16] =
{
     0,  +1,  -1,   0,
    -1,   0,   0,  +1,
    +1,   0,   0,  -1,
     0,  -1,  +1,   0
};

static volatile int32_t s_leftDelta = 0;
static volatile int32_t s_rightDelta = 0;
static volatile uint8_t s_leftPrevState = 0U;
static volatile uint8_t s_rightPrevState = 0U;
static volatile uint32_t s_leftIllegal = 0U;
static volatile uint32_t s_rightIllegal = 0U;

#if ENCODER_DIAG_ENABLE
static volatile uint32_t s_gpioIrqCount = 0U;
static volatile uint32_t s_leftIsrCount = 0U;
static volatile uint32_t s_rightIsrCount = 0U;
static volatile uint32_t s_leftStatusCount = 0U;
static volatile uint32_t s_rightStatusCount = 0U;
static volatile int32_t s_leftLastRawDeltaBeforeLimit = 0;
static volatile int32_t s_rightLastRawDeltaBeforeLimit = 0;
static volatile uint32_t s_leftLimitHitCount = 0U;
static volatile uint32_t s_rightLimitHitCount = 0U;
static volatile uint32_t s_leftGetDeltaCount = 0U;
static volatile uint32_t s_rightGetDeltaCount = 0U;
static volatile uint32_t s_leftNonZeroGetCount = 0U;
static volatile uint32_t s_rightNonZeroGetCount = 0U;
static volatile int32_t s_leftMaxRawDelta = 0;
static volatile int32_t s_rightMaxRawDelta = 0;
#endif

#ifndef ENCODER_DEBUG_DISABLE_GPIO_IRQ
#define ENCODER_DEBUG_DISABLE_GPIO_IRQ 0U
#endif

static uint8_t Encoder_ReadAB(GPIO_Regs *port,
    uint32_t pinA, uint32_t pinB)
{
    uint8_t a = (DL_GPIO_readPins(port, pinA) != 0U) ? 1U : 0U;
    uint8_t b = (DL_GPIO_readPins(port, pinB) != 0U) ? 1U : 0U;
    return (uint8_t)((a << 1) | b);
}

static int32_t Encoder_Abs32(int32_t value)
{
    if (value < 0) { return (int32_t)((uint32_t)(-(value + 1)) + 1U); }
    return value;
}

static int16_t Encoder_LimitDelta(int32_t value)
{
    if (value > 32767)  { return 32767; }
    if (value < -32768) { return -32768; }
    return (int16_t)value;
}

static void Encoder_DebugSendUInt32(uint32_t value)
{
    char buffer[11];
    uint8_t index = 10U;
    buffer[index] = '\0';
    do {
        index--;
        buffer[index] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (index > 0U));
    Serial_SendString(&buffer[index]);
}

static void Encoder_DebugSendInt32(int32_t value)
{
    uint32_t magnitude;
    if (value < 0) {
        Serial_SendString("-");
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    Encoder_DebugSendUInt32(magnitude);
}

static void Encoder_DebugSendUInt32Field(const char *name, uint32_t value)
{
    Serial_SendString(name); Serial_SendString("=");
    Encoder_DebugSendUInt32(value); Serial_SendString("\r\n");
}

static void Encoder_DebugSendInt32Field(const char *name, int32_t value)
{
    Serial_SendString(name); Serial_SendString("=");
    Encoder_DebugSendInt32(value); Serial_SendString("\r\n");
}

void Encoder_DebugPrintDirectNoPrintf(const char *tag)
{
#if ENCODER_DIAG_ENABLE
    if (tag != 0) { Serial_SendString(tag); Serial_SendString("\r\n"); }
    Encoder_DebugSendUInt32Field("girq", s_gpioIrqCount);
    Encoder_DebugSendUInt32Field("lisr", s_leftIsrCount);
    Encoder_DebugSendUInt32Field("lstat", s_leftStatusCount);
    Encoder_DebugSendInt32Field("lraw", s_leftLastRawDeltaBeforeLimit);
    Encoder_DebugSendUInt32Field("llim", s_leftLimitHitCount);
    Encoder_DebugSendUInt32Field("lget", s_leftGetDeltaCount);
    Encoder_DebugSendUInt32Field("lnz", s_leftNonZeroGetCount);
    Encoder_DebugSendInt32Field("lmax", s_leftMaxRawDelta);
    Encoder_DebugSendUInt32Field("risr", s_rightIsrCount);
    Encoder_DebugSendUInt32Field("rstat", s_rightStatusCount);
    Encoder_DebugSendInt32Field("rraw", s_rightLastRawDeltaBeforeLimit);
    Encoder_DebugSendUInt32Field("rlim", s_rightLimitHitCount);
    Encoder_DebugSendUInt32Field("rget", s_rightGetDeltaCount);
    Encoder_DebugSendUInt32Field("rnz", s_rightNonZeroGetCount);
    Encoder_DebugSendInt32Field("rmax", s_rightMaxRawDelta);
#else
    if (tag != 0) { Serial_SendString(tag); Serial_SendString("\r\n"); }
    Serial_SendString("[encoder] diag disabled\r\n");
#endif
}

void Encoder_DebugPrintGetterNoPrintf(const char *tag)
{
#if ENCODER_DIAG_ENABLE
    uint32_t girq = s_gpioIrqCount;
    uint32_t lisr = s_leftIsrCount;
    uint32_t lstat = s_leftStatusCount;
    int32_t lraw = s_leftLastRawDeltaBeforeLimit;
    uint32_t llim = s_leftLimitHitCount;
    uint32_t lget = s_leftGetDeltaCount;
    uint32_t lnz = s_leftNonZeroGetCount;
    int32_t lmax = s_leftMaxRawDelta;
    uint32_t risr = s_rightIsrCount;
    uint32_t rstat = s_rightStatusCount;
    int32_t rraw = s_rightLastRawDeltaBeforeLimit;
    uint32_t rlim = s_rightLimitHitCount;
    uint32_t rget = s_rightGetDeltaCount;
    uint32_t rnz = s_rightNonZeroGetCount;
    int32_t rmax = s_rightMaxRawDelta;
    if (tag != 0) { Serial_SendString(tag); Serial_SendString("\r\n"); }
    Encoder_DebugSendUInt32Field("girq", girq);
    Encoder_DebugSendUInt32Field("lisr", lisr);
    Encoder_DebugSendUInt32Field("lstat", lstat);
    Encoder_DebugSendInt32Field("lraw", lraw);
    Encoder_DebugSendUInt32Field("llim", llim);
    Encoder_DebugSendUInt32Field("lget", lget);
    Encoder_DebugSendUInt32Field("lnz", lnz);
    Encoder_DebugSendInt32Field("lmax", lmax);
    Encoder_DebugSendUInt32Field("risr", risr);
    Encoder_DebugSendUInt32Field("rstat", rstat);
    Encoder_DebugSendInt32Field("rraw", rraw);
    Encoder_DebugSendUInt32Field("rlim", rlim);
    Encoder_DebugSendUInt32Field("rget", rget);
    Encoder_DebugSendUInt32Field("rnz", rnz);
    Encoder_DebugSendInt32Field("rmax", rmax);
#else
    if (tag != 0) { Serial_SendString(tag); Serial_SendString("\r\n"); }
    Serial_SendString("[encoder] diag disabled\r\n");
#endif
}

void Encoder_Init(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    DL_GPIO_disableInterrupt(ENCODER_GPIO_PORT, ENCODER_ALL_PINS);
    NVIC_DisableIRQ(ENCODER_GPIO_IRQN);

    /* Configure all four encoder signals as pulled-up digital inputs first. */
    DL_GPIO_initDigitalInputFeatures(ENC_L_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENC_L_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENC_R_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENC_R_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_disableOutput(ENCODER_GPIO_PORT, ENCODER_ALL_PINS);

    /*
     * PB0/PB5/PB8/PB12 are all in GPIOB[0..15].  Configure both edges in
     * POLARITY15_0 so the A/B transition table receives true 4× events.
     */
    DL_GPIO_setLowerPinsPolarity(ENCODER_GPIO_PORT,
        DL_GPIO_PIN_0_EDGE_RISE_FALL |
        DL_GPIO_PIN_5_EDGE_RISE_FALL |
        DL_GPIO_PIN_8_EDGE_RISE_FALL |
        DL_GPIO_PIN_12_EDGE_RISE_FALL);

    DL_GPIO_clearInterruptStatus(ENCODER_GPIO_PORT, ENCODER_ALL_PINS);

    /* Capture valid initial quadrature states only after input setup. */
    s_leftPrevState  = Encoder_ReadAB(
        ENC_L_A_PORT, ENC_L_A_PIN, ENC_L_B_PIN);
    s_rightPrevState = Encoder_ReadAB(
        ENC_R_A_PORT, ENC_R_A_PIN, ENC_R_B_PIN);
    s_leftDelta   = 0;
    s_rightDelta  = 0;
    s_leftIllegal  = 0U;
    s_rightIllegal = 0U;

#if ENCODER_DIAG_ENABLE
    s_gpioIrqCount      = 0U;
    s_leftIsrCount     = 0U;
    s_leftStatusCount  = 0U;
    s_rightIsrCount    = 0U;
    s_rightStatusCount = 0U;
    s_leftLastRawDeltaBeforeLimit  = 0;
    s_rightLastRawDeltaBeforeLimit = 0;
    s_leftLimitHitCount    = 0U;
    s_rightLimitHitCount   = 0U;
    s_leftGetDeltaCount    = 0U;
    s_rightGetDeltaCount   = 0U;
    s_leftNonZeroGetCount  = 0U;
    s_rightNonZeroGetCount = 0U;
    s_leftMaxRawDelta      = 0;
    s_rightMaxRawDelta     = 0;
#endif

#if ENCODER_DEBUG_DISABLE_GPIO_IRQ
    NVIC_ClearPendingIRQ(ENCODER_GPIO_IRQN);
#else
    DL_GPIO_enableInterrupt(ENCODER_GPIO_PORT, ENCODER_ALL_PINS);
    NVIC_ClearPendingIRQ(ENCODER_GPIO_IRQN);
    NVIC_EnableIRQ(ENCODER_GPIO_IRQN);
#endif

    if (primask == 0U) { __enable_irq(); }
}

int16_t Encoder_GetLeftDelta(void)
{
    int32_t value;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
#if ENCODER_DIAG_ENABLE
    s_leftGetDeltaCount++;
#endif
    value = s_leftDelta;
#if ENCODER_DIAG_ENABLE
    s_leftLastRawDeltaBeforeLimit = value;
    if (value != 0) { s_leftNonZeroGetCount++; }
    if ((value > 32767) || (value < -32768)) { s_leftLimitHitCount++; }
    if (Encoder_Abs32(value) > Encoder_Abs32(s_leftMaxRawDelta))
        { s_leftMaxRawDelta = value; }
#endif
    s_leftDelta = 0;
    if (primask == 0U) { __enable_irq(); }
    return Encoder_LimitDelta(value);
}

int16_t Encoder_GetRightDelta(void)
{
    int32_t value;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
#if ENCODER_DIAG_ENABLE
    s_rightGetDeltaCount++;
#endif
    value = s_rightDelta;
#if ENCODER_DIAG_ENABLE
    s_rightLastRawDeltaBeforeLimit = value;
    if (value != 0) { s_rightNonZeroGetCount++; }
    if ((value > 32767) || (value < -32768)) { s_rightLimitHitCount++; }
    if (Encoder_Abs32(value) > Encoder_Abs32(s_rightMaxRawDelta))
        { s_rightMaxRawDelta = value; }
#endif
    s_rightDelta = 0;
    if (primask == 0U) { __enable_irq(); }
    return Encoder_LimitDelta(value);
}

void Encoder_ClearAll(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_leftDelta = 0;
    s_rightDelta = 0;
    if (primask == 0U) { __enable_irq(); }
}

uint32_t Encoder_GetGpioIrqCount(void)
{
#if ENCODER_DIAG_ENABLE
    return s_gpioIrqCount;
#else
    return 0U;
#endif
}

uint32_t Encoder_GetLeftIsrCount(void)
{
#if ENCODER_DIAG_ENABLE
    return s_leftIsrCount;
#else
    return 0U;
#endif
}

uint32_t Encoder_GetLeftStatusCount(void)
{
#if ENCODER_DIAG_ENABLE
    return s_leftStatusCount;
#else
    return 0U;
#endif
}

int32_t Encoder_GetLeftLastRawDeltaBeforeLimit(void)
{
#if ENCODER_DIAG_ENABLE
    return s_leftLastRawDeltaBeforeLimit;
#else
    return 0;
#endif
}

uint32_t Encoder_GetLeftLimitHitCount(void)
{
#if ENCODER_DIAG_ENABLE
    return s_leftLimitHitCount;
#else
    return 0U;
#endif
}

uint32_t Encoder_GetLeftGetDeltaCount(void)
{
#if ENCODER_DIAG_ENABLE
    return s_leftGetDeltaCount;
#else
    return 0U;
#endif
}

uint32_t Encoder_GetLeftNonZeroGetCount(void)
{
#if ENCODER_DIAG_ENABLE
    return s_leftNonZeroGetCount;
#else
    return 0U;
#endif
}

int32_t Encoder_GetLeftMaxRawDelta(void)
{
#if ENCODER_DIAG_ENABLE
    return s_leftMaxRawDelta;
#else
    return 0;
#endif
}

uint32_t Encoder_GetRightIsrCount(void)
{
#if ENCODER_DIAG_ENABLE
    return s_rightIsrCount;
#else
    return 0U;
#endif
}

uint32_t Encoder_GetRightSameAIgnored(void) { return 0U; }

uint32_t Encoder_GetRightStatusCount(void)
{
#if ENCODER_DIAG_ENABLE
    return s_rightStatusCount;
#else
    return 0U;
#endif
}

int32_t Encoder_GetRightLastRawDeltaBeforeLimit(void)
{
#if ENCODER_DIAG_ENABLE
    return s_rightLastRawDeltaBeforeLimit;
#else
    return 0;
#endif
}

uint32_t Encoder_GetRightLimitHitCount(void)
{
#if ENCODER_DIAG_ENABLE
    return s_rightLimitHitCount;
#else
    return 0U;
#endif
}

uint32_t Encoder_GetRightGetDeltaCount(void)
{
#if ENCODER_DIAG_ENABLE
    return s_rightGetDeltaCount;
#else
    return 0U;
#endif
}

uint32_t Encoder_GetRightNonZeroGetCount(void)
{
#if ENCODER_DIAG_ENABLE
    return s_rightNonZeroGetCount;
#else
    return 0U;
#endif
}

int32_t Encoder_GetRightMaxRawDelta(void)
{
#if ENCODER_DIAG_ENABLE
    return s_rightMaxRawDelta;
#else
    return 0;
#endif
}

uint32_t Encoder_GetLeftIllegalCount(void)  { return s_leftIllegal; }
uint32_t Encoder_GetRightIllegalCount(void) { return s_rightIllegal; }

/* Unified ISR: all 4 encoder pins share GPIOB interrupt. */
void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
    {
    case ENCODER_GPIO_INT_IIDX:
    {
        uint32_t status = DL_GPIO_getEnabledInterruptStatus(
            ENCODER_GPIO_PORT, ENCODER_ALL_PINS);
        DL_GPIO_clearInterruptStatus(ENCODER_GPIO_PORT, status);

#if ENCODER_DIAG_ENABLE
        s_gpioIrqCount++;
#endif

        if (status & (ENC_L_A_PIN | ENC_L_B_PIN))
        {
            uint8_t cur = Encoder_ReadAB(
                ENC_L_A_PORT, ENC_L_A_PIN, ENC_L_B_PIN);
            uint8_t idx;
            int8_t step;
#if ENCODER_DIAG_ENABLE
            s_leftIsrCount++;
#endif
            idx = (uint8_t)((s_leftPrevState << 2) | cur);
            step = s_quadTable[idx];
            if (step != 0)
            {
#if ENCODER_DIAG_ENABLE
                s_leftStatusCount++;
#endif
                s_leftDelta += (int32_t)step
                               * (int32_t)LEFT_ENCODER_DIR_SIGN;
            }
            else if (cur != s_leftPrevState)
            {
                s_leftIllegal++;
            }
            s_leftPrevState = cur;
        }

        if (status & (ENC_R_A_PIN | ENC_R_B_PIN))
        {
            uint8_t cur = Encoder_ReadAB(
                ENC_R_A_PORT, ENC_R_A_PIN, ENC_R_B_PIN);
            uint8_t idx;
            int8_t step;
#if ENCODER_DIAG_ENABLE
            s_rightIsrCount++;
#endif
            idx = (uint8_t)((s_rightPrevState << 2) | cur);
            step = s_quadTable[idx];
            if (step != 0)
            {
#if ENCODER_DIAG_ENABLE
                s_rightStatusCount++;
#endif
                s_rightDelta += (int32_t)step
                                * (int32_t)RIGHT_ENCODER_DIR_SIGN;
            }
            else if (cur != s_rightPrevState)
            {
                s_rightIllegal++;
            }
            s_rightPrevState = cur;
        }
    }
    break;

    default:
        break;
    }
}
