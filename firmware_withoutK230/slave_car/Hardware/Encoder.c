#include "Encoder.h"
#include "app_config.h"
#include "Board_Config.h"
#include "Serial.h"
#include "cmsis_compiler.h"

static volatile int32_t s_leftDelta = 0;
static volatile int32_t s_rightDelta = 0;
static volatile uint8_t s_leftALast = 0U;
static volatile uint8_t s_rightALast = 0U;

#ifndef ENCODER_COUNT_A_RISE_ONLY
#define ENCODER_COUNT_A_RISE_ONLY 1U
#endif

#ifndef ENCODER_DIAG_ENABLE
#define ENCODER_DIAG_ENABLE 0U
#endif

#ifndef ENCODER_DEBUG_DISABLE_GPIO_IRQ
#define ENCODER_DEBUG_DISABLE_GPIO_IRQ 0U
#endif

#ifndef ENCODER_INIT_SETTLE_CYCLES
#define ENCODER_INIT_SETTLE_CYCLES 320000U
#endif

#if ENCODER_DIAG_ENABLE
static volatile uint32_t s_leftIsrCount = 0U;
static volatile uint32_t s_leftSameAIgnored = 0U;
static volatile uint32_t s_leftStatusCount = 0U;
static volatile uint32_t s_rightIsrCount = 0U;
static volatile uint32_t s_rightSameAIgnored = 0U;
static volatile uint32_t s_rightStatusCount = 0U;
static volatile int32_t s_rightLastRawDeltaBeforeLimit = 0;
static volatile uint32_t s_rightLimitHitCount = 0U;
static volatile uint32_t s_rightGetDeltaCount = 0U;
static volatile uint32_t s_rightNonZeroGetCount = 0U;
static volatile int32_t s_rightMaxRawDelta = 0;
#endif

static uint8_t Encoder_ReadLevel(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0U) ? 1U : 0U;
}

static uint32_t Encoder_Abs32(int32_t value)
{
    if (value < 0)
    {
        return (uint32_t)(-(value + 1)) + 1U;
    }
    return (uint32_t)value;
}

static int16_t Encoder_LimitDelta(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }
    return (int16_t)value;
}

static void Encoder_DebugSendUInt32(uint32_t value)
{
    char buffer[11];
    uint8_t index = 10U;

    buffer[index] = '\0';
    do
    {
        index--;
        buffer[index] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (index > 0U));

    Serial_SendString(&buffer[index]);
}

static void Encoder_DebugSendInt32(int32_t value)
{
    uint32_t magnitude;

    if (value < 0)
    {
        Serial_SendString("-");
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)value;
    }

    Encoder_DebugSendUInt32(magnitude);
}

static void Encoder_DebugSendUInt32Field(const char *name, uint32_t value)
{
    Serial_SendString(name);
    Serial_SendString("=");
    Encoder_DebugSendUInt32(value);
    Serial_SendString("\r\n");
}

static void Encoder_DebugSendInt32Field(const char *name, int32_t value)
{
    Serial_SendString(name);
    Serial_SendString("=");
    Encoder_DebugSendInt32(value);
    Serial_SendString("\r\n");
}

void Encoder_DebugPrintDirectNoPrintf(const char *tag)
{
#if ENCODER_DIAG_ENABLE
    if (tag != 0)
    {
        Serial_SendString(tag);
        Serial_SendString("\r\n");
    }

    Encoder_DebugSendUInt32Field("risr", s_rightIsrCount);
    Encoder_DebugSendUInt32Field("rign", s_rightSameAIgnored);
    Encoder_DebugSendUInt32Field("rstat", s_rightStatusCount);
    Encoder_DebugSendInt32Field("rraw", s_rightLastRawDeltaBeforeLimit);
    Encoder_DebugSendUInt32Field("rlim", s_rightLimitHitCount);
    Encoder_DebugSendUInt32Field("rget", s_rightGetDeltaCount);
    Encoder_DebugSendUInt32Field("rnz", s_rightNonZeroGetCount);
    Encoder_DebugSendInt32Field("rmax", s_rightMaxRawDelta);
#else
    if (tag != 0)
    {
        Serial_SendString(tag);
        Serial_SendString("\r\n");
    }
    Serial_SendString("[encoder] diag disabled\r\n");
#endif
}

void Encoder_DebugPrintGetterNoPrintf(const char *tag)
{
#if ENCODER_DIAG_ENABLE
    uint32_t risr = Encoder_GetRightIsrCount();
    uint32_t rign = Encoder_GetRightSameAIgnored();
    uint32_t rstat = Encoder_GetRightStatusCount();
    int32_t rraw = Encoder_GetRightLastRawDeltaBeforeLimit();
    uint32_t rlim = Encoder_GetRightLimitHitCount();
    uint32_t rget = Encoder_GetRightGetDeltaCount();
    uint32_t rnz = Encoder_GetRightNonZeroGetCount();
    int32_t rmax = Encoder_GetRightMaxRawDelta();

    if (tag != 0)
    {
        Serial_SendString(tag);
        Serial_SendString("\r\n");
    }

    Encoder_DebugSendUInt32Field("risr", risr);
    Encoder_DebugSendUInt32Field("rign", rign);
    Encoder_DebugSendUInt32Field("rstat", rstat);
    Encoder_DebugSendInt32Field("rraw", rraw);
    Encoder_DebugSendUInt32Field("rlim", rlim);
    Encoder_DebugSendUInt32Field("rget", rget);
    Encoder_DebugSendUInt32Field("rnz", rnz);
    Encoder_DebugSendInt32Field("rmax", rmax);
#else
    if (tag != 0)
    {
        Serial_SendString(tag);
        Serial_SendString("\r\n");
    }
    Serial_SendString("[encoder] diag disabled\r\n");
#endif
}

void Encoder_Init(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_leftALast = Encoder_ReadLevel(ENC_L_A_PORT, ENC_L_A_PIN);
    s_rightALast = Encoder_ReadLevel(ENC_R_A_PORT, ENC_R_A_PIN);
    s_leftDelta = 0;
    s_rightDelta = 0;
#if ENCODER_DIAG_ENABLE
    s_leftIsrCount = 0U;
    s_leftSameAIgnored = 0U;
    s_leftStatusCount = 0U;
    s_rightIsrCount = 0U;
    s_rightSameAIgnored = 0U;
    s_rightStatusCount = 0U;
    s_rightLastRawDeltaBeforeLimit = 0;
    s_rightLimitHitCount = 0U;
    s_rightGetDeltaCount = 0U;
    s_rightNonZeroGetCount = 0U;
    s_rightMaxRawDelta = 0;
#endif

    DL_GPIO_clearInterruptStatus(ENC_L_A_PORT, ENC_L_A_PIN);
    DL_GPIO_clearInterruptStatus(ENC_R_A_PORT, ENC_R_A_PIN);
    NVIC_ClearPendingIRQ(ENCODER_GPIO_IRQN);
#if ENCODER_DEBUG_DISABLE_GPIO_IRQ
    DL_GPIO_disableInterrupt(ENC_L_A_PORT, ENC_L_A_PIN);
    DL_GPIO_disableInterrupt(ENC_R_A_PORT, ENC_R_A_PIN);
    NVIC_DisableIRQ(ENCODER_GPIO_IRQN);
#else
    DL_GPIO_enableInterrupt(ENC_L_A_PORT, ENC_L_A_PIN);
    DL_GPIO_enableInterrupt(ENC_R_A_PORT, ENC_R_A_PIN);
    NVIC_EnableIRQ(ENCODER_GPIO_IRQN);
#endif

		if (primask == 0U)
		{
				__enable_irq();
		}

		#if !ENCODER_DEBUG_DISABLE_GPIO_IRQ
		/*
		 * Let GPIO input level and pending edge status settle after enabling
		 * encoder interrupts. Then clear pending status again to avoid startup
		 * glitches being counted as motion.
		 */
		delay_cycles(ENCODER_INIT_SETTLE_CYCLES);

		primask = __get_PRIMASK();
		__disable_irq();

		DL_GPIO_clearInterruptStatus(ENC_L_A_PORT, ENC_L_A_PIN);
		DL_GPIO_clearInterruptStatus(ENC_R_A_PORT, ENC_R_A_PIN);
		NVIC_ClearPendingIRQ(ENCODER_GPIO_IRQN);

		s_leftALast = Encoder_ReadLevel(ENC_L_A_PORT, ENC_L_A_PIN);
		s_rightALast = Encoder_ReadLevel(ENC_R_A_PORT, ENC_R_A_PIN);

		s_leftDelta = 0;
		s_rightDelta = 0;

#if ENCODER_DIAG_ENABLE
		s_leftIsrCount = 0U;
		s_leftSameAIgnored = 0U;
		s_leftStatusCount = 0U;

		s_rightIsrCount = 0U;
		s_rightSameAIgnored = 0U;
		s_rightStatusCount = 0U;
		s_rightLastRawDeltaBeforeLimit = 0;
		s_rightLimitHitCount = 0U;
		s_rightGetDeltaCount = 0U;
		s_rightNonZeroGetCount = 0U;
		s_rightMaxRawDelta = 0;
#endif

		if (primask == 0U)
		{
				__enable_irq();
		}
		#endif
}

int16_t Encoder_GetLeftDelta(void)
{
    int32_t value;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    value = s_leftDelta;
    s_leftDelta = 0;
    if (primask == 0U)
    {
        __enable_irq();
    }

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
    if (value != 0)
    {
        s_rightNonZeroGetCount++;
    }
    if ((value > 32767) || (value < -32768))
    {
        s_rightLimitHitCount++;
    }
    if (Encoder_Abs32(value) > Encoder_Abs32(s_rightMaxRawDelta))
    {
        s_rightMaxRawDelta = value;
    }
#endif
    s_rightDelta = 0;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return Encoder_LimitDelta(value);
}

void Encoder_ClearAll(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_leftDelta = 0;
    s_rightDelta = 0;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

uint32_t Encoder_GetRightIsrCount(void)
{
#if ENCODER_DIAG_ENABLE
    return s_rightIsrCount;
#else
    return 0U;
#endif
}

uint32_t Encoder_GetRightSameAIgnored(void)
{
#if ENCODER_DIAG_ENABLE
    return s_rightSameAIgnored;
#else
    return 0U;
#endif
}

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

static void Encoder_HandleLeftA(void)
{
    uint8_t a = Encoder_ReadLevel(ENC_L_A_PORT, ENC_L_A_PIN);
    uint8_t b;
    int32_t dir;

#if ENCODER_DIAG_ENABLE
    s_leftIsrCount++;
#endif
    if (a == s_leftALast)
    {
#if ENCODER_DIAG_ENABLE
        s_leftSameAIgnored++;
#endif
        return;
    }
    s_leftALast = a;

#if ENCODER_COUNT_A_RISE_ONLY
    if (a == 0U)
    {
        return;
    }
#endif

    b = Encoder_ReadLevel(ENC_L_B_PORT, ENC_L_B_PIN);
    dir = (a == b) ? 1 : -1;
    s_leftDelta += dir * LEFT_ENCODER_DIR;
}

static void Encoder_HandleRightA(void)
{
    uint8_t a = Encoder_ReadLevel(ENC_R_A_PORT, ENC_R_A_PIN);
    uint8_t b;
    int32_t dir;

#if ENCODER_DIAG_ENABLE
    s_rightIsrCount++;
#endif
    if (a == s_rightALast)
    {
#if ENCODER_DIAG_ENABLE
        s_rightSameAIgnored++;
#endif
        return;
    }
    s_rightALast = a;

#if ENCODER_COUNT_A_RISE_ONLY
    if (a == 0U)
    {
        return;
    }
#endif

    b = Encoder_ReadLevel(ENC_R_B_PORT, ENC_R_B_PIN);
    dir = (a == b) ? 1 : -1;
    s_rightDelta += dir * RIGHT_ENCODER_DIR;
}

static void Encoder_ServicePort(GPIO_Regs *port)
{
    uint32_t mask = 0U;
    uint32_t status;

    if (ENC_L_A_PORT == port)
    {
        mask |= ENC_L_A_PIN;
    }
    if (ENC_R_A_PORT == port)
    {
        mask |= ENC_R_A_PIN;
    }
    if (mask == 0U)
    {
        return;
    }

    status = DL_GPIO_getEnabledInterruptStatus(port, mask);
    DL_GPIO_clearInterruptStatus(port, status);

    if ((ENC_L_A_PORT == port) && ((status & ENC_L_A_PIN) != 0U))
    {
#if ENCODER_DIAG_ENABLE
        s_leftStatusCount++;
#endif
        Encoder_HandleLeftA();
    }
    if ((ENC_R_A_PORT == port) && ((status & ENC_R_A_PIN) != 0U))
    {
#if ENCODER_DIAG_ENABLE
        s_rightStatusCount++;
#endif
        Encoder_HandleRightA();
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
    {
        case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
            Encoder_ServicePort(GPIOB);
            break;

        case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
            Encoder_ServicePort(GPIOA);
            break;

        default:
            break;
    }
}
