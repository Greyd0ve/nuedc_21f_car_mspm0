#include "JY61P_Serial.h"
#include "Board_Config.h"
#include "cmsis_compiler.h"
#include <stdint.h>

#define JY61P_SERIAL_RX_BUF_SIZE 256U

static volatile uint8_t s_rxBuf[JY61P_SERIAL_RX_BUF_SIZE];
static volatile uint16_t s_rxHead = 0U;
static volatile uint16_t s_rxTail = 0U;
static volatile uint32_t s_rxOverflowCount = 0U;
static volatile uint16_t s_rxHighWaterMark = 0U;

static volatile uint32_t s_irqCount = 0U;
static volatile uint32_t s_rxIrqCount = 0U;
static volatile uint32_t s_otherIrqCount = 0U;
static volatile uint32_t s_rxByteCount = 0U;
static volatile uint32_t s_initDiscardCount = 0U;
static volatile uint32_t s_lastInterruptIndex = 0U;

static uint16_t JY61P_Serial_NextIndex(uint16_t index)
{
    index++;
    return (index >= JY61P_SERIAL_RX_BUF_SIZE) ? 0U : index;
}

static uint16_t JY61P_Serial_PendingFromIndexes(uint16_t head, uint16_t tail)
{
    if (head >= tail)
    {
        return (uint16_t)(head - tail);
    }
    return (uint16_t)(JY61P_SERIAL_RX_BUF_SIZE - tail + head);
}

static void JY61P_Serial_PushRx(uint8_t byte)
{
    uint16_t next = JY61P_Serial_NextIndex(s_rxHead);
    if (next == s_rxTail)
    {
        s_rxOverflowCount++;
        return;
    }
    s_rxBuf[s_rxHead] = byte;
    s_rxHead = next;
    {
        uint16_t pending = JY61P_Serial_PendingFromIndexes(s_rxHead, s_rxTail);
        if (pending > s_rxHighWaterMark)
        {
            s_rxHighWaterMark = pending;
        }
    }
}

void JY61P_Serial_Init(void)
{
    NVIC_DisableIRQ(JY61P_UART_IRQN);

    s_rxHead = 0U;
    s_rxTail = 0U;
    s_rxOverflowCount = 0U;
    s_rxHighWaterMark = 0U;
    s_irqCount = 0U;
    s_rxIrqCount = 0U;
    s_otherIrqCount = 0U;
    s_rxByteCount = 0U;
    s_initDiscardCount = 0U;
    s_lastInterruptIndex = 0U;

    while (!DL_UART_Main_isRXFIFOEmpty(JY61P_UART_INST))
    {
        (void)DL_UART_Main_receiveData(JY61P_UART_INST);
        s_initDiscardCount++;
    }

    NVIC_ClearPendingIRQ(JY61P_UART_IRQN);
    NVIC_EnableIRQ(JY61P_UART_IRQN);
}

uint8_t JY61P_Serial_ReadByte(uint8_t *byte)
{
    if (byte == 0 || s_rxHead == s_rxTail)
    {
        return 0U;
    }
    *byte = s_rxBuf[s_rxTail];
    s_rxTail = JY61P_Serial_NextIndex(s_rxTail);
    return 1U;
}

uint32_t JY61P_Serial_GetRxOverflowCount(void)
{
    return s_rxOverflowCount;
}

uint16_t JY61P_Serial_GetRxPendingCount(void)
{
    uint16_t pending;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    pending = JY61P_Serial_PendingFromIndexes(s_rxHead, s_rxTail);
    if (primask == 0U) { __enable_irq(); }
    return pending;
}

uint16_t JY61P_Serial_GetRxHighWaterMark(void)
{
    uint16_t highWater;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    highWater = s_rxHighWaterMark;
    if (primask == 0U) { __enable_irq(); }
    return highWater;
}

uint32_t JY61P_Serial_GetIrqCount(void)
{
    return s_irqCount;
}

uint32_t JY61P_Serial_GetRxIrqCount(void)
{
    return s_rxIrqCount;
}

uint32_t JY61P_Serial_GetOtherIrqCount(void)
{
    return s_otherIrqCount;
}

uint32_t JY61P_Serial_GetRxByteCount(void)
{
    return s_rxByteCount;
}

uint32_t JY61P_Serial_GetInitDiscardCount(void)
{
    return s_initDiscardCount;
}

uint32_t JY61P_Serial_GetLastInterruptIndex(void)
{
    return s_lastInterruptIndex;
}

void JY61P_UART_IRQHandler(void)
{
    uint32_t iidx;
    s_irqCount++;
    iidx = DL_UART_Main_getPendingInterrupt(JY61P_UART_INST);
    s_lastInterruptIndex = (uint32_t)iidx;
    switch (iidx)
    {
        case DL_UART_MAIN_IIDX_RX:
            s_rxIrqCount++;
            while (!DL_UART_Main_isRXFIFOEmpty(JY61P_UART_INST))
            {
                JY61P_Serial_PushRx(
                    DL_UART_Main_receiveData(JY61P_UART_INST));
                s_rxByteCount++;
            }
            break;
        default:
            s_otherIrqCount++;
            break;
    }
}
