#include "DebugSerial.h"
#include "ti_msp_dl_config.h"
#include "cmsis_compiler.h"
#include <stdint.h>
#include <stdarg.h>

#define DEBUG_RX_BUF_SIZE 128U
#define DEBUG_TX_BUF_SIZE 512U
#define DEBUG_PING_BUF_SIZE 64U

static volatile uint8_t s_rxBuf[DEBUG_RX_BUF_SIZE];
static volatile uint8_t s_rxHead = 0U;
static volatile uint8_t s_rxTail = 0U;
static volatile uint32_t s_rxOverflow = 0U;

static volatile uint8_t s_txBuf[DEBUG_TX_BUF_SIZE];
static volatile uint16_t s_txHead = 0U;
static volatile uint16_t s_txTail = 0U;
static volatile uint32_t s_txOverflow = 0U;
static volatile uint16_t s_txHighWaterMark = 0U;

static uint16_t DebugSerial_TxNextIndex(uint16_t index)
{
    index++;
    return (index >= DEBUG_TX_BUF_SIZE) ? 0U : index;
}

static uint16_t DebugSerial_TxPendingFromIndexes(uint16_t head, uint16_t tail)
{
    if (head >= tail)
    {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(DEBUG_TX_BUF_SIZE - tail + head);
}

static void DebugSerial_PushRx(uint8_t byte)
{
    uint8_t next = (uint8_t)(s_rxHead + 1U);
    if (next >= DEBUG_RX_BUF_SIZE) next = 0U;
    if (next == s_rxTail) { s_rxOverflow++; return; }
    s_rxBuf[s_rxHead] = byte;
    s_rxHead = next;
}

void UART_DEBUG_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_DEBUG_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_DEBUG_INST))
            {
                DebugSerial_PushRx(DL_UART_Main_receiveData(UART_DEBUG_INST));
            }
            break;

        case DL_UART_MAIN_IIDX_TX:
            while ((s_txTail != s_txHead) &&
                   !DL_UART_Main_isTXFIFOFull(UART_DEBUG_INST))
            {
                uint8_t byte = s_txBuf[s_txTail];
                s_txTail = DebugSerial_TxNextIndex(s_txTail);
                DL_UART_Main_transmitData(UART_DEBUG_INST, byte);
            }

            if (s_txTail == s_txHead)
            {
                DL_UART_Main_disableInterrupt(
                    UART_DEBUG_INST, DL_UART_MAIN_INTERRUPT_TX);
            }
            break;

        default:
            break;
    }
}

void DebugSerial_Init(void)
{
    s_rxHead = 0U;
    s_rxTail = 0U;
    s_rxOverflow = 0U;
    s_txHead = 0U;
    s_txTail = 0U;
    s_txOverflow = 0U;
    s_txHighWaterMark = 0U;
    DL_UART_Main_disableInterrupt(UART_DEBUG_INST, DL_UART_MAIN_INTERRUPT_TX);
    NVIC_ClearPendingIRQ(UART_DEBUG_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_DEBUG_INST_INT_IRQN);
}

uint32_t DebugSerial_GetRxOverflowCount(void)
{
    return s_rxOverflow;
}

void DebugSerial_SendByte(uint8_t byte)
{
    uint16_t next;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    next = DebugSerial_TxNextIndex(s_txHead);
    if (next == s_txTail)
    {
        s_txOverflow++;
    }
    else
    {
        uint16_t pending;

        s_txBuf[s_txHead] = byte;
        s_txHead = next;
        pending = DebugSerial_TxPendingFromIndexes(s_txHead, s_txTail);
        if (pending > s_txHighWaterMark)
        {
            s_txHighWaterMark = pending;
        }
        DL_UART_Main_enableInterrupt(
            UART_DEBUG_INST, DL_UART_MAIN_INTERRUPT_TX);
    }

    if (primask == 0U)
    {
        __enable_irq();
    }
}

void DebugSerial_SendString(const char *str)
{
    while (*str) { DebugSerial_SendByte((uint8_t)*str++); }
}

uint8_t DebugSerial_ReadByte(uint8_t *byte)
{
    if (s_rxHead == s_rxTail) return 0U;
    *byte = s_rxBuf[s_rxTail];
    s_rxTail++;
    if (s_rxTail >= DEBUG_RX_BUF_SIZE) s_rxTail = 0U;
    return 1U;
}

uint32_t DebugSerial_GetTxOverflowCount(void)
{
    uint32_t overflow;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    overflow = s_txOverflow;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return overflow;
}

uint16_t DebugSerial_GetTxPendingCount(void)
{
    uint16_t pending;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    pending = DebugSerial_TxPendingFromIndexes(s_txHead, s_txTail);
    if (primask == 0U)
    {
        __enable_irq();
    }

    return pending;
}

uint16_t DebugSerial_GetTxHighWaterMark(void)
{
    uint16_t highWater;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    highWater = s_txHighWaterMark;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return highWater;
}

static void DebugSerial_SendNumU(uint32_t num, uint8_t width, uint8_t zeroPad)
{
    uint8_t buf[10];
    uint8_t i = 0U;
    do { buf[i++] = (uint8_t)((num % 10U) + '0'); num /= 10U; } while (num > 0U);
    while (zeroPad && i < width) { DebugSerial_SendByte('0'); width--; }
    while (i > 0U) { DebugSerial_SendByte(buf[--i]); }
}

void DebugSerial_Printf(const char *format, ...)
{
    va_list args;
    const char *p;
    uint8_t zeroPad, width;
    int32_t ival;

    va_start(args, format);
    p = format;
    while (*p)
    {
        if (*p != '%') { DebugSerial_SendByte((uint8_t)*p++); continue; }
        p++;
        zeroPad = 0U; width = 0U;
        if (*p == '0') { zeroPad = 1U; p++; }
        while (*p >= '0' && *p <= '9') { width = (uint8_t)(width * 10U + (uint8_t)(*p - '0')); p++; }
        if (*p == 'l') p++;
        switch (*p)
        {
        case 'u': DebugSerial_SendNumU(va_arg(args, uint32_t), width, zeroPad); break;
        case 'd':
            ival = va_arg(args, int32_t);
            if (ival < 0) { DebugSerial_SendByte('-'); ival = -ival; }
            DebugSerial_SendNumU((uint32_t)ival, width, zeroPad);
            break;
        case 'x':
        case 'X':
            { uint32_t xv = va_arg(args, uint32_t); uint8_t xi; uint8_t xb[8];
              for (xi = 0U; xi < 8U; xi++) { uint8_t n = (uint8_t)((xv >> (xi * 4U)) & 0xFU); xb[7U - xi] = n < 10U ? (uint8_t)('0' + n) : (uint8_t)((*p == 'X' ? 'A' : 'a') + n - 10U); }
              xi = 0U; while (xi < 7U && xb[xi] == '0' && !zeroPad) xi++; if (width > 0U && (7U - xi) < width) { xi = 7U - width; if (xi > 7U) xi = 7U; }
              while (xi < 8U) DebugSerial_SendByte(xb[xi++]); }
            break;
        case 'c': DebugSerial_SendByte((uint8_t)va_arg(args, int32_t)); break;
        case 's': { const char *sp = va_arg(args, const char *); while (*sp) DebugSerial_SendByte((uint8_t)*sp++); } break;
        case '%': DebugSerial_SendByte('%'); break;
        default: DebugSerial_SendByte('%'); DebugSerial_SendByte((uint8_t)*p); break;
        }
        p++;
    }
    va_end(args);
}

void DebugSerial_Task10ms(void)
{
    static char buf[DEBUG_PING_BUF_SIZE];
    static uint8_t idx = 0U;
    uint8_t byte;

    while (DebugSerial_ReadByte(&byte))
    {
        if (byte == '\r' || byte == '\n')
        {
            if (idx > 0U) { buf[idx] = '\0'; idx = 0U; }
            continue;
        }
        if (idx >= sizeof(buf) - 1U) { idx = 0U; continue; }
        if (byte == '[') { idx = 0U; }
        buf[idx++] = (char)byte;
        if (byte == ']')
        {
            buf[idx] = '\0';
            idx = 0U;
            if (buf[0] == '[' && buf[1] == 'p' && buf[2] == 'i' && buf[3] == 'n'
                && buf[4] == 'g' && buf[5] == ']')
            {
                DebugSerial_SendString("[pong]\r\n");
            }
        }
    }
}
