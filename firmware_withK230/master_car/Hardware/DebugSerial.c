#include "DebugSerial.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdarg.h>

#define DEBUG_RX_BUF_SIZE 128U
#define DEBUG_TX_BUF_SIZE 512U
#define DEBUG_PING_BUF_SIZE 64U

static volatile uint8_t s_rxBuf[DEBUG_RX_BUF_SIZE];
static volatile uint8_t s_rxHead = 0U;
static volatile uint8_t s_rxTail = 0U;
static volatile uint32_t s_rxOverflow = 0U;

static volatile uint8_t  s_txBuf[DEBUG_TX_BUF_SIZE];
static volatile uint16_t s_txHead = 0U;
static volatile uint16_t s_txTail = 0U;
static volatile uint32_t s_txDropCount = 0U;
static DebugSerial_LineHandler_t s_lineHandler = 0;

static uint16_t DebugSerial_NextTxIndex(uint16_t index)
{
    index++;
    return (index >= DEBUG_TX_BUF_SIZE) ? 0U : index;
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
                DL_UART_Main_transmitData(
                    UART_DEBUG_INST, s_txBuf[s_txTail]);
                s_txTail = DebugSerial_NextTxIndex(s_txTail);
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
    s_txDropCount = 0U;
    s_lineHandler = 0;
    DL_UART_Main_disableInterrupt(
        UART_DEBUG_INST, DL_UART_MAIN_INTERRUPT_TX);
    NVIC_ClearPendingIRQ(UART_DEBUG_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_DEBUG_INST_INT_IRQN);
}

void DebugSerial_SetLineHandler(DebugSerial_LineHandler_t handler)
{
    s_lineHandler = handler;
}

uint32_t DebugSerial_GetRxOverflowCount(void)
{
    return s_rxOverflow;
}

void DebugSerial_SendByte(uint8_t byte)
{
    DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, byte);
}

uint8_t DebugSerial_TrySendByte(uint8_t byte)
{
    uint16_t next = DebugSerial_NextTxIndex(s_txHead);

    if (next == s_txTail)
    {
        return 0U;
    }

    s_txBuf[s_txHead] = byte;
    s_txHead = next;
    DL_UART_Main_enableInterrupt(
        UART_DEBUG_INST, DL_UART_MAIN_INTERRUPT_TX);
    return 1U;
}

uint16_t DebugSerial_GetTxFreeBytes(void)
{
    if (s_txHead >= s_txTail)
    {
        return (uint16_t)(DEBUG_TX_BUF_SIZE - 1U -
               (s_txHead - s_txTail));
    }

    return (uint16_t)(s_txTail - s_txHead - 1U);
}

uint8_t DebugSerial_TrySendBuffer(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (!data || (length == 0U)) return 1U;

    if (DebugSerial_GetTxFreeBytes() < length)
    {
        s_txDropCount++;
        return 0U;
    }

    for (i = 0U; i < length; i++)
    {
        uint16_t next = DebugSerial_NextTxIndex(s_txHead);
        s_txBuf[s_txHead] = data[i];
        s_txHead = next;
    }

    DL_UART_Main_enableInterrupt(
        UART_DEBUG_INST, DL_UART_MAIN_INTERRUPT_TX);
    return 1U;
}

uint32_t DebugSerial_GetTxDropCount(void)
{
    return s_txDropCount;
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

static void DebugSerial_SendNumU(uint32_t num, uint8_t width, uint8_t zeroPad)
{
    uint8_t buf[10];
    uint8_t i = 0U;
    do { buf[i++] = (uint8_t)((num % 10U) + '0'); num /= 10U; } while (num > 0U);
    while (zeroPad && i < width) { (void)DebugSerial_TrySendByte('0'); width--; }
    while (i > 0U) { (void)DebugSerial_TrySendByte(buf[--i]); }
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
        if (*p != '%') { (void)DebugSerial_TrySendByte((uint8_t)*p++); continue; }
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
            if (ival < 0) { (void)DebugSerial_TrySendByte('-'); ival = -ival; }
            DebugSerial_SendNumU((uint32_t)ival, width, zeroPad);
            break;
        case 'x':
        case 'X':
            { uint32_t xv = va_arg(args, uint32_t); uint8_t xi; uint8_t xb[8];
              for (xi = 0U; xi < 8U; xi++) { uint8_t n = (uint8_t)((xv >> (xi * 4U)) & 0xFU); xb[7U - xi] = n < 10U ? (uint8_t)('0' + n) : (uint8_t)((*p == 'X' ? 'A' : 'a') + n - 10U); }
              xi = 0U; while (xi < 7U && xb[xi] == '0' && !zeroPad) xi++; if (width > 0U && (7U - xi) < width) { xi = 7U - width; if (xi > 7U) xi = 7U; }
              while (xi < 8U) (void)DebugSerial_TrySendByte(xb[xi++]); }
            break;
        case 'c': (void)DebugSerial_TrySendByte((uint8_t)va_arg(args, int32_t)); break;
        case 's': { const char *sp = va_arg(args, const char *); while (*sp) (void)DebugSerial_TrySendByte((uint8_t)*sp++); } break;
        case '%': (void)DebugSerial_TrySendByte('%'); break;
        default: (void)DebugSerial_TrySendByte('%'); (void)DebugSerial_TrySendByte((uint8_t)*p); break;
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
            if (s_lineHandler != 0)
            {
                s_lineHandler(buf);
            }
            if (buf[0] == '[' && buf[1] == 'p' && buf[2] == 'i' && buf[3] == 'n'
                && buf[4] == 'g' && buf[5] == ']')
            {
                DebugSerial_SendString("[pong]\r\n");
            }
        }
    }
}
