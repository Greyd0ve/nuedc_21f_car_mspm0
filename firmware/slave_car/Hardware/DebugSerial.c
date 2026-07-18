#include "DebugSerial.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdarg.h>

#define DEBUG_RX_BUF_SIZE 128U

static volatile uint8_t s_rxBuf[DEBUG_RX_BUF_SIZE];
static volatile uint8_t s_rxHead = 0U;
static volatile uint8_t s_rxTail = 0U;

static void DebugSerial_PushRx(uint8_t byte)
{
    uint8_t next = (uint8_t)(s_rxHead + 1U);
    if (next >= DEBUG_RX_BUF_SIZE) next = 0U;
    if (next == s_rxTail) return;
    s_rxBuf[s_rxHead] = byte;
    s_rxHead = next;
}

void UART_TUNING_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_TUNING_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_TUNING_INST))
            {
                DebugSerial_PushRx(DL_UART_Main_receiveData(UART_TUNING_INST));
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
    NVIC_ClearPendingIRQ(UART_TUNING_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_TUNING_INST_INT_IRQN);
}

void DebugSerial_SendByte(uint8_t byte)
{
    DL_UART_Main_transmitDataBlocking(UART_TUNING_INST, byte);
}

void DebugSerial_SendString(const char *str)
{
    while (*str)
    {
        DebugSerial_SendByte((uint8_t)*str++);
    }
}

uint8_t DebugSerial_ReadByte(uint8_t *byte)
{
    if (s_rxHead == s_rxTail) return 0U;
    *byte = s_rxBuf[s_rxTail];
    s_rxTail++;
    if (s_rxTail >= DEBUG_RX_BUF_SIZE) s_rxTail = 0U;
    return 1U;
}

static void DebugSerial_SendNumber(uint32_t num, uint8_t len)
{
    uint8_t i;
    uint8_t buf[10];
    for (i = 0U; i < len; i++)
    {
        buf[len - 1U - i] = (uint8_t)((num % 10U) + '0');
        num /= 10U;
    }
    for (i = 0U; i < len; i++) { DebugSerial_SendByte(buf[i]); }
}

void DebugSerial_Printf(const char *format, ...)
{
    va_list args;
    const char *p;
    uint32_t uval;
    int32_t ival;
    const char *sp;

    va_start(args, format);
    p = format;
    while (*p)
    {
        if (*p != '%') { DebugSerial_SendByte((uint8_t)*p++); continue; }
        p++;
        switch (*p)
        {
        case 'u': uval = va_arg(args, uint32_t);
                  { uint8_t n = 0U; uint32_t t = uval; do { n++; t /= 10U; } while (t > 0U); if (n == 0U) n = 1U; DebugSerial_SendNumber(uval, n); }
                  break;
        case 'd': ival = va_arg(args, int32_t);
                  if (ival < 0) { DebugSerial_SendByte('-'); ival = -ival; }
                  { uint8_t n = 0U; int32_t t = ival; do { n++; t /= 10; } while (t != 0); if (n == 0U) n = 1U; DebugSerial_SendNumber((uint32_t)ival, n); }
                  break;
        case 'x': uval = va_arg(args, uint32_t);
                  { uint8_t i2; for (i2 = 8U; i2 > 0U; i2--) { uint8_t nib = (uint8_t)((uval >> ((i2 - 1U) * 4U)) & 0xFU); DebugSerial_SendByte(nib < 10U ? (uint8_t)('0' + nib) : (uint8_t)('A' + nib - 10U)); } }
                  break;
        case 's': sp = va_arg(args, const char *); while (*sp) DebugSerial_SendByte((uint8_t)*sp++); break;
        case '%': DebugSerial_SendByte('%'); break;
        default: DebugSerial_SendByte('%'); DebugSerial_SendByte((uint8_t)*p); break;
        }
        p++;
    }
    va_end(args);
}
