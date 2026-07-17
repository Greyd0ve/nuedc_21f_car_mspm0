#include "Serial.h"
#include "Board_Config.h"
#include <stdarg.h>
#include <stdint.h>

#define SERIAL_RX_BUF_SIZE 128U /* 串口接收环形缓冲区大小，单位字节 */

/* 中断驱动的 RX 环形缓冲区。
 * TX 保持阻塞发送，只允许前台短帧调用，不放入高频控制或定时器 ISR。
 */
static volatile uint8_t s_rxBuf[SERIAL_RX_BUF_SIZE];
static volatile uint16_t s_rxHead = 0U;
static volatile uint16_t s_rxTail = 0U;
static volatile uint8_t s_rxFlag = 0U;
static volatile uint8_t s_rxData = 0U;
static volatile uint32_t s_rxOverflowCount = 0U;

static uint16_t Serial_NextIndex(uint16_t index)
{
    index++;
    return (index >= SERIAL_RX_BUF_SIZE) ? 0U : index;
}

static void Serial_PushRx(uint8_t byte)
{
    uint16_t next = Serial_NextIndex(s_rxHead);

    if (next == s_rxTail)
    {
        /* 缓冲区满时丢弃新字节，并累计溢出次数供遥测查看。 */
        s_rxOverflowCount++;
        return;
    }

    s_rxBuf[s_rxHead] = byte;
    s_rxHead = next;
    s_rxData = byte;
    s_rxFlag = 1U;
}

void Serial_Init(void)
{
    /* 使能 UART RX 中断前，先复位软件缓冲区状态。 */
    s_rxHead = 0U;
    s_rxTail = 0U;
    s_rxFlag = 0U;
    s_rxData = 0U;
    s_rxOverflowCount = 0U;

    NVIC_ClearPendingIRQ(SERIAL_UART_IRQN);
    NVIC_EnableIRQ(SERIAL_UART_IRQN);
}

void Serial_SendByte(uint8_t byte)
{
    /* 阻塞式 TX 适合前台短遥测帧，逻辑简单可预测。 */
    DL_UART_Main_transmitDataBlocking(SERIAL_UART_INST, byte);
}

void Serial_SendArray(const uint8_t *array, uint16_t length)
{
    uint16_t i;

    if (array == 0)
    {
        return;
    }

    for (i = 0U; i < length; i++)
    {
        Serial_SendByte(array[i]);
    }
}

static uint8_t Serial_IsDigit(char ch)
{
    return (uint8_t)((ch >= '0') && (ch <= '9'));
}

static void Serial_SendRepeat(char ch, uint8_t count)
{
    while (count > 0U)
    {
        Serial_SendByte((uint8_t)ch);
        count--;
    }
}

static void Serial_SendUnsignedBase(uint32_t value,
                                    uint32_t base,
                                    uint8_t width,
                                    uint8_t zeroPad,
                                    uint8_t upperCase)
{
    char buf[11];
    uint8_t len = 0U;
    char padChar = zeroPad ? '0' : ' ';

    if ((base != 10U) && (base != 16U))
    {
        base = 10U;
    }

    do
    {
        uint32_t digit = value % base;
        value /= base;

        if (digit < 10U)
        {
            buf[len] = (char)('0' + digit);
        }
        else
        {
            buf[len] = (char)((upperCase ? 'A' : 'a') + (digit - 10U));
        }

        len++;
    } while ((value != 0U) && (len < sizeof(buf)));

    if (width > len)
    {
        Serial_SendRepeat(padChar, (uint8_t)(width - len));
    }

    while (len > 0U)
    {
        len--;
        Serial_SendByte((uint8_t)buf[len]);
    }
}

static void Serial_SendSignedDecimal(int32_t value, uint8_t width, uint8_t zeroPad)
{
    uint32_t absValue;

    if (value < 0)
    {
        Serial_SendByte((uint8_t)'-');

        if (width > 0U)
        {
            width--;
        }

        absValue = (uint32_t)(-(value + 1)) + 1U;
    }
    else
    {
        absValue = (uint32_t)value;
    }

    Serial_SendUnsignedBase(absValue, 10U, width, zeroPad, 0U);
}


void Serial_SendString(const char *string)
{
    if (string == 0)
    {
        return;
    }

    while (*string != '\0')
    {
        Serial_SendByte((uint8_t)*string);
        string++;
    }
}

void Serial_SendNumber(uint32_t number, uint8_t length)
{
    char buf[11];
    uint8_t i;

    if (length == 0U)
    {
        return;
    }
    if (length > 10U)
    {
        length = 10U;
    }

    for (i = 0U; i < length && i < 10U; i++)
    {
        buf[length - 1U - i] = (char)('0' + (number % 10U));
        number /= 10U;
    }
    buf[length] = '\0';
    Serial_SendString(buf);
}

void Serial_Printf(const char *format, ...)
{
    va_list args;

    if (format == 0)
    {
        return;
    }

    va_start(args, format);

    while (*format != '\0')
    {
        uint8_t zeroPad = 0U;
        uint8_t width = 0U;
        uint8_t longFlag = 0U;
        char spec;

        if (*format != '%')
        {
            Serial_SendByte((uint8_t)*format);
            format++;
            continue;
        }

        format++;

        if (*format == '\0')
        {
            break;
        }

        if (*format == '%')
        {
            Serial_SendByte((uint8_t)'%');
            format++;
            continue;
        }

        if (*format == '0')
        {
            zeroPad = 1U;
            format++;
        }

        while (Serial_IsDigit(*format))
        {
            uint16_t nextWidth = (uint16_t)width * 10U + (uint16_t)(*format - '0');

            if (nextWidth > 255U)
            {
                width = 255U;
            }
            else
            {
                width = (uint8_t)nextWidth;
            }

            format++;
        }

        if (*format == 'l')
        {
            longFlag = 1U;
            format++;
        }

        spec = *format;
        if (spec == '\0')
        {
            break;
        }
        format++;

        switch (spec)
        {
            case 'd':
            {
                int32_t value;

                if (longFlag)
                {
                    value = (int32_t)va_arg(args, long);
                }
                else
                {
                    value = (int32_t)va_arg(args, int);
                }

                Serial_SendSignedDecimal(value, width, zeroPad);
                break;
            }

            case 'u':
            {
                uint32_t value;

                if (longFlag)
                {
                    value = (uint32_t)va_arg(args, unsigned long);
                }
                else
                {
                    value = (uint32_t)va_arg(args, unsigned int);
                }

                Serial_SendUnsignedBase(value, 10U, width, zeroPad, 0U);
                break;
            }

            case 'x':
            {
                uint32_t value;

                if (longFlag)
                {
                    value = (uint32_t)va_arg(args, unsigned long);
                }
                else
                {
                    value = (uint32_t)va_arg(args, unsigned int);
                }

                Serial_SendUnsignedBase(value, 16U, width, zeroPad, 0U);
                break;
            }

            case 'X':
            {
                uint32_t value;

                if (longFlag)
                {
                    value = (uint32_t)va_arg(args, unsigned long);
                }
                else
                {
                    value = (uint32_t)va_arg(args, unsigned int);
                }

                Serial_SendUnsignedBase(value, 16U, width, zeroPad, 1U);
                break;
            }

            case 'c':
            {
                int value = va_arg(args, int);
                Serial_SendByte((uint8_t)value);
                break;
            }

            case 's':
            {
                const char *str = va_arg(args, const char *);

                if (str == 0)
                {
                    str = "(null)";
                }

                Serial_SendString(str);
                break;
            }

            default:
            {
                Serial_SendByte((uint8_t)'%');

                if (zeroPad)
                {
                    Serial_SendByte((uint8_t)'0');
                }

                if (width > 0U)
                {
                    Serial_SendUnsignedBase(width, 10U, 0U, 0U, 0U);
                }

                if (longFlag)
                {
                    Serial_SendByte((uint8_t)'l');
                }

                Serial_SendByte((uint8_t)spec);
                break;
            }
        }
    }

    va_end(args);
}

uint8_t Serial_GetRxFlag(void)
{
    return (s_rxHead != s_rxTail) ? 1U : s_rxFlag;
}

uint8_t Serial_GetRxData(void)
{
    uint8_t byte = 0U;

    (void)Serial_ReadByte(&byte);
    return byte;
}

uint8_t Serial_ReadByte(uint8_t *byte)
{
    if (byte == 0 || s_rxHead == s_rxTail)
    {
        return 0U;
    }

    /* 前台单消费者读取；ISR 是唯一生产者。 */
    *byte = s_rxBuf[s_rxTail];
    s_rxTail = Serial_NextIndex(s_rxTail);
    s_rxFlag = (s_rxHead != s_rxTail) ? 1U : 0U;
    return 1U;
}

uint32_t Serial_GetRxOverflowCount(void)
{
    return s_rxOverflowCount;
}

void UART_DEBUG_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(SERIAL_UART_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            /* ISR 只把 FIFO 搬进环形缓冲区，协议解析留给前台执行。 */
            while (!DL_UART_Main_isRXFIFOEmpty(SERIAL_UART_INST))
            {
                Serial_PushRx(DL_UART_Main_receiveData(SERIAL_UART_INST));
            }
            break;

        default:
            break;
    }
}
