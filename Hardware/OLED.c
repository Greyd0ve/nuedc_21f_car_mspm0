#include "OLED.h"
#include "Board_Config.h"
#include <string.h>

#define OLED_I2C_ADDRESS              (0x3CU)
#define OLED_WIDTH                    (128U)
#define OLED_HEIGHT                   (64U)
#define OLED_PAGE_COUNT               (8U)
#define OLED_I2C_PACKET_MAX           (8U)
#define OLED_I2C_PAYLOAD_MAX          (OLED_I2C_PACKET_MAX - 1U)
#define OLED_I2C_TIMEOUT              (100000U)
#define OLED_CONTROL_COMMAND          (0x00U)
#define OLED_CONTROL_DATA             (0x40U)
#define OLED_SPI_DELAY_CYCLES         (4U)

static uint8_t s_oledBuffer[OLED_WIDTH * OLED_PAGE_COUNT];
static uint8_t s_oledReady = 0U;

#if !BOARD_OLED_USE_H8_SPI
static uint32_t s_i2cErrataDelayCycles = 3U;
#endif

static const uint8_t s_font5x7[95][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},
    {0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00},{0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},
    {0x40,0x40,0x40,0x40,0x40},{0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7F},
    {0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},
    {0x7F,0x10,0x28,0x44,0x00},{0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},
    {0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},{0x7C,0x14,0x14,0x14,0x08},
    {0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},
    {0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},{0x00,0x00,0x7F,0x00,0x00},
    {0x00,0x41,0x36,0x08,0x00},{0x10,0x08,0x08,0x10,0x08}
};

#if BOARD_OLED_USE_H8_SPI
static void OLED_H8_GPIOInit(void)
{
    DL_GPIO_initDigitalOutput(OLED_H8_SCL_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_H8_SDA_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_H8_RES_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_H8_DC_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_H8_CS_IOMUX);

    DL_GPIO_clearPins(OLED_H8_PORT, OLED_H8_SCL_PIN);
    DL_GPIO_setPins(OLED_H8_PORT, OLED_H8_SDA_PIN |
                                  OLED_H8_RES_PIN |
                                  OLED_H8_DC_PIN |
                                  OLED_H8_CS_PIN);
    DL_GPIO_enableOutput(OLED_H8_PORT, OLED_H8_PIN_MASK);
}

static void OLED_H8_Reset(void)
{
    DL_GPIO_clearPins(OLED_H8_PORT, OLED_H8_RES_PIN);
    delay_cycles(320000U);
    DL_GPIO_setPins(OLED_H8_PORT, OLED_H8_RES_PIN);
    delay_cycles(320000U);
}

static void OLED_H8_WriteByte(uint8_t data)
{
    uint8_t mask;

    for (mask = 0x80U; mask != 0U; mask >>= 1U)
    {
        if ((data & mask) != 0U)
        {
            DL_GPIO_setPins(OLED_H8_PORT, OLED_H8_SDA_PIN);
        }
        else
        {
            DL_GPIO_clearPins(OLED_H8_PORT, OLED_H8_SDA_PIN);
        }

        delay_cycles(OLED_SPI_DELAY_CYCLES);
        DL_GPIO_setPins(OLED_H8_PORT, OLED_H8_SCL_PIN);
        delay_cycles(OLED_SPI_DELAY_CYCLES);
        DL_GPIO_clearPins(OLED_H8_PORT, OLED_H8_SCL_PIN);
    }
}

static uint8_t OLED_WriteControlBytes(uint8_t control, const uint8_t *data, uint16_t len)
{
    uint16_t offset;

    if (data == NULL && len != 0U)
    {
        return 0U;
    }

    if (control == OLED_CONTROL_COMMAND)
    {
        DL_GPIO_clearPins(OLED_H8_PORT, OLED_H8_DC_PIN);
    }
    else
    {
        DL_GPIO_setPins(OLED_H8_PORT, OLED_H8_DC_PIN);
    }

    DL_GPIO_clearPins(OLED_H8_PORT, OLED_H8_CS_PIN);
    for (offset = 0U; offset < len; offset++)
    {
        OLED_H8_WriteByte(data[offset]);
    }
    DL_GPIO_setPins(OLED_H8_PORT, OLED_H8_CS_PIN);

    return 1U;
}

static uint8_t OLED_WriteCommands(const uint8_t *cmds, uint16_t len)
{
    return OLED_WriteControlBytes(OLED_CONTROL_COMMAND, cmds, len);
}
#else
static void OLED_CalcI2CDelay(void)
{
    DL_I2C_ClockConfig clockConfig;
    uint32_t clockHz = CPUCLK_FREQ;

    DL_I2C_getClockConfig(OLED_I2C_INST, &clockConfig);
    if (clockConfig.clockSel == DL_I2C_CLOCK_MFCLK)
    {
        clockHz = 4000000U;
    }

    if (clockHz > 0U)
    {
        s_i2cErrataDelayCycles = 3U * ((uint32_t)clockConfig.divideRatio + 1U) *
                                  (CPUCLK_FREQ / clockHz);
        if (s_i2cErrataDelayCycles == 0U)
        {
            s_i2cErrataDelayCycles = 3U;
        }
    }
}

static uint8_t OLED_WaitIdle(void)
{
    uint32_t timeout = OLED_I2C_TIMEOUT;

    while (timeout > 0U)
    {
        uint32_t status = DL_I2C_getControllerStatus(OLED_I2C_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)
        {
            return 0U;
        }
        if ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U)
        {
            return 1U;
        }
        timeout--;
    }

    return 0U;
}

static uint8_t OLED_WaitDone(void)
{
    uint32_t timeout = OLED_I2C_TIMEOUT;

    while (timeout > 0U)
    {
        uint32_t status = DL_I2C_getControllerStatus(OLED_I2C_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_BUSY) == 0U)
        {
            return (uint8_t)(((status & DL_I2C_CONTROLLER_STATUS_ERROR) == 0U) ? 1U : 0U);
        }
        timeout--;
    }

    return 0U;
}

static uint8_t OLED_WritePacket(const uint8_t *packet, uint8_t len)
{
    uint16_t written;

    if (len == 0U || len > OLED_I2C_PACKET_MAX)
    {
        return 0U;
    }

    if (!OLED_WaitIdle())
    {
        DL_I2C_resetControllerTransfer(OLED_I2C_INST);
    }

    DL_I2C_resetControllerTransfer(OLED_I2C_INST);
    written = DL_I2C_fillControllerTXFIFO(OLED_I2C_INST, (uint8_t *)packet, len);
    if (written != len)
    {
        return 0U;
    }

    DL_I2C_startControllerTransfer(OLED_I2C_INST, OLED_I2C_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, len);
    delay_cycles(s_i2cErrataDelayCycles);

    return OLED_WaitDone();
}

static uint8_t OLED_WriteControlBytes(uint8_t control, const uint8_t *data, uint16_t len)
{
    uint8_t packet[OLED_I2C_PACKET_MAX];
    uint16_t offset = 0U;

    while (offset < len)
    {
        uint8_t chunk = (uint8_t)(len - offset);
        uint8_t i;

        if (chunk > OLED_I2C_PAYLOAD_MAX)
        {
            chunk = OLED_I2C_PAYLOAD_MAX;
        }

        packet[0] = control;
        for (i = 0U; i < chunk; i++)
        {
            packet[i + 1U] = data[offset + i];
        }

        if (!OLED_WritePacket(packet, (uint8_t)(chunk + 1U)))
        {
            return 0U;
        }

        offset += chunk;
    }

    return 1U;
}

static uint8_t OLED_WriteCommands(const uint8_t *cmds, uint16_t len)
{
    return OLED_WriteControlBytes(OLED_CONTROL_COMMAND, cmds, len);
}
#endif

static void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t on)
{
    uint16_t index;
    uint8_t mask;

    if (x >= OLED_WIDTH || y >= OLED_HEIGHT)
    {
        return;
    }

    index = (uint16_t)x + ((uint16_t)(y >> 3U) * OLED_WIDTH);
    mask = (uint8_t)(1U << (y & 0x07U));

    if (on)
    {
        s_oledBuffer[index] |= mask;
    }
    else
    {
        s_oledBuffer[index] &= (uint8_t)~mask;
    }
}

static const uint8_t *OLED_GetFont(char c)
{
    if (c < ' ' || c > '~')
    {
        c = '?';
    }
    return s_font5x7[(uint8_t)c - (uint8_t)' '];
}

static void OLED_DrawChar6x8(uint8_t x, uint8_t y, char c)
{
    const uint8_t *bitmap = OLED_GetFont(c);
    uint8_t col;

    for (col = 0U; col < 5U; col++)
    {
        uint8_t bits = bitmap[col];
        uint8_t row;

        for (row = 0U; row < 7U; row++)
        {
            OLED_DrawPixel((uint8_t)(x + col), (uint8_t)(y + row),
                (uint8_t)((bits >> row) & 0x01U));
        }
    }
}

static void OLED_DrawChar8x16(uint8_t x, uint8_t y, char c)
{
    const uint8_t *bitmap = OLED_GetFont(c);
    uint8_t col;

    for (col = 0U; col < 5U; col++)
    {
        uint8_t bits = bitmap[col];
        uint8_t row;

        for (row = 0U; row < 7U; row++)
        {
            uint8_t on = (uint8_t)((bits >> row) & 0x01U);
            uint8_t px = (uint8_t)(x + 1U + col);
            uint8_t py = (uint8_t)(y + 1U + (row * 2U));

            OLED_DrawPixel(px, py, on);
            OLED_DrawPixel(px, (uint8_t)(py + 1U), on);
        }
    }
}

void OLED_Init(void)
{
    static const uint8_t initCmds[] = {
        0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
        0x00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB,
        0x40, 0x8D, 0x14, 0xAF
    };

#if BOARD_OLED_USE_H8_SPI
    OLED_H8_GPIOInit();
    OLED_H8_Reset();
#else
    OLED_CalcI2CDelay();
#endif

    memset(s_oledBuffer, 0, sizeof(s_oledBuffer));
    s_oledReady = OLED_WriteCommands(initCmds, (uint16_t)sizeof(initCmds));

    OLED_Clear();
    OLED_ShowString(0, 0, "E-Car MSPM0", OLED_8X16);
#if BOARD_OLED_USE_H8_SPI
    OLED_ShowString(0, 16, "OLED H8 SPI", OLED_8X16);
    OLED_ShowString(0, 32, "SCL PB9 SDA PB8", OLED_8X16);
#else
    OLED_ShowString(0, 16, "OLED I2C OK", OLED_8X16);
    OLED_ShowString(0, 32, "SCL PA1 SDA PA0", OLED_8X16);
#endif
    OLED_ShowString(0, 48, "Motor OFF", OLED_8X16);
    OLED_Update();
}

void OLED_Clear(void)
{
    memset(s_oledBuffer, 0, sizeof(s_oledBuffer));
}

void OLED_Update(void)
{
    static const uint8_t addrCmds[] = {0x21, 0x00, 0x7F, 0x22, 0x00, 0x07};

    if (!s_oledReady)
    {
        return;
    }

    if (!OLED_WriteCommands(addrCmds, (uint16_t)sizeof(addrCmds)))
    {
        s_oledReady = 0U;
        return;
    }

    if (!OLED_WriteControlBytes(OLED_CONTROL_DATA, s_oledBuffer, (uint16_t)sizeof(s_oledBuffer)))
    {
        s_oledReady = 0U;
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t fontSize)
{
    uint8_t step = (fontSize == OLED_8X16) ? 8U : 6U;

    if (str == NULL)
    {
        return;
    }

    while (*str != '\0')
    {
        if (x > (OLED_WIDTH - step))
        {
            break;
        }

        if (fontSize == OLED_8X16)
        {
            OLED_DrawChar8x16(x, y, *str);
        }
        else
        {
            OLED_DrawChar6x8(x, y, *str);
        }

        x = (uint8_t)(x + step);
        str++;
    }
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t fontSize)
{
    char buf[11];
    uint8_t i;

    if (len == 0U)
    {
        return;
    }
    if (len > 10U)
    {
        len = 10U;
    }

    buf[len] = '\0';
    for (i = 0U; i < len; i++)
    {
        uint8_t pos = (uint8_t)(len - 1U - i);
        buf[pos] = (char)('0' + (num % 10U));
        num /= 10U;
    }

    OLED_ShowString(x, y, buf, fontSize);
}

void OLED_ShowSignedNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t fontSize)
{
    uint32_t absNum;
    uint8_t step = (fontSize == OLED_8X16) ? 8U : 6U;

    if (num < 0)
    {
        OLED_ShowString(x, y, "-", fontSize);
        x = (uint8_t)(x + step);
        absNum = (uint32_t)(-num);
    }
    else
    {
        absNum = (uint32_t)num;
    }

    OLED_ShowNum(x, y, absNum, len, fontSize);
}
