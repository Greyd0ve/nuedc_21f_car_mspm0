#include "Grayscale.h"
#include "Board_Config.h"

/* 通过 CD4051 读取 8 路灰度输入。
 * AD0..AD2 选择通道，OUT 作为数字电平采样。
 */
static void Grayscale_SetAddress(uint8_t channel)
{
    if ((channel & 0x01U) != 0U)
    {
        DL_GPIO_setPins(GRAYSCALE_AD0_PORT, GRAYSCALE_AD0_PIN);
    }
    else
    {
        DL_GPIO_clearPins(GRAYSCALE_AD0_PORT, GRAYSCALE_AD0_PIN);
    }

    if ((channel & 0x02U) != 0U)
    {
        DL_GPIO_setPins(GRAYSCALE_AD1_PORT, GRAYSCALE_AD1_PIN);
    }
    else
    {
        DL_GPIO_clearPins(GRAYSCALE_AD1_PORT, GRAYSCALE_AD1_PIN);
    }

    if ((channel & 0x04U) != 0U)
    {
        DL_GPIO_setPins(GRAYSCALE_AD2_PORT, GRAYSCALE_AD2_PIN);
    }
    else
    {
        DL_GPIO_clearPins(GRAYSCALE_AD2_PORT, GRAYSCALE_AD2_PIN);
    }

    /* 地址切换后等待模拟开关输出和输入缓冲稳定。 */
    delay_cycles(160U);
}

void Grayscale_Init(void)
{
#if BOARD_OLED_H8_SPI_OWNS_GRAY_AD1
    /* H8 SPI OLED uses PB10 as OLED_RES, which conflicts with GRAY_AD1. */
    return;
#else
    /* 默认选择通道 0，等待第一次显式读取。 */
    DL_GPIO_clearPins(GRAYSCALE_AD0_PORT, GRAYSCALE_AD0_PIN);
    DL_GPIO_clearPins(GRAYSCALE_AD1_PORT, GRAYSCALE_AD1_PIN);
    DL_GPIO_clearPins(GRAYSCALE_AD2_PORT, GRAYSCALE_AD2_PIN);
#endif
}

uint8_t Grayscale_RawOUT(void)
{
#if BOARD_OLED_H8_SPI_OWNS_GRAY_AD1
    return 0U;
#else
    return (DL_GPIO_readPins(GRAYSCALE_OUT_PORT, GRAYSCALE_OUT_PIN) != 0U) ? 1U : 0U;
#endif
}

uint8_t Grayscale_ReadChannel(uint8_t channel)
{
#if BOARD_OLED_H8_SPI_OWNS_GRAY_AD1
    (void)channel;
    return 0U;
#else
    uint8_t a;
    uint8_t b;
    uint8_t c;

    Grayscale_SetAddress((uint8_t)(channel & 0x07U));
    /* 连续快速采样 3 次并多数表决，降低单次读数噪声。 */
    a = Grayscale_RawOUT();
    delay_cycles(40U);
    b = Grayscale_RawOUT();
    delay_cycles(40U);
    c = Grayscale_RawOUT();

    return (uint8_t)(((uint16_t)a + (uint16_t)b + (uint16_t)c) >= 2U);
#endif
}

void Grayscale_ReadAll(uint8_t raw[GRAYSCALE_CHANNELS])
{
    uint8_t i;

    if (raw == 0)
    {
        return;
    }

#if BOARD_OLED_H8_SPI_OWNS_GRAY_AD1
    for (i = 0U; i < GRAYSCALE_CHANNELS; i++)
    {
        raw[i] = 0U;
    }
#else
    /* 按物理通道顺序读取；逻辑顺序反转由 app_line 处理。 */
    for (i = 0U; i < GRAYSCALE_CHANNELS; i++)
    {
        raw[i] = Grayscale_ReadChannel(i);
    }
#endif
}

uint8_t Grayscale_ReadOne(uint8_t channel)
{
    return Grayscale_ReadChannel(channel);
}
