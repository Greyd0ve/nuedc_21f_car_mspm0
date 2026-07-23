#include "Key.h"
#include "Board_Config.h"

/* 消抖后的按键事件；按键释放时锁存非零键值，并只被消费一次。 */
static volatile uint8_t s_keyNum = 0U;

void Key_Init(void)
{
    s_keyNum = 0U;
}

uint8_t Key_GetNum(void)
{
    /* 前台轮询读取并清除一个消抖后的按键事件。 */
    uint8_t key = s_keyNum;

    if (key != 0U)
    {
        s_keyNum = 0U;
    }
    return key;
}

uint8_t Key_GetState(void)
{
    /* 按键低电平有效；返回第一个按下的键号，未按下返回 0。 */
#if !BOARD_OLED_H8_SPI_OWNS_KEY12
    if (DL_GPIO_readPins(KEY_K1_PORT, KEY_K1_PIN) == 0U)
    {
        return 1U;
    }
    if (DL_GPIO_readPins(KEY_K2_PORT, KEY_K2_PIN) == 0U)
    {
        return 2U;
    }
#endif
    if (DL_GPIO_readPins(KEY_K3_PORT, KEY_K3_PIN) == 0U)
    {
        return 3U;
    }
    if (DL_GPIO_readPins(KEY_K4_PORT, KEY_K4_PIN) == 0U)
    {
        return 4U;
    }
    return 0U;
}

void Key_Tick(void)
{
    static uint8_t count = 0U;
    static uint8_t currState = 0U;
    static uint8_t prevState = 0U;

    /* 每 20ms 采样一次，稳定按下后释放才产生按键事件。 */
    count++;
    if (count >= 20U)
    {
        count = 0U;
        prevState = currState;
        currState = Key_GetState();
        if (currState == 0U && prevState != 0U)
        {
            s_keyNum = prevState;
        }
    }
}
