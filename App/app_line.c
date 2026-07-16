#include "app_line.h"
#include "app_config.h"
#include "app_car_state.h"
#include "Grayscale.h"
#include <stdint.h>

/* 8 路灰度循迹处理，硬件通过 CD4051 多路复用读取。
 * 若传感器物理方向装反，可通过 g_lineReverseOrderF 反转逻辑顺序，无需改线。
 */
#ifndef GRAYSCALE_CHANNELS
#define GRAYSCALE_CHANNELS 8U
#endif

extern volatile float g_lineBlackLevelF;
extern volatile float g_lineReverseOrderF;
extern volatile float g_lineTurnSign;
extern volatile float g_lineKp;
extern volatile float g_lineKd;
extern volatile float g_lineTurnLimit;
extern volatile float g_lineFilterAlpha;

extern volatile int16_t g_lineError;
extern volatile uint8_t g_lineValid;
extern volatile uint8_t g_lineMask;
extern volatile uint8_t g_lineRawMask;
extern volatile uint8_t g_lineBlackCount;
extern volatile uint8_t g_lineBadMaskCount;
extern volatile uint8_t g_lineZeroMaskCount;
extern volatile int8_t g_lastLineDir;
extern volatile uint16_t g_lineLostMs;

static volatile float g_lineErrorFiltered = 0.0f;
static volatile float g_lineLastCtrlError = 0.0f;
static int16_t s_lastValidError = 0;
static uint8_t s_hasValidError = 0U;
static uint8_t s_badMaskCount = 0U;
static uint8_t s_zeroMaskCount = 0U;

static float App_Line_LimitFloat(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

static void App_Line_HoldLastError(void)
{
    g_lineError = s_lastValidError;
}

void App_Line_GPIOForceInit(void)
{
    Grayscale_Init();
}

void App_Line_ResetState(void)
{
    g_lineLostMs = 0;
    g_lineBlackCount = 0U;
    g_lineBadMaskCount = 0U;
    g_lineZeroMaskCount = 0U;
    g_lineErrorFiltered = 0.0f;
    g_lineLastCtrlError = 0.0f;
    s_lastValidError = 0;
    s_hasValidError = 0U;
    s_badMaskCount = 0U;
    s_zeroMaskCount = 0U;
}

void App_Line_Update(void)
{
    uint8_t raw[GRAYSCALE_CHANNELS];
    static const int16_t weight[GRAYSCALE_CHANNELS] = {-400, -280, -160, -60, 60, 160, 280, 400};
    int32_t sum = 0;
    int16_t count = 0;
    uint8_t mask = 0;
    uint8_t i;
    uint8_t blackLevel;
    uint8_t reverseOrder;

    blackLevel = (g_lineBlackLevelF <= 0.5f) ? 0U : 1U;
    reverseOrder = (g_lineReverseOrderF <= 0.5f) ? 0U : 1U;

    Grayscale_ReadAll(raw);

    g_lineRawMask = 0;
    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        if (raw[i] == blackLevel) g_lineRawMask |= (uint8_t)(1U << i);
    }

    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        uint8_t physicalIndex;
        uint8_t isBlack;

        physicalIndex = reverseOrder ? (uint8_t)(GRAYSCALE_CHANNELS - 1U - i) : i;
        isBlack = (raw[physicalIndex] == blackLevel) ? 1U : 0U;

        if (isBlack)
        {
            mask |= (uint8_t)(1U << i);
            sum += weight[i];
            count++;
        }
    }

    g_lineMask = mask;
    g_lineBlackCount = (uint8_t)count;

    if (count == 0)
    {
        g_lineValid = 0U;
        App_Line_HoldLastError();
        if (s_zeroMaskCount < 255U) s_zeroMaskCount++;
        if (g_lineLostMs <= (uint16_t)(60000U - ECAR_CONTROL_PERIOD_MS))
        {
            g_lineLostMs = (uint16_t)(g_lineLostMs + ECAR_CONTROL_PERIOD_MS);
        }
    }
    else
    {
        float rawError;
        float alpha;

        rawError = (float)sum / (float)count;
        alpha = App_Line_LimitFloat(g_lineFilterAlpha, 0.0f, 1.0f);

        if (!s_hasValidError)
        {
            g_lineErrorFiltered = rawError;
            s_hasValidError = 1U;
        }
        else
        {
            g_lineErrorFiltered = g_lineErrorFiltered * (1.0f - alpha) + rawError * alpha;
        }

        g_lineError = (int16_t)g_lineErrorFiltered;
        s_lastValidError = g_lineError;
        g_lineValid = 1U;
        g_lastLineDir = (g_lineError >= 0) ? 1 : -1;
        g_lineLostMs = 0U;
        s_zeroMaskCount = 0U;
        s_badMaskCount = (uint8_t)(s_badMaskCount / 2U);
    }

    g_lineBadMaskCount = s_badMaskCount;
    g_lineZeroMaskCount = s_zeroMaskCount;
}

float App_Line_CalcTurnCmd(void)
{
    float error;
    float dError;
    float turn;

    error = (float)g_lineError;
    dError = error - g_lineLastCtrlError;
    g_lineLastCtrlError = error;

    turn = (-g_lineTurnSign) * (error * g_lineKp + dError * g_lineKd);

    return App_Line_LimitFloat(turn, -g_lineTurnLimit, g_lineTurnLimit);
}
