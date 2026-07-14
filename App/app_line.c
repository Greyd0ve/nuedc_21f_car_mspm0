#include "app_line.h"
#include "app_config.h"
#include "app_e_car.h"
#include "Grayscale.h"
#include <stdint.h>

/* 8 路灰度循迹处理，硬件通过 CD4051 多路复用读取。
 * 若传感器物理方向装反，可通过 g_lineReverseOrderF 反转逻辑顺序，无需改线。
 */
#ifndef GRAYSCALE_CHANNELS
#define GRAYSCALE_CHANNELS 8U /* 灰度传感器通道数，默认 8 路 */
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
extern volatile uint8_t g_lineCornerMaskStableCount;
extern volatile int8_t g_lastLineDir;
extern volatile uint16_t g_lineLostMs;

static volatile float g_lineErrorFiltered = 0.0f;
static volatile float g_lineLastCtrlError = 0.0f;
static int16_t s_lastValidError = 0;
static uint8_t s_hasValidError = 0U;
static uint8_t s_badMaskCount = 0U;
static uint8_t s_zeroMaskCount = 0U;
static uint8_t s_cornerMaskStableCount = 0U;

/* 对串口调参后的滤波/转向参数做范围限制。 */
static float App_Line_LimitFloat(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

static uint8_t App_Line_IsContinuousMask(uint8_t mask, uint8_t blackCount)
{
    uint8_t first = 0U;
    uint16_t expected;

    if (blackCount == 0U || blackCount > GRAYSCALE_CHANNELS)
    {
        return 0U;
    }

    /* 正常黑线应形成连续传感器区域，分裂成多个区域时按异常 mask 处理。 */
    while (first < GRAYSCALE_CHANNELS && (mask & (uint8_t)(1U << first)) == 0U)
    {
        first++;
    }

    if ((uint16_t)first + (uint16_t)blackCount > GRAYSCALE_CHANNELS)
    {
        return 0U;
    }

    expected = (uint16_t)(((uint16_t)1U << blackCount) - 1U);
    expected = (uint16_t)(expected << first);

    return (uint8_t)(((uint16_t)mask == expected) ? 1U : 0U);
}

static void App_Line_HoldLastError(void)
{
    /* 短暂无效 mask 时保持上一次有效误差，避免转向命令突变。 */
    g_lineError = s_lastValidError;
}

void App_Line_GPIOForceInit(void)
{
    Grayscale_Init();
}

void App_Line_ResetState(void)
{
    /* 新一轮运行前同时清空公开遥测量和内部滤波状态。 */
    g_lineLostMs = 0;
    g_lineBlackCount = 0U;
    g_lineBadMaskCount = 0U;
    g_lineZeroMaskCount = 0U;
    g_lineCornerMaskStableCount = 0U;
    g_lineErrorFiltered = 0.0f;
    g_lineLastCtrlError = 0.0f;
    s_lastValidError = 0;
    s_hasValidError = 0U;
    s_badMaskCount = 0U;
    s_zeroMaskCount = 0U;
    s_cornerMaskStableCount = 0U;
}

void App_Line_Update(void)
{
    uint8_t raw[GRAYSCALE_CHANNELS];
    /* 传感器权重以中心为 0，正值表示黑线偏右。 */
    static const int16_t weight[GRAYSCALE_CHANNELS] = {-350, -250, -150, -50, 50, 150, 250, 350};
    int32_t sum = 0;
    int16_t count = 0;
    uint8_t mask = 0;
    uint8_t i;
    uint8_t blackLevel;
    uint8_t reverseOrder;
    uint8_t cornerBlackCountTh;
    uint8_t continuous;

    blackLevel = (g_lineBlackLevelF <= 0.5f) ? 0U : 1U;
    reverseOrder = (g_lineReverseOrderF <= 0.5f) ? 0U : 1U;
    cornerBlackCountTh = g_eCarParam.corner_black_count_th;
    if (cornerBlackCountTh == 0U || cornerBlackCountTh > GRAYSCALE_CHANNELS)
    {
        cornerBlackCountTh = 5U;
    }

    Grayscale_ReadAll(raw);

    /* rawMask 保留物理通道顺序，方便板级测试确认接线。 */
    g_lineRawMask = 0;
    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        if (raw[i] == blackLevel) g_lineRawMask |= (uint8_t)(1U << i);
    }

    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        uint8_t physicalIndex;
        uint8_t isBlack;

        /* lineMask 是转向控制使用的逻辑顺序，可按配置反转。 */
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
    continuous = App_Line_IsContinuousMask(mask, (uint8_t)count);

    if (count == 0)
    {
        /* 没有检测到黑线：标记丢线，但保留最近一次有效误差。 */
        g_lineValid = 0U;
        App_Line_HoldLastError();
        if (s_zeroMaskCount < 255U) s_zeroMaskCount++;
        if (g_lineLostMs <= (uint16_t)(60000U - ECAR_CONTROL_PERIOD_MS))
        {
            g_lineLostMs = (uint16_t)(g_lineLostMs + ECAR_CONTROL_PERIOD_MS);
        }
        s_cornerMaskStableCount = 0U;
    }
    else if ((uint8_t)count >= cornerBlackCountTh)
    {
        /* 大面积黑色区域视为角点标记，不作为普通可循迹黑线。 */
        g_lineValid = 0U;
        App_Line_HoldLastError();
        if (s_cornerMaskStableCount < 255U) s_cornerMaskStableCount++;
        s_zeroMaskCount = 0U;
    }
    else if (((uint8_t)count < cornerBlackCountTh) && continuous)
    {
        float rawError;
        float alpha;

        /* 普通黑线：1 到 cornerBlackCountTh-1 路连续黑线视为有效循迹线。 */
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
        g_lineLostMs = 0;
        s_zeroMaskCount = 0U;
        s_cornerMaskStableCount = 0U;
        s_badMaskCount = (uint8_t)(s_badMaskCount / 2U);
    }
    else
    {
        /* 非连续或过宽 mask 判为异常，避免车辆跟随噪声转向。 */
        g_lineValid = 0U;
        App_Line_HoldLastError();
        if (s_badMaskCount < 255U) s_badMaskCount++;
        s_zeroMaskCount = 0U;
        s_cornerMaskStableCount = 0U;
    }

    g_lineBadMaskCount = s_badMaskCount;
    g_lineZeroMaskCount = s_zeroMaskCount;
    g_lineCornerMaskStableCount = s_cornerMaskStableCount;
}

float App_Line_CalcTurnCmd(void)
{
    float error;
    float dError;
    float turn;

    error = (float)g_lineError;
    dError = error - g_lineLastCtrlError;
    g_lineLastCtrlError = error;

    /* PD 转向辅助函数，供未直接使用完整 E 题状态机的模块调用。 */
    turn = (-g_lineTurnSign) * (error * g_lineKp + dError * g_lineKd);

    return App_Line_LimitFloat(turn, -g_lineTurnLimit, g_lineTurnLimit);
}
