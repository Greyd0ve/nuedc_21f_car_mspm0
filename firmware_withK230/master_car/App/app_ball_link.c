#include "app_ball_link.h"
#include "Serial.h"
#include "Timer.h"
#include <stdint.h>

/*
 * A rolling 14-byte window is intentionally used instead of a fixed COLLECT
 * state.  After any byte loss, the next AA 55 at an arbitrary byte offset is
 * found as soon as the following complete packet has arrived.
 */
static uint8_t s_rxWindow[BALL_LINK_FRAME_SIZE];
static uint8_t s_rxCount = 0U;
static BallLinkFrame_t s_latest;
static uint8_t s_hasNewFrame = 0U;
static uint32_t s_validFrameCount = 0U;
static uint32_t s_crcErrorCount = 0U;
static uint32_t s_formatErrorCount = 0U;
static uint32_t s_parserByteCount = 0U;
static uint32_t s_headerSyncLossCount = 0U;

static uint16_t BallLink_ReadLe16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint16_t BallLink_Crc16CcittFalse(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    for (index = 0U; index < length; index++)
    {
        crc ^= (uint16_t)data[index] << 8U;
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }
    return crc;
}

static uint8_t BallLink_ParseWindow(const uint8_t *frame)
{
    uint16_t crcRx;
    uint16_t crcCalc;
    BallLinkFrame_t next;

    if (frame[0] != BALL_LINK_HEADER_0 || frame[1] != BALL_LINK_HEADER_1 ||
        frame[2] != BALL_LINK_VERSION || frame[3] != BALL_LINK_MESSAGE_BALL)
    {
        s_headerSyncLossCount++;
        return 0U;
    }

    crcRx = BallLink_ReadLe16(&frame[12]);
    crcCalc = BallLink_Crc16CcittFalse(frame, 12U);
    if (crcRx != crcCalc)
    {
        s_crcErrorCount++;
        return 0U;
    }

    next.sequence = BallLink_ReadLe16(&frame[4]);
    next.flags = frame[6];
    next.confidence = frame[7];
    next.ballXPixel = BallLink_ReadLe16(&frame[8]);
    next.positionCentiCm = BallLink_ReadLe16(&frame[10]);
    next.ballValid = ((next.flags & BALL_LINK_FLAG_BALL_VALID) != 0U) ? 1U : 0U;
    next.pipeValid = ((next.flags & BALL_LINK_FLAG_PIPE_VALID) != 0U) ? 1U : 0U;

    if (next.confidence > 100U ||
        (next.ballValid != 0U &&
         (next.positionCentiCm > BALL_LINK_MAX_POSITION_CENTICM ||
          next.ballXPixel > BALL_LINK_MAX_X_PIXEL)))
    {
        s_formatErrorCount++;
        return 0U;
    }

    next.receiveTimeMs = Timer_GetMillis();
    next.transportValid = 1U;
    s_latest = next;
    s_hasNewFrame = 1U;
    s_validFrameCount++;
    return 1U;
}

static void BallLink_PushByte(uint8_t byte)
{
    uint8_t index;

    if (s_rxCount < BALL_LINK_FRAME_SIZE)
    {
        s_rxWindow[s_rxCount++] = byte;
    }
    else
    {
        for (index = 0U; index < (BALL_LINK_FRAME_SIZE - 1U); index++)
        {
            s_rxWindow[index] = s_rxWindow[index + 1U];
        }
        s_rxWindow[BALL_LINK_FRAME_SIZE - 1U] = byte;
    }

    if (s_rxCount == BALL_LINK_FRAME_SIZE &&
        BallLink_ParseWindow(s_rxWindow) != 0U)
    {
        /* Normal packets are exactly adjacent, so begin collecting the next. */
        s_rxCount = 0U;
    }
}

void App_BallLink_Init(void)
{
    App_BallLink_Reset();
    App_BallLink_ResetDiagnostics();
}

void App_BallLink_ResetDiagnostics(void)
{
    s_validFrameCount = 0U;
    s_crcErrorCount = 0U;
    s_formatErrorCount = 0U;
    s_parserByteCount = 0U;
    s_headerSyncLossCount = 0U;
    Serial_ResetRxDiagnostics();
}

void App_BallLink_Reset(void)
{
    s_rxCount = 0U;
    s_hasNewFrame = 0U;
    s_latest.sequence = 0U;
    s_latest.flags = 0U;
    s_latest.confidence = 0U;
    s_latest.ballXPixel = 0xFFFFU;
    s_latest.positionCentiCm = 0xFFFFU;
    s_latest.receiveTimeMs = 0U;
    s_latest.transportValid = 0U;
    s_latest.ballValid = 0U;
    s_latest.pipeValid = 0U;
}

void App_BallLink_Task10ms(void)
{
    uint8_t byte;

    s_hasNewFrame = 0U;
    while (Serial_ReadByte(&byte) != 0U)
    {
        s_parserByteCount++;
        BallLink_PushByte(byte);
    }
}

uint8_t App_BallLink_GetLatest(BallLinkFrame_t *frame)
{
    if (frame == 0 || s_latest.transportValid == 0U)
    {
        return 0U;
    }
    *frame = s_latest;
    return 1U;
}

uint8_t App_BallLink_HasNewFrame(void)
{
    return s_hasNewFrame;
}

uint32_t App_BallLink_GetFrameAgeMs(uint32_t nowMs)
{
    if (s_latest.transportValid == 0U)
    {
        return 0xFFFFFFFFUL;
    }
    return nowMs - s_latest.receiveTimeMs;
}

uint32_t App_BallLink_GetValidFrameCount(void)
{
    return s_validFrameCount;
}

uint32_t App_BallLink_GetCrcErrorCount(void)
{
    return s_crcErrorCount;
}

uint32_t App_BallLink_GetFormatErrorCount(void)
{
    return s_formatErrorCount;
}

uint32_t App_BallLink_GetRxOverflowCount(void)
{
    return Serial_GetRxOverflowCount();
}

uint32_t App_BallLink_GetParserByteCount(void)
{
    return s_parserByteCount;
}

uint32_t App_BallLink_GetHeaderSyncLossCount(void)
{
    return s_headerSyncLossCount;
}
