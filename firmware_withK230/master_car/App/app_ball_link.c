#include "app_ball_link.h"
#include "Serial.h"
#include "Timer.h"
#include <stdint.h>

typedef enum
{
    BALL_RX_WAIT_AA = 0,
    BALL_RX_WAIT_55,
    BALL_RX_COLLECT
} BallRxState_t;

static BallRxState_t s_rxState = BALL_RX_WAIT_AA;
static uint8_t s_rxBuf[BALL_LINK_FRAME_SIZE];
static uint8_t s_rxIndex = 0U;
static BallLinkFrame_t s_latest;
static uint8_t s_hasNewFrame = 0U;
static uint32_t s_validFrameCount = 0U;
static uint32_t s_crcErrorCount = 0U;

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

static void BallLink_ParseFrame(const uint8_t *frame)
{
    uint16_t crcRx;
    uint16_t crcCalc;
    BallLinkFrame_t next;

    crcRx = BallLink_ReadLe16(&frame[12]);
    crcCalc = BallLink_Crc16CcittFalse(frame, 12U);
    if (crcRx != crcCalc)
    {
        s_crcErrorCount++;
        return;
    }

    if (frame[2] != BALL_LINK_VERSION || frame[3] != BALL_LINK_MESSAGE_BALL)
    {
        return;
    }

    next.sequence = BallLink_ReadLe16(&frame[4]);
    next.flags = frame[6];
    next.confidence = frame[7];
    next.ballXPixel = BallLink_ReadLe16(&frame[8]);
    next.positionCentiCm = BallLink_ReadLe16(&frame[10]);
    next.receiveTimeMs = Timer_GetMillis();
    next.transportValid = 1U;
    next.ballValid = ((next.flags & BALL_LINK_FLAG_BALL_VALID) != 0U) ? 1U : 0U;
    next.pipeValid = ((next.flags & BALL_LINK_FLAG_PIPE_VALID) != 0U) ? 1U : 0U;

    s_latest = next;
    s_hasNewFrame = 1U;
    s_validFrameCount++;
}

void App_BallLink_Init(void)
{
    App_BallLink_Reset();
    s_validFrameCount = 0U;
    s_crcErrorCount = 0U;
}

void App_BallLink_Reset(void)
{
    s_rxState = BALL_RX_WAIT_AA;
    s_rxIndex = 0U;
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
        switch (s_rxState)
        {
        case BALL_RX_WAIT_AA:
            if (byte == BALL_LINK_HEADER_0)
            {
                s_rxBuf[0] = byte;
                s_rxState = BALL_RX_WAIT_55;
            }
            break;

        case BALL_RX_WAIT_55:
            if (byte == BALL_LINK_HEADER_1)
            {
                s_rxBuf[1] = byte;
                s_rxIndex = 2U;
                s_rxState = BALL_RX_COLLECT;
            }
            else if (byte == BALL_LINK_HEADER_0)
            {
                s_rxBuf[0] = byte;
            }
            else
            {
                s_rxState = BALL_RX_WAIT_AA;
            }
            break;

        case BALL_RX_COLLECT:
            s_rxBuf[s_rxIndex++] = byte;
            if (s_rxIndex >= BALL_LINK_FRAME_SIZE)
            {
                BallLink_ParseFrame(s_rxBuf);
                s_rxIndex = 0U;
                s_rxState = BALL_RX_WAIT_AA;
            }
            break;

        default:
            s_rxIndex = 0U;
            s_rxState = BALL_RX_WAIT_AA;
            break;
        }
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

uint32_t App_BallLink_GetRxOverflowCount(void)
{
    return Serial_GetRxOverflowCount();
}
