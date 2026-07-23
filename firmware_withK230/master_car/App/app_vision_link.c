#include "app_vision_link.h"
#include "Serial.h"
#include "Timer.h"
#include <limits.h>
#include <stdint.h>

typedef enum
{
    VISION_RX_WAIT_AA = 0,
    VISION_RX_WAIT_55,
    VISION_RX_COLLECT
} VisionRxState_t;

static VisionRxState_t s_rxState = VISION_RX_WAIT_AA;
static uint8_t         s_rxBuf[VISION_BINARY_FRAME_SIZE];
static uint8_t         s_rxIdx = 0U;

static VisionTrackFrame_t s_latest;
static uint8_t             s_hasNewFrame = 0U;

static uint32_t s_validFrameCount       = 0U;
static uint32_t s_crcErrorCount         = 0U;
static uint32_t s_headerSyncLossCount   = 0U;
static uint32_t s_versionErrorCount     = 0U;
static uint32_t s_messageTypeErrorCount = 0U;
static uint32_t s_duplicateFrameCount   = 0U;

static uint16_t s_lastProcessedSeq = 0xFFFFU;
static uint8_t  s_lastSeqValid     = 0U;

#define VISION_RESET_FLUSH_MAX_BYTES 128U

static uint16_t Vision_ReadLe16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t Vision_ReadS16Le(const uint8_t *p)
{
    return (int16_t)Vision_ReadLe16(p);
}

static uint32_t Vision_ReadLe32(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static uint16_t Vision_Crc16CcittFalse(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    for (i = 0U; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8;

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static void Vision_ParseAndUpdate(const uint8_t *frame)
{
    uint16_t crcRx;
    uint16_t crcCalc;
    uint16_t sequence;
    uint8_t  isNewSeq;
    int16_t  lat;
    int16_t  head;
    uint16_t rw;
    VisionTrackFrame_t tmp;

    crcRx   = Vision_ReadLe16(&frame[22]);
    crcCalc = Vision_Crc16CcittFalse(frame, 22U);

    if (crcRx != crcCalc)
    {
        s_crcErrorCount++;
        return;
    }

    if (frame[2] != VISION_BINARY_VERSION)
    {
        s_versionErrorCount++;
        return;
    }

    if (frame[3] != VISION_BINARY_MESSAGE_ROAD)
    {
        s_messageTypeErrorCount++;
        return;
    }

    sequence = Vision_ReadLe16(&frame[4]);
    lat      = Vision_ReadS16Le(&frame[12]);
    head     = Vision_ReadS16Le(&frame[14]);
    rw       = Vision_ReadLe16(&frame[16]);

    if (frame[20] > 100U) return;

    tmp.k230TimestampMs     = Vision_ReadLe32(&frame[6]);
    tmp.mode                = frame[10];
    tmp.statusFlags         = frame[11];
    tmp.lateralErrorDeciMm  = lat;
    tmp.headingErrorCentiDeg = head;
    tmp.roadWidthDeciMm     = rw;
    tmp.junctionStage       = frame[18];
    tmp.junctionDistanceLevel = frame[19];
    tmp.confidence          = frame[20];
    tmp.anomalyFlags        = frame[21];

    tmp.transportValid = 1U;
    tmp.visionValid = ((tmp.statusFlags & 0x01U) != 0U) &&
                      (lat != INT16_MIN) &&
                      (head != INT16_MIN) &&
                      (rw != 0xFFFFU);

    isNewSeq = s_lastSeqValid
        ? ((sequence != s_lastProcessedSeq) ? 1U : 0U)
        : 1U;

    if (!isNewSeq)
    {
        s_duplicateFrameCount++;
        return;
    }

    tmp.receiveTimeMs = Timer_GetMillis();
    tmp.sequence      = sequence;

    s_latest           = tmp;
    s_hasNewFrame      = 1U;
    s_validFrameCount++;
    s_lastProcessedSeq = sequence;
    s_lastSeqValid     = 1U;
}

void App_VisionLink_Init(void)
{
    s_rxState = VISION_RX_WAIT_AA;
    s_rxIdx   = 0U;

    s_latest.transportValid = 0U;
    s_latest.visionValid    = 0U;
    s_latest.sequence       = 0U;
    s_hasNewFrame           = 0U;

    s_validFrameCount       = 0U;
    s_crcErrorCount         = 0U;
    s_headerSyncLossCount   = 0U;
    s_versionErrorCount     = 0U;
    s_messageTypeErrorCount = 0U;
    s_duplicateFrameCount   = 0U;

    s_lastProcessedSeq = 0xFFFFU;
    s_lastSeqValid     = 0U;
}

void App_VisionLink_Task10ms(void)
{
    uint8_t byte;

    s_hasNewFrame = 0U;

    while (Serial_ReadByte(&byte))
    {
        switch (s_rxState)
        {
        case VISION_RX_WAIT_AA:
            if (byte == VISION_BINARY_HEADER_0)
            {
                s_rxBuf[0] = byte;
                s_rxState  = VISION_RX_WAIT_55;
            }
            else
            {
                s_headerSyncLossCount++;
            }
            break;

        case VISION_RX_WAIT_55:
            if (byte == VISION_BINARY_HEADER_1)
            {
                s_rxBuf[1] = byte;
                s_rxIdx    = 2U;
                s_rxState  = VISION_RX_COLLECT;
            }
            else if (byte == VISION_BINARY_HEADER_0)
            {
                s_rxBuf[0] = byte;
            }
            else
            {
                s_rxState = VISION_RX_WAIT_AA;
                s_headerSyncLossCount++;
            }
            break;

        case VISION_RX_COLLECT:
            if (s_rxIdx < VISION_BINARY_FRAME_SIZE)
            {
                s_rxBuf[s_rxIdx++] = byte;

                if (s_rxIdx >= VISION_BINARY_FRAME_SIZE)
                {
                    Vision_ParseAndUpdate(s_rxBuf);
                    s_rxState = VISION_RX_WAIT_AA;
                    s_rxIdx   = 0U;
                }
            }
            else
            {
                s_rxState = VISION_RX_WAIT_AA;
                s_rxIdx   = 0U;
            }
            break;

        default:
            s_rxState = VISION_RX_WAIT_AA;
            s_rxIdx   = 0U;
            break;
        }
    }
}

void App_VisionLink_Reset(void)
{
    uint8_t  byte;
    uint16_t flushed = 0U;

    s_rxState = VISION_RX_WAIT_AA;
    s_rxIdx   = 0U;

    while ((flushed < VISION_RESET_FLUSH_MAX_BYTES) &&
           Serial_ReadByte(&byte))
    {
        flushed++;
    }

    s_rxState = VISION_RX_WAIT_AA;
    s_rxIdx   = 0U;

    s_latest.transportValid = 0U;
    s_latest.visionValid    = 0U;
    s_latest.sequence       = 0U;
    s_hasNewFrame           = 0U;

    s_lastProcessedSeq = 0xFFFFU;
    s_lastSeqValid     = 0U;
}

uint8_t App_VisionLink_GetLatest(VisionTrackFrame_t *frame)
{
    if (!frame) return 0U;
    *frame = s_latest;
    return s_latest.transportValid;
}

uint8_t App_VisionLink_HasNewFrame(void)
{
    return s_hasNewFrame;
}

uint32_t App_VisionLink_GetFrameAgeMs(void)
{
    uint32_t now;
    if (!s_latest.transportValid) return 0xFFFFFFFFUL;
    now = Timer_GetMillis();
    if (s_latest.receiveTimeMs > now) return 0U;
    return now - s_latest.receiveTimeMs;
}

uint32_t App_VisionLink_GetValidFrameCount(void)
{
    return s_validFrameCount;
}

uint32_t App_VisionLink_GetCrcErrorCount(void)
{
    return s_crcErrorCount;
}

uint32_t App_VisionLink_GetHeaderSyncLossCount(void)
{
    return s_headerSyncLossCount;
}

uint32_t App_VisionLink_GetVersionErrorCount(void)
{
    return s_versionErrorCount;
}

uint32_t App_VisionLink_GetMessageTypeErrorCount(void)
{
    return s_messageTypeErrorCount;
}

uint32_t App_VisionLink_GetDuplicateFrameCount(void)
{
    return s_duplicateFrameCount;
}

uint32_t App_VisionLink_GetRxOverflowCount(void)
{
    return Serial_GetRxOverflowCount();
}

void App_VisionLink_SendTrackMode(void)
{
}

void App_VisionLink_SendIdleMode(void)
{
}
