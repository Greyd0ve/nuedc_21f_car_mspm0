#include "app_vision_link.h"
#include "Serial.h"
#include <stdint.h>

static char            s_rxBuf[VISION_RX_FRAME_SIZE];
static uint8_t         s_rxIdx = 0U;
static uint8_t         s_inFrame = 0U;

static VisionTrackFrame_t s_latest;
static uint8_t             s_hasNewFrame = 0U;

static uint32_t s_validFrameCount  = 0U;
static uint32_t s_parseErrorCount  = 0U;
static uint32_t s_systemTimeMs     = 0U;

static uint16_t s_lastProcessedSeq = 0xFFFFU;
static uint8_t  s_lastSeqValid     = 0U;

static int32_t  ParseInt32(const char *p, const char **endp)
{
    int32_t val = 0;
    uint8_t neg = 0U;

    if (*p == '-')
    {
        neg = 1U;
        p++;
    }
    while (*p >= '0' && *p <= '9')
    {
        val = val * 10 + (int32_t)(*p - '0');
        p++;
    }
    if (endp) *endp = p;
    return neg ? -val : val;
}

static const char *SkipField(const char *p)
{
    while (*p >= '0' && *p <= '9') p++;
    if (*p == '-') p++;
    while (*p >= '0' && *p <= '9') p++;
    return p;
}

static uint8_t ParseFields(const char *buf, VisionTrackFrame_t *f)
{
    const char *p = buf;
    uint32_t fldCount = 0U;

    if (p[0] != '[') return 0U;
    p++;

    if (!(p[0] == 't' && p[1] == 'r' && p[2] == 'k' && p[3] == '1')) return 0U;
    p += 4;
    if (*p != ',') return 0U;
    p++;
    fldCount = 1U;

    {
        int32_t v = ParseInt32(p, &p);
        if (v < 0 || v > 65535) return 0U;
        f->seq = (uint16_t)v;
    }
    if (*p == ',') p++; else return 0U;

    {
        int32_t v = ParseInt32(p, &p);
        if (v < -1000 || v > 1000) return 0U;
        f->lateralErrorQ1000 = (int16_t)v;
    }
    if (*p == ',') p++; else return 0U;

    {
        int32_t v = ParseInt32(p, &p);
        if (v < -1800 || v > 1800) return 0U;
        f->headingErrorDeciDeg = (int16_t)v;
    }
    if (*p == ',') p++; else return 0U;

    {
        int32_t v = ParseInt32(p, &p);
        if (v < 0 || v > 5000) return 0U;
        f->laneWidthQ1000 = (uint16_t)v;
    }
    if (*p == ',') p++; else return 0U;

    {
        int32_t v = ParseInt32(p, &p);
        if (v < 0 || v > 255) return 0U;
        f->flags = (uint8_t)v;
    }
    if (*p == ',') p++; else return 0U;

    {
        int32_t v = ParseInt32(p, &p);
        if (v < 0 || v > 65535) return 0U;
        f->junctionDistanceMm = (uint16_t)v;
    }
    if (*p == ',') p++; else return 0U;

    {
        int32_t v = ParseInt32(p, &p);
        if (v < 0 || v > 100) return 0U;
        f->confidence = (uint8_t)v;
    }

    if (*p != ']') return 0U;

    f->frameValid = 1U;
    return 1U;
}

void App_VisionLink_Init(void)
{
    s_rxIdx   = 0U;
    s_inFrame = 0U;

    s_latest.frameValid = 0U;
    s_latest.seq        = 0U;
    s_hasNewFrame       = 0U;

    s_validFrameCount   = 0U;
    s_parseErrorCount   = 0U;
    s_systemTimeMs      = 0U;
    s_lastProcessedSeq  = 0xFFFFU;
    s_lastSeqValid      = 0U;
}

void App_VisionLink_Task10ms(void)
{
    uint8_t byte;

    s_hasNewFrame = 0U;
    s_systemTimeMs += 10U;

    while (Serial_ReadByte(&byte))
    {
        if (byte == '\r' || byte == '\n')
        {
            continue;
        }

        if (byte == '[')
        {
            s_rxIdx   = 0U;
            s_inFrame = 1U;
            continue;
        }

        if (!s_inFrame) continue;

        if (byte == ']')
        {
            VisionTrackFrame_t tmp;
            if (s_rxIdx < VISION_RX_FRAME_SIZE)
            {
                s_rxBuf[s_rxIdx] = '\0';
            }
            s_inFrame = 0U;
            s_rxIdx   = 0U;

            tmp.frameValid = 0U;
            if (ParseFields(s_rxBuf, &tmp))
            {
                uint8_t isNewSeq;
                tmp.receiveTimeMs = s_systemTimeMs;
                tmp.frameValid    = 1U;

                s_latest = tmp;

                isNewSeq = s_lastSeqValid
                    ? ((tmp.seq != s_lastProcessedSeq) ? 1U : 0U)
                    : 1U;
                s_lastProcessedSeq = tmp.seq;
                s_lastSeqValid     = 1U;

                s_hasNewFrame = isNewSeq ? 1U : 0U;
                if (isNewSeq) s_validFrameCount++;
            }
            else
            {
                s_parseErrorCount++;
            }
            continue;
        }

        if (s_rxIdx < (VISION_RX_FRAME_SIZE - 1U))
        {
            s_rxBuf[s_rxIdx++] = (char)byte;
        }
        else
        {
            s_inFrame = 0U;
            s_rxIdx   = 0U;
            s_parseErrorCount++;
        }
    }
}

void App_VisionLink_Reset(void)
{
    s_latest.frameValid = 0U;
    s_latest.seq        = 0U;
    s_hasNewFrame       = 0U;
    s_lastProcessedSeq  = 0xFFFFU;
    s_lastSeqValid      = 0U;
}

uint8_t App_VisionLink_GetLatest(VisionTrackFrame_t *frame)
{
    if (!frame) return 0U;
    *frame = s_latest;
    return s_latest.frameValid;
}

uint8_t App_VisionLink_HasNewFrame(void)
{
    return s_hasNewFrame;
}

uint8_t App_VisionLink_IsFresh(uint32_t maxAgeMs)
{
    if (!s_latest.frameValid) return 0U;
    return (App_VisionLink_GetFrameAgeMs() <= maxAgeMs) ? 1U : 0U;
}

uint32_t App_VisionLink_GetFrameAgeMs(void)
{
    if (!s_latest.frameValid) return 0xFFFFFFFFUL;
    if (s_latest.receiveTimeMs > s_systemTimeMs) return 0U;
    return s_systemTimeMs - s_latest.receiveTimeMs;
}

uint32_t App_VisionLink_GetValidFrameCount(void)
{
    return s_validFrameCount;
}

uint32_t App_VisionLink_GetParseErrorCount(void)
{
    return s_parseErrorCount;
}

void App_VisionLink_SendTrackMode(void)
{
    Serial_SendString("[mode,track]\r\n");
}

void App_VisionLink_SendIdleMode(void)
{
    Serial_SendString("[mode,idle]\r\n");
}
