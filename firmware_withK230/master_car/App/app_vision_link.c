#include "app_vision_link.h"
#include "Serial.h"
#include "Timer.h"
#include <stdint.h>

static char    s_rxBuf[VISION_RX_FRAME_SIZE];
static uint8_t s_rxIdx   = 0U;
static uint8_t s_inFrame = 0U;

static VisionTrackFrame_t s_latest;
static uint8_t             s_hasNewFrame = 0U;

static uint32_t s_validFrameCount    = 0U;
static uint32_t s_parseErrorCount    = 0U;
static uint32_t s_duplicateFrameCount = 0U;
static uint32_t s_unknownFrameCount   = 0U;

static uint16_t s_lastProcessedSeq = 0xFFFFU;
static uint8_t  s_lastSeqValid     = 0U;

static int32_t ParseInt32(const char *p, const char **endp, uint8_t *ok)
{
    int32_t  val = 0;
    uint8_t  neg = 0U;
    uint8_t  hasDigit = 0U;

    *ok = 0U;

    if (*p == '-')
    {
        neg = 1U;
        p++;
    }

    if (*p < '0' || *p > '9')
    {
        return 0;
    }

    while (*p >= '0' && *p <= '9')
    {
        int32_t digit = (int32_t)(*p - '0');

        if (neg)
        {
            if (val < (INT32_MIN + digit) / 10)
            {
                return 0;
            }
            val = val * 10 - digit;
        }
        else
        {
            if (val > (INT32_MAX - digit) / 10)
            {
                return 0;
            }
            val = val * 10 + digit;
        }

        hasDigit = 1U;
        p++;
    }

    if (!hasDigit) return 0;
    if (endp) *endp = p;
    *ok = 1U;
    return val;
}

static uint8_t IsTrk1Frame(const char *buf)
{
    return (buf[0] == '[' &&
            buf[1] == 't' && buf[2] == 'r' && buf[3] == 'k' && buf[4] == '1' &&
            buf[5] == ',') ? 1U : 0U;
}

static uint8_t IsReadyFrame(const char *buf)
{
    return (buf[0] == '[' &&
            buf[1] == 'r' && buf[2] == 'e' && buf[3] == 'a' && buf[4] == 'd' &&
            buf[5] == 'y' && buf[6] == ',') ? 1U : 0U;
}

static uint8_t ParseFields(const char *buf, VisionTrackFrame_t *f)
{
    const char *p = buf;
    int32_t v;
    uint8_t ok;

    if (p[0] != '[') return 0U;
    p++;

    if (!(p[0] == 't' && p[1] == 'r' && p[2] == 'k' && p[3] == '1')) return 0U;
    p += 4;
    if (*p != ',') return 0U;
    p++;

    v = ParseInt32(p, &p, &ok);
    if (!ok || v < 0 || v > 65535) return 0U;
    f->seq = (uint16_t)v;
    if (*p != ',') return 0U;
    p++;

    v = ParseInt32(p, &p, &ok);
    if (!ok || v < -1000 || v > 1000) return 0U;
    f->lateralErrorQ1000 = (int16_t)v;
    if (*p != ',') return 0U;
    p++;

    v = ParseInt32(p, &p, &ok);
    if (!ok || v < -1800 || v > 1800) return 0U;
    f->headingErrorDeciDeg = (int16_t)v;
    if (*p != ',') return 0U;
    p++;

    v = ParseInt32(p, &p, &ok);
    if (!ok || v < 0 || v > 5000) return 0U;
    f->laneWidthQ1000 = (uint16_t)v;
    if (*p != ',') return 0U;
    p++;

    v = ParseInt32(p, &p, &ok);
    if (!ok || v < 0 || v > 255) return 0U;
    f->flags = (uint8_t)v;
    if (*p != ',') return 0U;
    p++;

    v = ParseInt32(p, &p, &ok);
    if (!ok || v < 0 || v > 65535) return 0U;
    f->junctionDistanceMm = (uint16_t)v;
    if (*p != ',') return 0U;
    p++;

    v = ParseInt32(p, &p, &ok);
    if (!ok || v < 0 || v > 100) return 0U;
    f->confidence = (uint8_t)v;

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

    s_validFrameCount    = 0U;
    s_parseErrorCount    = 0U;
    s_duplicateFrameCount = 0U;
    s_unknownFrameCount   = 0U;

    s_lastProcessedSeq = 0xFFFFU;
    s_lastSeqValid     = 0U;
}

void App_VisionLink_Task10ms(void)
{
    uint8_t byte;

    s_hasNewFrame = 0U;

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
            continue;
        }

        if (!s_inFrame) continue;

        if (byte == ']')
        {
            VisionTrackFrame_t tmp;

            if (s_rxIdx >= (VISION_RX_FRAME_SIZE - 1U))
            {
                s_inFrame = 0U;
                s_rxIdx   = 0U;
                s_rxBuf[0] = '\0';
                s_parseErrorCount++;
                continue;
            }

            s_rxBuf[s_rxIdx++] = (char)byte;
            s_rxBuf[s_rxIdx]   = '\0';

            s_inFrame = 0U;
            s_rxIdx   = 0U;

            tmp.frameValid = 0U;

            if (IsTrk1Frame(s_rxBuf))
            {
                if (ParseFields(s_rxBuf, &tmp))
                {
                    uint8_t isNewSeq;

                    isNewSeq = s_lastSeqValid
                        ? ((tmp.seq != s_lastProcessedSeq) ? 1U : 0U)
                        : 1U;

                    if (isNewSeq)
                    {
                        tmp.receiveTimeMs = Timer_GetMillis();
                        tmp.frameValid    = 1U;

                        s_latest            = tmp;
                        s_hasNewFrame       = 1U;
                        s_validFrameCount++;
                        s_lastProcessedSeq  = tmp.seq;
                        s_lastSeqValid      = 1U;
                    }
                    else
                    {
                        s_duplicateFrameCount++;
                    }
                }
                else
                {
                    s_parseErrorCount++;
                }
            }
            else if (IsReadyFrame(s_rxBuf))
            {
                s_unknownFrameCount++;
            }
            else
            {
                s_unknownFrameCount++;
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

#define VISION_RESET_FLUSH_MAX_BYTES 128U

void App_VisionLink_Reset(void)
{
    uint8_t  byte;
    uint16_t flushed = 0U;

    s_rxIdx   = 0U;
    s_inFrame = 0U;
    s_rxBuf[0] = '\0';

    while ((flushed < VISION_RESET_FLUSH_MAX_BYTES) &&
           Serial_ReadByte(&byte))
    {
        flushed++;
    }

    s_rxIdx   = 0U;
    s_inFrame = 0U;
    s_rxBuf[0] = '\0';

    s_latest.frameValid = 0U;
    s_latest.seq        = 0U;
    s_hasNewFrame       = 0U;

    s_lastProcessedSeq = 0xFFFFU;
    s_lastSeqValid     = 0U;
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
    uint32_t now;
    if (!s_latest.frameValid) return 0xFFFFFFFFUL;
    now = Timer_GetMillis();
    if (s_latest.receiveTimeMs > now) return 0U;
    return now - s_latest.receiveTimeMs;
}

uint32_t App_VisionLink_GetValidFrameCount(void)
{
    return s_validFrameCount;
}

uint32_t App_VisionLink_GetParseErrorCount(void)
{
    return s_parseErrorCount;
}

uint32_t App_VisionLink_GetDuplicateFrameCount(void)
{
    return s_duplicateFrameCount;
}

uint32_t App_VisionLink_GetUnknownFrameCount(void)
{
    return s_unknownFrameCount;
}

uint32_t App_VisionLink_GetRxOverflowCount(void)
{
    return Serial_GetRxOverflowCount();
}

void App_VisionLink_SendTrackMode(void)
{
    Serial_SendString("[mode,track]\r\n");
}

void App_VisionLink_SendIdleMode(void)
{
    Serial_SendString("[mode,idle]\r\n");
}
