#include "app_vision_track.h"
#include "app_vision_link.h"
#include "app_config.h"
#include "app_car_state.h"
#include "app_control.h"
#if VISION_TRACK_DEBUG_ENABLE
#include "DebugSerial.h"
#include "Timer.h"
#endif
#include <stdint.h>

/*
 * Non-drivable reason bit masks (matches CSV field "ir").
 */
#define VISION_DEBUG_INVALID_NO_TRANSPORT   0x01U
#define VISION_DEBUG_INVALID_VISION         0x02U
#define VISION_DEBUG_INVALID_MODE           0x04U
#define VISION_DEBUG_INVALID_STALE          0x08U
#define VISION_DEBUG_INVALID_STOPPED        0x10U

static VisionTrackState_t s_state = VISION_TRACK_STOPPED;
static VisionTrackFrame_t s_curFrame;

#if VISION_TRACK_DEBUG_ENABLE
#define VISION_DEBUG_LINE_SIZE    256U
static char    s_debugLine[VISION_DEBUG_LINE_SIZE];
static uint16_t s_debugLineLen = 0U;

static float  s_dbgRawSteer;
static float  s_dbgFinalSteer;
static float  s_dbgForward;
static float  s_dbgLeftTarget;
static float  s_dbgRightTarget;
static uint8_t  s_dbgInvalidReason;
static uint8_t  s_dbgDrivable;
static uint32_t s_dbgFrameAgeMs;
#endif

static float clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int32_t ToCenti(float value)
{
    if (value >= 0.0f)
        return (int32_t)(value * 100.0f + 0.5f);
    return (int32_t)(value * 100.0f - 0.5f);
}

static uint8_t GetInvalidReason(const VisionTrackFrame_t *f,
                                uint32_t frameAgeMs)
{
    uint8_t reason = 0U;

    if (!f->transportValid)  reason |= VISION_DEBUG_INVALID_NO_TRANSPORT;
    if (!f->visionValid)     reason |= VISION_DEBUG_INVALID_VISION;
    if (f->mode != VISION_MODE_TRACK) reason |= VISION_DEBUG_INVALID_MODE;
    if (frameAgeMs > VISION_TRACK_FRESH_LIMIT_MS) reason |= VISION_DEBUG_INVALID_STALE;
    if (s_state != VISION_TRACK_RUNNING) reason |= VISION_DEBUG_INVALID_STOPPED;

    return reason;
}

static uint8_t IsFrameDrivable(const VisionTrackFrame_t *f)
{
    if (!f->transportValid) return 0U;
    if (!f->visionValid) return 0U;
    if (f->mode != VISION_MODE_TRACK) return 0U;
    if (App_VisionLink_GetFrameAgeMs() > VISION_TRACK_FRESH_LIMIT_MS) return 0U;
    return 1U;
}

static float VisionTrack_CalcSteer(int16_t lateralErrorDeciMm)
{
    float errorMm;
    errorMm = (float)lateralErrorDeciMm *
              VISION_TRACK_ERROR_DECI_MM_TO_MM;
    return VISION_TRACK_KY_CMPS_PER_MM * errorMm;
}

#if VISION_TRACK_DEBUG_ENABLE
static void DebugAppendC(char c)
{
    if (s_debugLineLen < (VISION_DEBUG_LINE_SIZE - 1U))
        s_debugLine[s_debugLineLen++] = c;
}

static void DebugAppendS(const char *s)
{
    while (*s && (s_debugLineLen < (VISION_DEBUG_LINE_SIZE - 1U)))
        s_debugLine[s_debugLineLen++] = *s++;
}

static void DebugAppendU32(uint32_t v)
{
    char buf[10];
    uint8_t i = 0U;
    do { buf[i++] = (char)('0' + (v % 10U)); v /= 10U; } while (v > 0U);
    while (i > 0U) DebugAppendC(buf[--i]);
}

static void DebugAppendI32(int32_t v)
{
    if (v < 0) { DebugAppendC('-'); DebugAppendU32((uint32_t)(-(v + 1)) + 1U); }
    else { DebugAppendU32((uint32_t)v); }
}

static void DebugAppendHex8(uint8_t v)
{
    uint8_t hi = (v >> 4) & 0xFU;
    uint8_t lo = v & 0xFU;
    DebugAppendC((char)(hi < 10U ? '0' + hi : 'A' + hi - 10U));
    DebugAppendC((char)(lo < 10U ? '0' + lo : 'A' + lo - 10U));
}

static void DebugAppendEvent(const char *event)
{
    char line[32];
    uint8_t i = 0U;
    while (*event && i < 30U) line[i++] = *event++;
    line[i++] = '\r';
    line[i++] = '\n';
    DebugSerial_TrySendBuffer((const uint8_t *)line, (uint16_t)i);
}
#endif

void App_VisionTrack_Init(void)
{
    s_state = VISION_TRACK_STOPPED;
    s_curFrame.transportValid = 0U;
    s_curFrame.visionValid    = 0U;

    App_Control_ForcePWMZero();
}

void App_VisionTrack_Task10ms(void)
{
    float forward;
    float rawSteer;
    float steer;
    uint8_t drivable;
#if VISION_TRACK_DEBUG_ENABLE
    uint8_t invalidReason;
    uint32_t frameAgeMs;
#endif

    App_VisionLink_GetLatest(&s_curFrame);

#if VISION_TRACK_DEBUG_ENABLE
    frameAgeMs = App_VisionLink_GetFrameAgeMs();
    invalidReason = GetInvalidReason(&s_curFrame, frameAgeMs);
    drivable = (invalidReason == 0U);

    s_dbgInvalidReason = invalidReason;
    s_dbgDrivable = drivable;
    s_dbgFrameAgeMs = frameAgeMs;
#else
    drivable = IsFrameDrivable(&s_curFrame);
#endif

    if (s_state != VISION_TRACK_RUNNING)
    {
        App_Control_ForcePWMZero();
#if VISION_TRACK_DEBUG_ENABLE
        s_dbgRawSteer   = 0.0f;
        s_dbgFinalSteer = 0.0f;
        s_dbgForward    = 0.0f;
        s_dbgLeftTarget = 0.0f;
        s_dbgRightTarget = 0.0f;
#endif
        return;
    }

    if (!drivable)
    {
        App_Control_ForcePWMZero();
#if VISION_TRACK_DEBUG_ENABLE
        s_dbgRawSteer   = 0.0f;
        s_dbgFinalSteer = 0.0f;
        s_dbgForward    = 0.0f;
        s_dbgLeftTarget = 0.0f;
        s_dbgRightTarget = 0.0f;
#endif
        return;
    }

    forward = s_curFrame.degraded
        ? VISION_TRACK_DEGRADED_SPEED_CMPS
        : VISION_TRACK_FORWARD_SPEED_CMPS;

    rawSteer = VisionTrack_CalcSteer(s_curFrame.lateralErrorDeciMm);
    steer = VISION_TRACK_TURN_SIGN * rawSteer;

    steer = clamp(steer, -VISION_TRACK_TURN_LIMIT_CMPS,
                   VISION_TRACK_TURN_LIMIT_CMPS);
    steer = clamp(steer, -forward, forward);

    g_targetForwardSpeed = forward;
    g_targetTurnSpeed    = steer;
    g_carEnable = 1U;
    App_Control_ApplyMotorOutput();

#if VISION_TRACK_DEBUG_ENABLE
    s_dbgRawSteer   = rawSteer;
    s_dbgFinalSteer = steer;
    s_dbgForward    = forward;
    s_dbgLeftTarget = forward - steer;
    s_dbgRightTarget = forward + steer;
#endif
}

void App_VisionTrack_Task100ms(void)
{
#if VISION_TRACK_DEBUG_ENABLE
    s_debugLineLen = 0U;

    DebugAppendS("VDBG,t=");
    DebugAppendU32(Timer_GetMillis());
    DebugAppendS(",st=");
    DebugAppendU32((uint32_t)s_state);
    DebugAppendS(",m=");
    DebugAppendU32((uint32_t)s_curFrame.mode);
    DebugAppendS(",seq=");
    DebugAppendU32((uint32_t)s_curFrame.sequence);
    DebugAppendS(",age=");
    DebugAppendU32(s_dbgFrameAgeMs);
    DebugAppendS(",ok=");
    DebugAppendU32((uint32_t)s_dbgDrivable);
    DebugAppendS(",ir=");
    DebugAppendHex8(s_dbgInvalidReason);
    DebugAppendS(",fl=");
    DebugAppendHex8(s_curFrame.statusFlags);
    DebugAppendS(",cf=");
    DebugAppendU32((uint32_t)s_curFrame.confidence);
    DebugAppendS(",an=");
    DebugAppendHex8(s_curFrame.anomalyFlags);
    DebugAppendS(",lr=");
    DebugAppendU32((uint32_t)s_curFrame.leftBoundaryValid);
    DebugAppendS(",rr=");
    DebugAppendU32((uint32_t)s_curFrame.rightBoundaryValid);
    DebugAppendS(",ey=");
    DebugAppendI32((int32_t)s_curFrame.lateralErrorDeciMm);
    DebugAppendS(",hd=");
    DebugAppendI32((int32_t)s_curFrame.headingErrorCentiDeg);
    DebugAppendS(",rw=");
    DebugAppendU32((uint32_t)s_curFrame.roadWidthDeciMm);
    DebugAppendS(",rs=");
    DebugAppendI32(ToCenti(s_dbgRawSteer));
    DebugAppendS(",ts=");
    DebugAppendI32(ToCenti(s_dbgFinalSteer));
    DebugAppendS(",fw=");
    DebugAppendI32(ToCenti(s_dbgForward));
    DebugAppendS(",lt=");
    DebugAppendI32(ToCenti(s_dbgLeftTarget));
    DebugAppendS(",rt=");
    DebugAppendI32(ToCenti(s_dbgRightTarget));
    DebugAppendS(",lv=");
    DebugAppendI32(ToCenti(g_leftSpeed));
    DebugAppendS(",rv=");
    DebugAppendI32(ToCenti(g_rightSpeed));
    DebugAppendS(",lp=");
    DebugAppendI32((int32_t)g_leftPwm);
    DebugAppendS(",rp=");
    DebugAppendI32((int32_t)g_rightPwm);
    DebugAppendS(",crc=");
    DebugAppendU32(App_VisionLink_GetCrcErrorCount());
    DebugAppendS(",rxov=");
    DebugAppendU32(App_VisionLink_GetRxOverflowCount());
    DebugAppendS(",txdrop=");
    DebugAppendU32(DebugSerial_GetTxDropCount());
    DebugAppendS("\r\n");

    DebugSerial_TrySendBuffer(
        (const uint8_t *)s_debugLine, s_debugLineLen);
#endif
}

void App_VisionTrack_HandleKey(uint8_t key)
{
    if (key == 0U) return;

    switch (key)
    {
    case 2U:
        App_VisionLink_Reset();
        App_VisionLink_SendTrackMode();
        s_curFrame.transportValid = 0U;
        s_curFrame.visionValid    = 0U;
        App_Control_ForcePWMZero();
        s_state = VISION_TRACK_RUNNING;
#if VISION_TRACK_DEBUG_ENABLE
        DebugAppendEvent("VEVT,K2,track");
#endif
        break;

    case 3U:
        App_Control_ForcePWMZero();
        App_VisionLink_SendIdleMode();
        App_VisionLink_Reset();
        s_curFrame.transportValid = 0U;
        s_curFrame.visionValid    = 0U;
        s_state = VISION_TRACK_STOPPED;
#if VISION_TRACK_DEBUG_ENABLE
        DebugAppendEvent("VEVT,K3,stop");
#endif
        break;

    default:
        break;
    }
}
