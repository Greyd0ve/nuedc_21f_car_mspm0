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

#define VISION_DEBUG_INVALID_NO_TRANSPORT   0x01U
#define VISION_DEBUG_INVALID_VISION         0x02U
#define VISION_DEBUG_INVALID_MODE           0x04U
#define VISION_DEBUG_INVALID_STALE          0x08U
#define VISION_DEBUG_INVALID_STOPPED        0x10U

#if VISION_TRACK_DEBUG_ENABLE
#define VISION_DEBUG_LINE_BUFFER_SIZE 80U
#define VISION_DEBUG_LINE_MAX_LENGTH  75U
#define VISION_DEBUG_EVENT_BUF_SIZE   20U

static uint8_t  s_debugLine[VISION_DEBUG_LINE_BUFFER_SIZE];
static uint16_t s_debugLineLen = 0U;
static uint8_t  s_debugDivider = 0U;
static uint8_t  s_debugHeaderSent = 0U;

static uint16_t s_dbgSequence;
static uint32_t s_dbgFrameAgeMs;
static uint8_t  s_dbgDrivable;
static uint8_t  s_dbgInvalidReason;
static uint8_t  s_dbgVisionBits;

static float s_dbgFinalSteer;
static float s_dbgLeftTarget;
static float s_dbgRightTarget;
#endif

static VisionTrackState_t s_state = VISION_TRACK_STOPPED;
static VisionTrackFrame_t s_curFrame;

static float clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float VisionTrack_CalcSteer(int16_t lateralErrorDeciMm)
{
    float errorMm;
    errorMm = (float)lateralErrorDeciMm *
              VISION_TRACK_ERROR_DECI_MM_TO_MM;
    return VISION_TRACK_KY_CMPS_PER_MM * errorMm;
}

static uint8_t VisionTrack_GetInvalidReason(
    const VisionTrackFrame_t *frame,
    uint32_t frameAgeMs)
{
    uint8_t reason = 0U;

    if (!frame->transportValid)
        reason |= VISION_DEBUG_INVALID_NO_TRANSPORT;
    if (!frame->visionValid)
        reason |= VISION_DEBUG_INVALID_VISION;
    if (frame->mode != VISION_MODE_TRACK)
        reason |= VISION_DEBUG_INVALID_MODE;
    if (frameAgeMs > VISION_TRACK_FRESH_LIMIT_MS)
        reason |= VISION_DEBUG_INVALID_STALE;
    if (s_state != VISION_TRACK_RUNNING)
        reason |= VISION_DEBUG_INVALID_STOPPED;

    return reason;
}

#if VISION_TRACK_DEBUG_ENABLE
static int32_t VisionDebug_ToCenti(float value)
{
    if (value >= 0.0f)
        return (int32_t)(value * 100.0f + 0.5f);
    return (int32_t)(value * 100.0f - 0.5f);
}

static void dbgAppendC(char c)
{
    if (s_debugLineLen < (VISION_DEBUG_LINE_BUFFER_SIZE - 1U))
        s_debugLine[s_debugLineLen++] = (uint8_t)c;
}

static void dbgAppendS(const char *s)
{
    while (*s && (s_debugLineLen < (VISION_DEBUG_LINE_BUFFER_SIZE - 1U)))
        s_debugLine[s_debugLineLen++] = (uint8_t)*s++;
}

static void dbgAppendU32(uint32_t v)
{
    char b[10]; uint8_t i = 0U;
    do { b[i++] = (char)('0' + (v % 10U)); v /= 10U; } while (v > 0U);
    while (i > 0U) dbgAppendC(b[--i]);
}

static void dbgAppendI32(int32_t v)
{
    if (v < 0) { dbgAppendC('-'); dbgAppendU32((uint32_t)(-(v + 1)) + 1U); }
    else dbgAppendU32((uint32_t)v);
}

static void dbgAppendHex8(uint8_t v)
{
    uint8_t hi = (v >> 4) & 0xFU;
    uint8_t lo = v & 0xFU;
    dbgAppendC((char)(hi < 10U ? '0' + hi : 'A' + hi - 10U));
    dbgAppendC((char)(lo < 10U ? '0' + lo : 'A' + lo - 10U));
}

static void dbgAppendEvent(const char *txt)
{
    uint8_t buf[VISION_DEBUG_EVENT_BUF_SIZE];
    uint8_t n = 0U;
    while (*txt && n < (VISION_DEBUG_EVENT_BUF_SIZE - 2U)) buf[n++] = (uint8_t)*txt++;
    buf[n++] = '\r'; buf[n++] = '\n';
    DebugSerial_TrySendBuffer(buf, (uint16_t)n);
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
    uint8_t visionBits;
#endif

    App_VisionLink_GetLatest(&s_curFrame);

#if VISION_TRACK_DEBUG_ENABLE
    frameAgeMs = App_VisionLink_GetFrameAgeMs();
    invalidReason = VisionTrack_GetInvalidReason(&s_curFrame, frameAgeMs);
    drivable = (invalidReason == 0U);

    visionBits = (uint8_t)((s_curFrame.mode & 0x07U) << 5);
    if (s_curFrame.transportValid)   visionBits |= 0x10U;
    if (s_curFrame.visionValid)      visionBits |= 0x08U;
    if (s_curFrame.degraded)         visionBits |= 0x04U;
    if (s_curFrame.leftBoundaryValid) visionBits |= 0x02U;
    if (s_curFrame.rightBoundaryValid)visionBits |= 0x01U;

    s_dbgSequence      = s_curFrame.sequence;
    s_dbgFrameAgeMs    = frameAgeMs;
    s_dbgDrivable      = drivable;
    s_dbgInvalidReason = invalidReason;
    s_dbgVisionBits    = visionBits;
#else
    drivable = (VisionTrack_GetInvalidReason(
        &s_curFrame, App_VisionLink_GetFrameAgeMs()) == 0U);
#endif

    if (s_state != VISION_TRACK_RUNNING)
    {
        App_Control_ForcePWMZero();
#if VISION_TRACK_DEBUG_ENABLE
        s_dbgFinalSteer = 0.0f;
        s_dbgLeftTarget = 0.0f;
        s_dbgRightTarget = 0.0f;
#endif
        return;
    }

    if (!drivable)
    {
        App_Control_ForcePWMZero();
#if VISION_TRACK_DEBUG_ENABLE
        s_dbgFinalSteer = 0.0f;
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
    s_dbgFinalSteer = steer;
    s_dbgLeftTarget = forward - steer;
    s_dbgRightTarget = forward + steer;
#endif
}

void App_VisionTrack_Task100ms(void)
{
#if VISION_TRACK_DEBUG_ENABLE
    uint32_t ageMs;

    s_debugDivider++;
    if (s_debugDivider < 2U) return;
    s_debugDivider = 0U;

    if (s_debugHeaderSent == 0U)
    {
        s_debugLineLen = 0U;
        dbgAppendS("#D,q,a,k,i,v,e,h,u,l,r,L,R,p,P\r\n");
        if (DebugSerial_TrySendBuffer(s_debugLine, s_debugLineLen))
            s_debugHeaderSent = 1U;
        return;
    }

    ageMs = s_dbgFrameAgeMs;
    if (ageMs > 999U) ageMs = 999U;

    s_debugLineLen = 0U;
    dbgAppendS("D,");
    dbgAppendU32((uint32_t)s_dbgSequence);
    dbgAppendC(',');
    dbgAppendU32(ageMs);
    dbgAppendC(',');
    dbgAppendU32((uint32_t)s_dbgDrivable);
    dbgAppendC(',');
    dbgAppendHex8(s_dbgInvalidReason);
    dbgAppendC(',');
    dbgAppendHex8(s_dbgVisionBits);
    dbgAppendC(',');
    dbgAppendI32((int32_t)s_curFrame.lateralErrorDeciMm);
    dbgAppendC(',');
    dbgAppendI32((int32_t)s_curFrame.headingErrorCentiDeg);
    dbgAppendC(',');
    dbgAppendI32(VisionDebug_ToCenti(s_dbgFinalSteer));
    dbgAppendC(',');
    dbgAppendI32(VisionDebug_ToCenti(s_dbgLeftTarget));
    dbgAppendC(',');
    dbgAppendI32(VisionDebug_ToCenti(s_dbgRightTarget));
    dbgAppendC(',');
    dbgAppendI32(VisionDebug_ToCenti(g_leftSpeed));
    dbgAppendC(',');
    dbgAppendI32(VisionDebug_ToCenti(g_rightSpeed));
    dbgAppendC(',');
    dbgAppendI32((int32_t)g_leftPwm);
    dbgAppendC(',');
    dbgAppendI32((int32_t)g_rightPwm);
    dbgAppendS("\r\n");

    if (s_debugLineLen >= 60U && s_debugLineLen <= VISION_DEBUG_LINE_MAX_LENGTH)
        DebugSerial_TrySendBuffer(s_debugLine, s_debugLineLen);
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
        dbgAppendEvent("VEVT,K2,track");
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
        dbgAppendEvent("VEVT,K3,stop");
#endif
        break;

    default:
        break;
    }
}
