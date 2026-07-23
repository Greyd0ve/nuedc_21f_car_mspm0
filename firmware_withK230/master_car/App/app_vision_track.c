#include "app_vision_track.h"
#include "app_vision_link.h"
#include "app_config.h"
#include "app_car_state.h"
#include "app_control.h"
#include "DebugSerial.h"
#include "Motor.h"
#include <stdint.h>

static VisionTrackState_t s_state = VISION_TRACK_IDLE;
static uint32_t           s_stateMs = 0U;

static int16_t            s_lastEy = 0;
static uint8_t            s_hasLastEy = 0U;

static uint8_t            s_acquireCount = 0U;
static uint16_t           s_lastAcquireSeq = 0xFFFFU;

static uint8_t            s_lostRecoverCount = 0U;
static uint16_t           s_lastRecoverSeq = 0xFFFFU;

static VisionTrackFrame_t s_curFrame;

static float LimitFloat(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint8_t IsVisionDataValid(const VisionTrackFrame_t *f)
{
    uint32_t ageMs;
    if (!f->frameValid) return 0U;

    ageMs = App_VisionLink_GetFrameAgeMs();
    if (ageMs > VISION_TRACK_FRESH_LIMIT_MS) return 0U;

    if (f->flags & 0x80U) return 0U;
    if ((f->flags & 0x03U) != 0x03U) return 0U;
    if (f->confidence < VISION_TRACK_MIN_CONFIDENCE) return 0U;

    return 1U;
}

static void SafeStop(void)
{
    App_Control_ForcePWMZero();
    Motor_StopAll();
    g_carEnable = 0U;
}

static void EnterState(VisionTrackState_t next)
{
    if (s_state == next) return;
    s_state = next;
    s_stateMs = 0U;
}

void App_VisionTrack_Init(void)
{
    s_state      = VISION_TRACK_IDLE;
    s_stateMs    = 0U;
    s_lastEy     = 0;
    s_hasLastEy  = 0U;
    s_acquireCount   = 0U;
    s_lastAcquireSeq = 0xFFFFU;
    s_lostRecoverCount = 0U;
    s_lastRecoverSeq   = 0xFFFFU;
    s_curFrame.frameValid = 0U;

    SafeStop();
    App_VisionLink_SendIdleMode();
}

void App_VisionTrack_Task10ms(void)
{
    uint8_t    hasFrame;
    uint8_t    isNewSeq;
    uint8_t    dataValid;
    int16_t    ey;
    int16_t    ea;
    float      turnCmd;
    float      dEy;

    (void)App_VisionLink_GetLatest(&s_curFrame);
    hasFrame = s_curFrame.frameValid;
    isNewSeq = App_VisionLink_HasNewFrame();
    dataValid = IsVisionDataValid(&s_curFrame);

    s_stateMs += CAR_CONTROL_PERIOD_MS;

    switch (s_state)
    {
    case VISION_TRACK_IDLE:
        SafeStop();
        break;

    case VISION_TRACK_ACQUIRE:
        SafeStop();
        if (!dataValid)
        {
            s_acquireCount = 0U;
            s_lastAcquireSeq = 0xFFFFU;
        }
        else if (isNewSeq && s_curFrame.seq != s_lastAcquireSeq)
        {
            s_lastAcquireSeq = s_curFrame.seq;
            s_acquireCount++;
            if (s_acquireCount >= VISION_TRACK_ACQUIRE_FRAMES)
            {
                s_hasLastEy = 0U;
                EnterState(VISION_TRACK_RUN);
            }
        }
        else if (App_VisionLink_GetFrameAgeMs() > VISION_TRACK_TIMEOUT_MS)
        {
            s_acquireCount = 0U;
        }
        break;

    case VISION_TRACK_RUN:
        if (!dataValid)
        {
            SafeStop();
            s_lostRecoverCount = 0U;
            s_lastRecoverSeq   = 0xFFFFU;
            EnterState(VISION_TRACK_LOST);
            break;
        }

        ey = s_curFrame.lateralErrorQ1000;
        ea = s_curFrame.headingErrorDeciDeg;

        s_lastEy    = ey;
        s_hasLastEy = 1U;

        if (isNewSeq && s_hasLastEy)
        {
            dEy = (float)(s_curFrame.lateralErrorQ1000 - s_lastEy);
        }
        else
        {
            dEy = 0.0f;
        }

        turnCmd = VISION_TRACK_TURN_SIGN *
            (VISION_TRACK_KY * (float)ey +
             VISION_TRACK_KA * (float)ea +
             VISION_TRACK_KD * dEy);

        turnCmd = LimitFloat(turnCmd,
            -VISION_TRACK_TURN_LIMIT_CMPS,
             VISION_TRACK_TURN_LIMIT_CMPS);

        g_targetForwardSpeed = VISION_TRACK_BASE_SPEED_CMPS;
        g_targetTurnSpeed    = turnCmd;
        g_carEnable = 1U;
        App_Control_ApplyMotorOutput();
        break;

    case VISION_TRACK_LOST:
        SafeStop();
        if (dataValid && isNewSeq && s_curFrame.seq != s_lastRecoverSeq)
        {
            s_lastRecoverSeq = s_curFrame.seq;
            s_lostRecoverCount++;
            if (s_lostRecoverCount >= VISION_TRACK_ACQUIRE_FRAMES)
            {
                s_hasLastEy = 0U;
                s_lostRecoverCount = 0U;
                s_lastRecoverSeq   = 0xFFFFU;
                EnterState(VISION_TRACK_RUN);
            }
        }
        else if (!dataValid)
        {
            s_lostRecoverCount = 0U;
        }
        break;

    case VISION_TRACK_STOP:
        SafeStop();
        break;

    default:
        SafeStop();
        EnterState(VISION_TRACK_IDLE);
        break;
    }
}

void App_VisionTrack_Task100ms(void)
{
    VisionTrackFrame_t f;
    uint8_t hasFrame;

    hasFrame = App_VisionLink_GetLatest(&f);

    if (hasFrame)
    {
        DebugSerial_Printf(
            "[vision,state=%u,seq=%u,age=%lu,ey=%d,ea=%d,flags=%u,conf=%u,ovf=%lu,perr=%lu]\r\n",
            (unsigned int)s_state,
            (unsigned int)f.seq,
            (unsigned long)App_VisionLink_GetFrameAgeMs(),
            (int)f.lateralErrorQ1000,
            (int)f.headingErrorDeciDeg,
            (unsigned int)f.flags,
            (unsigned int)f.confidence,
            (unsigned long)0UL,
            (unsigned long)App_VisionLink_GetParseErrorCount());
    }
}

void App_VisionTrack_HandleKey(uint8_t key)
{
    if (key == 0U) return;

    switch (key)
    {
    case 2U:
        if (s_state == VISION_TRACK_IDLE)
        {
            App_VisionTrack_Init();
            App_VisionLink_Reset();
            s_acquireCount   = 0U;
            s_lastAcquireSeq = 0xFFFFU;
            App_VisionLink_SendTrackMode();
            EnterState(VISION_TRACK_ACQUIRE);
        }
        break;

    case 3U:
        SafeStop();
        App_VisionLink_SendIdleMode();
        EnterState(VISION_TRACK_STOP);
        break;

    case 4U:
        SafeStop();
        App_VisionLink_SendIdleMode();
        App_VisionLink_Reset();
        App_VisionTrack_Init();
        break;

    default:
        break;
    }
}
