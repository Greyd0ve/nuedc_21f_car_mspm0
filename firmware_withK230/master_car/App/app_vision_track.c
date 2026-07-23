#include "app_vision_track.h"
#include "app_vision_link.h"
#include "app_config.h"
#include "app_car_state.h"
#include "app_control.h"
#include "Motor.h"
#include <stdint.h>

static VisionTrackState_t s_state = VISION_TRACK_IDLE;

static int16_t  s_lastEy    = 0;
static uint8_t  s_hasLastEy = 0U;

static uint8_t  s_acquireCount   = 0U;
static uint16_t s_lastAcquireSeq = 0xFFFFU;

static uint8_t  s_lostRecoverCount = 0U;
static uint16_t s_lastRecoverSeq   = 0xFFFFU;

static uint32_t s_lastOverflowCount = 0U;

static VisionTrackFrame_t s_curFrame;

static int16_t Vision_CentiDegToDeciDeg(int16_t value)
{
    if (value >= 0)
        return (int16_t)((value + 5) / 10);
    return (int16_t)((value - 5) / 10);
}

static float LimitFloat(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint8_t IsVisionDataValid(const VisionTrackFrame_t *f)
{
    if (!f->transportValid) return 0U;
    if (!f->visionValid) return 0U;
    if (App_VisionLink_GetFrameAgeMs() > VISION_TRACK_FRESH_LIMIT_MS) return 0U;
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
}

void App_VisionTrack_Init(void)
{
    s_state      = VISION_TRACK_IDLE;
    s_lastEy     = 0;
    s_hasLastEy  = 0U;
    s_acquireCount   = 0U;
    s_lastAcquireSeq = 0xFFFFU;
    s_lostRecoverCount = 0U;
    s_lastRecoverSeq   = 0xFFFFU;
    s_lastOverflowCount = 0U;
    s_curFrame.transportValid = 0U;
    s_curFrame.visionValid    = 0U;

    SafeStop();
}

void App_VisionTrack_Task10ms(void)
{
    uint8_t  isNewSeq;
    uint8_t  dataValid;
    int16_t  eyControl;
    int16_t  eaControl;
    float    turnCmd;
    float    dEy;
    uint32_t curOverflow;

    (void)App_VisionLink_GetLatest(&s_curFrame);
    isNewSeq    = App_VisionLink_HasNewFrame();
    dataValid   = IsVisionDataValid(&s_curFrame);
    curOverflow = App_VisionLink_GetRxOverflowCount();

    switch (s_state)
    {
    case VISION_TRACK_IDLE:
        SafeStop();
        break;

    case VISION_TRACK_ACQUIRE:
        SafeStop();
        if (!dataValid)
        {
            s_acquireCount   = 0U;
            s_lastAcquireSeq = 0xFFFFU;
        }
        else if (isNewSeq && s_curFrame.sequence != s_lastAcquireSeq)
        {
            s_lastAcquireSeq = s_curFrame.sequence;
            s_acquireCount++;
            if (s_acquireCount >= VISION_TRACK_ACQUIRE_FRAMES)
            {
                s_hasLastEy = 0U;
                s_lastOverflowCount = App_VisionLink_GetRxOverflowCount();
                EnterState(VISION_TRACK_RUN);
            }
        }
        break;

    case VISION_TRACK_RUN:
        if (!dataValid || (curOverflow != s_lastOverflowCount))
        {
            SafeStop();
            s_lostRecoverCount = 0U;
            s_lastRecoverSeq   = 0xFFFFU;
            if (curOverflow != s_lastOverflowCount)
            {
                App_VisionLink_Reset();
            }
            EnterState(VISION_TRACK_LOST);
            break;
        }

        eyControl = s_curFrame.lateralErrorDeciMm;
        eaControl = Vision_CentiDegToDeciDeg(s_curFrame.headingErrorCentiDeg);

        if (isNewSeq)
        {
            if (s_hasLastEy)
            {
                dEy = (float)(eyControl - s_lastEy);
            }
            else
            {
                dEy = 0.0f;
            }
            s_lastEy    = eyControl;
            s_hasLastEy = 1U;
        }
        else
        {
            dEy = 0.0f;
        }

        turnCmd = VISION_TRACK_TURN_SIGN *
            (VISION_TRACK_KY * (float)eyControl +
             VISION_TRACK_KA * (float)eaControl +
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
        if (dataValid && isNewSeq && s_curFrame.sequence != s_lastRecoverSeq)
        {
            s_lastRecoverSeq = s_curFrame.sequence;
            s_lostRecoverCount++;
            if (s_lostRecoverCount >= VISION_TRACK_ACQUIRE_FRAMES)
            {
                s_hasLastEy = 0U;
                s_lostRecoverCount = 0U;
                s_lastRecoverSeq   = 0xFFFFU;
                s_lastOverflowCount = App_VisionLink_GetRxOverflowCount();
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
}

void App_VisionTrack_HandleKey(uint8_t key)
{
    if (key == 0U) return;

    switch (key)
    {
    case 2U:
        if (s_state == VISION_TRACK_IDLE)
        {
            App_VisionLink_Reset();
            App_VisionTrack_Init();
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
