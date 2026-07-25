#include "app_vision_track.h"
#include "app_vision_link.h"
#include "app_config.h"
#include "app_car_state.h"
#include "app_control.h"
#include <stdint.h>

static VisionTrackState_t s_state = VISION_TRACK_STOPPED;

static VisionTrackFrame_t s_curFrame;

static float clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint8_t IsFrameDrivable(const VisionTrackFrame_t *f)
{
    if (!f->transportValid) return 0U;
    if (!f->visionValid) return 0U;
    if (f->mode != VISION_MODE_TRACK) return 0U;
    if (App_VisionLink_GetFrameAgeMs() > VISION_TRACK_FRESH_LIMIT_MS) return 0U;
    return 1U;
}

/*
 * rawSteer > 0:  road centre is to the RIGHT, car needs to steer RIGHT.
 * This is in K230 eye-space.  VISION_TRACK_TURN_SIGN converts to MSPM0
 * turn-space before setting g_targetTurnSpeed.
 */
static float VisionTrack_CalcSteer(int16_t lateralErrorDeciMm)
{
    float errorMm;
    errorMm = (float)lateralErrorDeciMm *
              VISION_TRACK_ERROR_DECI_MM_TO_MM;
    return VISION_TRACK_KY_CMPS_PER_MM * errorMm;
}

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

    App_VisionLink_GetLatest(&s_curFrame);

    if (s_state != VISION_TRACK_RUNNING)
    {
        App_Control_ForcePWMZero();
        return;
    }

    drivable = IsFrameDrivable(&s_curFrame);

    if (!drivable)
    {
        App_Control_ForcePWMZero();
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
        App_VisionLink_Reset();
        App_VisionLink_SendTrackMode();
        s_curFrame.transportValid = 0U;
        s_curFrame.visionValid    = 0U;
        App_Control_ForcePWMZero();
        s_state = VISION_TRACK_RUNNING;
        break;

    case 3U:
        App_Control_ForcePWMZero();
        App_VisionLink_SendIdleMode();
        App_VisionLink_Reset();
        s_curFrame.transportValid = 0U;
        s_curFrame.visionValid    = 0U;
        s_state = VISION_TRACK_STOPPED;
        break;

    default:
        break;
    }
}
