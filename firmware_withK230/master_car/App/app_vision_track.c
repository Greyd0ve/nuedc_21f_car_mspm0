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

static float CalcSteer(int16_t lateralErrorDeciMm,
                       int16_t headingErrorCentiDeg,
                       float dEy)
{
    float ey  = (float)lateralErrorDeciMm * VISION_TRACK_KY_SCALE;
    float hdg = (float)headingErrorCentiDeg;
    return VISION_TRACK_KY * ey +
           VISION_TRACK_KA * hdg +
           VISION_TRACK_KD * dEy;
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

    steer = VISION_TRACK_TURN_SIGN *
        CalcSteer(s_curFrame.lateralErrorDeciMm,
                  s_curFrame.headingErrorCentiDeg,
                  0.0f);

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
        App_VisionTrack_Init();
        App_Control_ResetPID();
        App_VisionLink_SendTrackMode();
        s_state = VISION_TRACK_RUNNING;
        break;

    case 3U:
        App_Control_ForcePWMZero();
        App_Control_ResetPID();
        App_VisionLink_SendIdleMode();
        App_VisionLink_Reset();
        App_VisionTrack_Init();
        break;

    default:
        break;
    }
}
