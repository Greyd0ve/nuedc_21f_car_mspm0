#ifndef __APP_VISION_TRACK_H
#define __APP_VISION_TRACK_H

#include <stdint.h>

/*
 * Pure vision-only track mode parameters.
 *
 * K230 protocol convention:
 *   lateralErrorDeciMm > 0  -> road centre is to the RIGHT of the car.
 *   headingErrorCentiDeg > 0 -> road extends to the RIGHT of the car.
 *
 * MSPM0 control convention:
 *   g_targetTurnSpeed > 0 -> right-wheel target is HIGHER than left,
 *                            i.e. the car steers LEFT.
 *
 * VISION_TRACK_TURN_SIGN translates K230 eye-space to MSPM0 turn-space.
 * Default -1.0f:  positive ey (road to right) -> negative turn -> steer right.
 * Tune by observing which side the car drifts toward.
 */
#define VISION_TRACK_FORWARD_SPEED_CMPS      10.0f
#define VISION_TRACK_DEGRADED_SPEED_CMPS     4.5f
#define VISION_TRACK_KY                      0.008f
#define VISION_TRACK_KA                      0.0f
#define VISION_TRACK_KD                      0.0f
#define VISION_TRACK_TURN_LIMIT_CMPS         8.0f
#define VISION_TRACK_TURN_SIGN               -1.0f
#define VISION_TRACK_FRESH_LIMIT_MS          120U
#define VISION_TRACK_KY_SCALE                0.01f

#define VISION_TRACK_DEBUG_ENABLE            0

typedef enum
{
    VISION_TRACK_STOPPED = 0,
    VISION_TRACK_RUNNING
} VisionTrackState_t;

void App_VisionTrack_Init(void);
void App_VisionTrack_Task10ms(void);
void App_VisionTrack_Task100ms(void);

void App_VisionTrack_HandleKey(uint8_t key);

#endif
