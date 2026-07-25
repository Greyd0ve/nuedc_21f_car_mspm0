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
 *
 * VISION_TRACK_ERROR_DECI_MM_TO_MM:  convert K230 0.1mm units to mm.
 *   e.g. lateralErrorDeciMm=600 -> errorMm=60.0
 *
 * VISION_TRACK_KY_CMPS_PER_MM:  lateral error per mm produces how many cm/s
 *   of differential steer.  0.04f means 60mm error -> 2.4cm/s steer.
 */
#define VISION_TRACK_FORWARD_SPEED_CMPS      6.0f
#define VISION_TRACK_DEGRADED_SPEED_CMPS     4.5f
#define VISION_TRACK_ERROR_DECI_MM_TO_MM     0.1f
#define VISION_TRACK_KY_CMPS_PER_MM           0.04f
#define VISION_TRACK_TURN_LIMIT_CMPS         5.0f
#define VISION_TRACK_TURN_SIGN               -1.0f
#define VISION_TRACK_FRESH_LIMIT_MS          120U

#define VISION_TRACK_DEBUG_ENABLE            1

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
