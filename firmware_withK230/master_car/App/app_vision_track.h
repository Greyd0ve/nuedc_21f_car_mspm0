#ifndef __APP_VISION_TRACK_H
#define __APP_VISION_TRACK_H

#include <stdint.h>

#define VISION_TRACK_BASE_SPEED_CMPS   10.0f
#define VISION_TRACK_TURN_LIMIT_CMPS   6.0f
#define VISION_TRACK_KY                0.004f
#define VISION_TRACK_KA                0.02f
#define VISION_TRACK_KD                0.001f
#define VISION_TRACK_TURN_SIGN        -1.0f
#define VISION_TRACK_MIN_CONFIDENCE    70U
#define VISION_TRACK_FRESH_LIMIT_MS    150U
#define VISION_TRACK_ACQUIRE_FRAMES    3U

typedef enum
{
    VISION_TRACK_IDLE = 0,
    VISION_TRACK_ACQUIRE,
    VISION_TRACK_RUN,
    VISION_TRACK_LOST,
    VISION_TRACK_STOP
} VisionTrackState_t;

void App_VisionTrack_Init(void);
void App_VisionTrack_Task10ms(void);
void App_VisionTrack_Task100ms(void);

void App_VisionTrack_HandleKey(uint8_t key);

#endif
