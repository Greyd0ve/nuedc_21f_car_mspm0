#ifndef __APP_VISION_TRACK_H
#define __APP_VISION_TRACK_H

#include <stdint.h>

#define VISION_TRACK_MAX_SPEED_CMPS          10.0f
#define VISION_TRACK_MIN_SPEED_CMPS           4.5f
#define VISION_TRACK_DEGRADED_SPEED_CMPS      4.8f

#define VISION_TRACK_KY                       0.008f
#define VISION_TRACK_KA                       0.040f
#define VISION_TRACK_KD                       0.0015f
#define VISION_TRACK_TURN_LIMIT_CMPS         10.0f
#define VISION_TRACK_TURN_SIGN                1.0f

#define VISION_TRACK_EY_FULL_SLOW_MM         45.0f
#define VISION_TRACK_EA_FULL_SLOW_DEG        12.0f
#define VISION_TRACK_EY_DECI_MM_TO_MM         0.1f
#define VISION_TRACK_EA_CENTI_DEG_TO_DEG     0.01f

#define VISION_TRACK_DECEL_STEP_CMPS          0.5f
#define VISION_TRACK_ACCEL_STEP_CMPS          0.15f
#define VISION_TRACK_DEGRADED_TURN_STEP_CMPS  0.5f

#define VISION_TRACK_FRESH_LIMIT_MS           150U
#define VISION_TRACK_ACQUIRE_FRAMES           3U
#define VISION_TRACK_INVALID_CONFIRM_FRAMES   3U

/*
 * UART_DEBUG is 9600 baud.  A complete diagnostic record is deliberately
 * scheduled at 500 ms and transmitted non-blocking so it cannot stall the
 * 10 ms control loop.
 */
#define VISION_TRACK_DEBUG_ENABLE              1
#define VISION_TRACK_DEBUG_PERIOD_MS           500U
#define VISION_TRACK_DEBUG_BUFFER_SIZE         384U

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
