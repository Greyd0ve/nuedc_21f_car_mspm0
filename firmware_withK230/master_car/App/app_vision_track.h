#ifndef __APP_VISION_TRACK_H
#define __APP_VISION_TRACK_H

#include <stdint.h>

#define VISION_TRACK_MAX_SPEED_CMPS          25.0f
#define VISION_TRACK_MIN_SPEED_CMPS          12.0f
#define VISION_TRACK_DEGRADED_SPEED_CMPS      4.5f
#define VISION_TRACK_DEGRADED_GRACE_SPEED_CMPS 6.5f
#define VISION_TRACK_DEGRADED_GRACE_FRAMES      2U
#define VISION_TRACK_CURVE_HOLD_SPEED_CMPS   20.0f

/*
 * Normal visual tracking uses forward +/- turn wheel targets.  Retain a
 * meaningful rolling target for the inside wheel, so a broad curve remains
 * a continuous arc instead of becoming a one-wheel pivot correction.
 */
#define VISION_TRACK_MIN_INNER_WHEEL_SPEED_CMPS 3.0f

/*
 * Controller input units:
 *   ey  = lateralErrorDeciMm, 0.1 mm/count.
 *   ea  = headingErrorCentiDeg converted to 0.1 degree/count.
 *   dEy = new-frame ey difference, not divided by elapsed time.
 */
#define VISION_TRACK_KY                       0.008f
#define VISION_TRACK_KA                       0.028f
#define VISION_TRACK_KD                       0.0015f
#define VISION_TRACK_TURN_LIMIT_CMPS         10.0f

/*
 * K230 positive ey/ea means the road is to the right.  In the current
 * rear-camera motor mapping, a positive turn makes the car steer right.
 * Keep the visual error and differential-drive correction in the same
 * direction: road right -> steer right; road left -> steer left.
 */
#define VISION_TRACK_TURN_SIGN               (+1.0f)
#define VISION_TRACK_HEADING_SIGN            (+1.0f)

#define VISION_TRACK_EY_FULL_SLOW_MM         80.0f
#define VISION_TRACK_EA_FULL_SLOW_DEG        20.0f
#define VISION_TRACK_EY_DECI_MM_TO_MM         0.1f
#define VISION_TRACK_EA_CENTI_DEG_TO_DEG     0.01f
#define VISION_TRACK_EA_DECI_DEG_TO_DEG       0.1f

/*
 * In rear-camera / reversed-heading mode, K230 observes the road at a point
 * about 150 mm ahead of the drive-axle midpoint in the new forward direction.
 * Track a virtual point 60 mm ahead of the axle midpoint.  On a curved road,
 * this avoids converting the complete image tangent into an immediate axle
 * correction, which otherwise creates alternating left/right commands.
 * The linear small-angle projection is bounded and is used only with a
 * confident, full-boundary frame.
 */
#define VISION_TRACK_CAMERA_LOOKAHEAD_MM       150.0f
#define VISION_TRACK_VIRTUAL_LOOKAHEAD_MM       60.0f
/* K230: heading < 0 means the road advances toward image left. */
#define VISION_TRACK_PREVIEW_HEADING_SIGN       -1.0f
#define VISION_TRACK_PREVIEW_RAD_PER_DECI_DEG 0.00174533f
#define VISION_TRACK_PREVIEW_MAX_COMP_MM        25.0f
#define VISION_TRACK_PREVIEW_EA_LIMIT_DEG       15.0f
#define VISION_TRACK_PREVIEW_STEP_DECI_MM       25.0f

#define VISION_TRACK_EY_HEADING_GUARD_DECI_MM 150.0f
#define VISION_TRACK_OPPOSING_EA_RATIO        0.35f
#define VISION_TRACK_D_TERM_LIMIT_CMPS        0.50f
#define VISION_TRACK_HEADING_CONTROL_LIMIT_DEG 12.0f
#define VISION_TRACK_HEADING_TERM_LIMIT_CMPS   2.00f
#define VISION_TRACK_HEADING_MIN_SCALE          0.30f
#define VISION_TRACK_HEADING_FULL_EY_DECI_MM  150.0f
#define VISION_TRACK_HEADING_LOW_CONF_SCALE     0.35f

/*
 * Keep normal centering gain near the road centre.  On a deep bend, reduce
 * the complete turn command smoothly instead of pulling the vehicle toward
 * the inside of the curve with a large camera-preview error.
 */
#define VISION_TRACK_TURN_ATTENUATE_START_DECI_MM 100.0f
#define VISION_TRACK_TURN_ATTENUATE_FULL_DECI_MM  250.0f
#define VISION_TRACK_LARGE_ERROR_MIN_TURN_SCALE     0.65f

#define VISION_TRACK_EY_FILTER_ALPHA          0.35f
#define VISION_TRACK_EA_FILTER_ALPHA          0.25f
#define VISION_TRACK_CURVE_EY_FILTER_ALPHA    0.22f
#define VISION_TRACK_CURVE_EA_FILTER_ALPHA    0.18f
#define VISION_TRACK_DEGRADED_EY_FILTER_ALPHA 0.18f
#define VISION_TRACK_DEGRADED_EA_DECAY        0.85f
#define VISION_TRACK_EY_JUMP_REJECT_DECI_MM  300.0f

#define VISION_TRACK_DECEL_STEP_CMPS          1.80f
#define VISION_TRACK_ACCEL_STEP_CMPS          1.00f
#define VISION_TRACK_TURN_STEP_CMPS           0.65f
#define VISION_TRACK_TURN_DECAY_STEP_CMPS     0.35f
#define VISION_TRACK_DEGRADED_TURN_STEP_CMPS  0.50f
#define VISION_TRACK_TURN_REVERSAL_DEADBAND_CMPS 0.50f
#define VISION_TRACK_TURN_REVERSAL_CONFIRM_FRAMES 3U

#define VISION_TRACK_CURVE_DIRECTION_LOCK_FRAMES 3U
#define VISION_TRACK_CURVE_REVERSE_CONFIRM_FRAMES 6U
/* Reverse only after a reliable, raw camera cross-track displacement. */
#define VISION_TRACK_CURVE_REVERSE_EY_DECI_MM 250.0f
#define VISION_TRACK_CURVE_LOCK_HOLD_TURN_CMPS 0.60f

#define VISION_TRACK_CURVE_HOLD_FRAMES        8U
#define VISION_TRACK_CURVE_RELEASE_SEVERITY   0.35f
#define VISION_TRACK_CURVE_TRIGGER_SEVERITY   0.70f
#define VISION_TRACK_TRUSTED_CONFIDENCE       80U

#define VISION_TRACK_FRESH_LIMIT_MS           150U
#define VISION_TRACK_ACQUIRE_FRAMES           3U
#define VISION_TRACK_INVALID_CONFIRM_FRAMES   3U
#define VISION_TRACK_MIN_CONFIDENCE           30U

/*
 * UART_DEBUG is the 9600-baud Bluetooth/debug port.  A full self-describing
 * record needs nearly 0.5 s on the wire, so 600 ms is the safe default.
 */
#define VISION_TRACK_DEBUG_ENABLE              1
#define VISION_TRACK_DEBUG_PERIOD_MS           600U
#define VISION_TRACK_DEBUG_BUFFER_SIZE         512U

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
