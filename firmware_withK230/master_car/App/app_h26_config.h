#ifndef __APP_H26_CONFIG_H
#define __APP_H26_CONFIG_H

#include "app_config.h"

/* H26 task-2 control and confirmation parameters.  Calibrate on the real track. */
#define H26_T2_STRAIGHT_SPEED_CMPS          42.0f
#define H26_T2_CURVE_SPEED_CMPS             34.0f
#define H26_T2_FINISH_SPEED_CMPS            15.0f
#define H26_T2_TURN_LIMIT_CMPS               6.0f

/* Turn-command hysteresis for straight/curve speed-zone selection. */
#define H26_T2_CURVE_ENTER_TURN_CMPS         2.5f
#define H26_T2_CURVE_EXIT_TURN_CMPS          1.2f
#define H26_T2_CURVE_ENTER_HOLD_MS          50U
#define H26_T2_CURVE_EXIT_HOLD_MS          100U

/* Per-10-ms command ramp; avoids an abrupt speed step at zone boundaries. */
#define H26_T2_SPEED_SLEW_CMPS_PER_TICK      0.8f

/* Slow down before the guarded A-point finish window; not a finish predicate. */
#define H26_T2_FINISH_SLOWDOWN_DISTANCE_CM 550.0f
#define H26_T2_FINISH_SLOWDOWN_MIN_TIME_MS 12000U

#define H26_T2_LEAVE_DISTANCE_CM            15.0f
#define H26_T2_LEAVE_LINE_STABLE_MS         80U

/* False-trigger guards only; the four-black-channel rule remains decisive. */
#define H26_T2_MIN_FINISH_DISTANCE_CM      500.0f
#define H26_T2_MIN_FINISH_TIME_MS        10000U
#define H26_T2_MAX_RUN_TIME_MS           30000U

#define H26_T2_FINISH_BLACK_CHANNELS        4U
#define H26_T2_FINISH_HOLD_MS               30U

#define H26_T2_STOP_SPEED_CMPS               1.0f
#define H26_T2_STOP_HOLD_MS                 100U
#define H26_T2_LINE_LOST_FAULT_MS     ECAR_LINE_LOST_FAULT_MS

/*
 * H26 task-3 (stationary vehicle ball motion) parameters.
 *
 * The K230 sends the ball's longitudinal location in 0.01 cm units over
 * UART1.  The camera script maps its detected pipe endpoints to 0..25 cm,
 * therefore O is the geometric centre (12.50 cm).  Confirm the physical
 * direction on the assembled mechanism before a run: change only
 * H26_T3_STEPPER_SIGN_FOR_POSITIVE_BALL when the rod direction is reversed.
 */
#define H26_T3_PIPE_LENGTH_CENTICM          2500U
#define H26_T3_CENTER_CM                    12.50f
#define H26_T3_OFFSET_CM                     5.00f
#define H26_T3_START_O_TOLERANCE_CM          0.80f
#define H26_T3_TARGET_TOLERANCE_CM           0.80f
#define H26_T3_COMMAND_DEADBAND_CM           0.12f
#define H26_T3_STABLE_SPEED_CMPS             0.80f

#define H26_T3_ACQUIRE_HOLD_MS              100U
#define H26_T3_TARGET_HOLD_MS               150U
#define H26_T3_ACQUIRE_TIMEOUT_MS          1000U
#define H26_T3_MAX_RUN_TIME_MS             5000U

/* K230 packets are emitted at 50 Hz; reject a held / stale vision result. */
#define H26_T3_MIN_CONFIDENCE                50U
#define H26_T3_MAX_FRAME_AGE_MS              60U
#define H26_T3_MIN_SPEED_DT_MS               10U
#define H26_T3_MAX_SPEED_DT_MS              100U
#define H26_T3_SPEED_FILTER_ALPHA          0.55f

/* Stepper velocity command tuning, in STEP pulse frequency (Hz). */
#define H26_T3_POSITION_KP_HZ_PER_CM       180.0f
#define H26_T3_SPEED_KD_HZ_PER_CMPS         90.0f
#define H26_T3_MIN_COMMAND_HZ                60U
#define H26_T3_MAX_COMMAND_HZ               700U
#define H26_T3_COMMAND_SLEW_HZ_PER_TICK      40U
#define H26_T3_STEPPER_SIGN_FOR_POSITIVE_BALL 1

#define H26_LED_SHOW_ON_MS                 200U
#define H26_LED_SHOW_OFF_MS                200U
#define H26_LED_FINISHED_TOGGLE_MS         500U
#define H26_LED_FAULT_TOGGLE_MS            100U

#define H26_DEBUG_PERIOD_MS                500U

#endif
