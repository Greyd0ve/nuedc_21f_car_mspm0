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
 * UART1.  Task 3 never uses that 0..25 cm camera coordinate directly: it is
 * converted to the signed competition coordinate where O = 0.00 cm, the
 * positive target is +5.00 cm, and the negative target is -5.00 cm.
 *
 * First-car calibration:
 *  1. Place the ball at physical O and record raw position_centi_cm.
 *  2. Place it at physical +5 cm and record H26_T3_RAW_POS5_CENTICM.
 *  3. Place it at physical -5 cm and record H26_T3_RAW_NEG5_CENTICM.
 * The +5/-5 raw values must lie on opposite sides of raw O.  This permits a
 * reversed camera image without changing control code.
 */
#define H26_T3_PIPE_LENGTH_CENTICM          2500U
#define H26_T3_RAW_MIN_CENTICM                  0U
#define H26_T3_RAW_MAX_CENTICM               2500U
#define H26_T3_RAW_O_CENTICM                 1250U
#define H26_T3_RAW_POS5_CENTICM              1750U
#define H26_T3_RAW_NEG5_CENTICM               750U
#define H26_T3_TARGET_POSITIVE_CM              5.00f
#define H26_T3_TARGET_NEGATIVE_CM             -5.00f
#define H26_T3_START_O_TOLERANCE_CM            0.60f
#define H26_T3_TARGET_TOLERANCE_CM             0.60f
#define H26_T3_COMMAND_DEADBAND_CM           0.12f
#define H26_T3_STABLE_SPEED_CMPS             0.80f

#define H26_T3_ACQUIRE_HOLD_MS              100U
#define H26_T3_TARGET_HOLD_MS               150U
#define H26_T3_ACQUIRE_TIMEOUT_MS          1000U
#define H26_T3_MAX_RUN_TIME_MS             5000U

/* K230 packets are emitted at 50 Hz.  Values are conservative first-run values. */
#define H26_T3_MIN_CONFIDENCE                50U
#define H26_T3_MAX_FRAME_AGE_MS             100U
#define H26_T3_MIN_SPEED_DT_MS               10U
#define H26_T3_MAX_SPEED_DT_MS              100U
#define H26_T3_NOMINAL_FRAME_MS              20U
#define H26_T3_VISION_SHORT_HOLD_MS          80U
#define H26_T3_VISION_FAULT_MS              250U
#define H26_T3_MAX_BALL_SPEED_CMPS           80.0f
#define H26_T3_MAX_POSITION_JUMP_CM           2.00f
#define H26_T3_SPEED_FILTER_ALPHA          0.55f

/* Stepper velocity command tuning, in STEP pulse frequency (Hz). */
#define H26_T3_POSITION_KP_HZ_PER_CM       180.0f
#define H26_T3_SPEED_KD_HZ_PER_CMPS         90.0f
#define H26_T3_MIN_COMMAND_HZ                60U
#define H26_T3_MAX_COMMAND_HZ               700U
#define H26_T3_COMMAND_SLEW_HZ_PER_TICK      40U
#define H26_T3_STEPPER_SIGN_FOR_POSITIVE_BALL 1
#define H26_T3_DIRECTION_GUARD_MS             2U

/*
 * Raw camera-coordinate end guard and relative rod-encoder protections.
 * RodEncoder is reset at each task-3 start, so this is a travel envelope,
 * not an absolute mechanical home.  Keep the mechanism near its known centre
 * before K2 until a physical home/limit switch is added.
 */
#define H26_T3_RAW_END_GUARD_CENTICM         80U
#define H26_T3_ROD_SOFT_LIMIT_COUNT        3096
#define H26_T3_ROD_STALL_MIN_COMMAND_HZ      80U
#define H26_T3_ROD_STALL_FAULT_MS           400U

#define H26_LED_SHOW_ON_MS                 200U
#define H26_LED_SHOW_OFF_MS                200U
#define H26_LED_FINISHED_TOGGLE_MS         500U
#define H26_LED_FAULT_TOGGLE_MS            100U

#define H26_DEBUG_PERIOD_MS                500U

#endif
