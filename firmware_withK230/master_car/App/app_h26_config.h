#ifndef __APP_H26_CONFIG_H
#define __APP_H26_CONFIG_H

#include "app_config.h"

/* H26 task-2 control and confirmation parameters.  Calibrate on the real track. */
#define H26_T2_STRAIGHT_SPEED_CMPS          42.0f
/* Reduced speed gives the front-mounted grayscale sensor more correction time. */
#define H26_T2_CURVE_SPEED_CMPS             28.0f
#define H26_T2_FINISH_SPEED_CMPS            15.0f
#define H26_T2_STRAIGHT_TURN_LIMIT_CMPS      8.0f
#define H26_T2_CURVE_TURN_LIMIT_CMPS        25.0f

/* Turn-command hysteresis for straight/curve speed-zone selection. */
#define H26_T2_CURVE_ENTER_TURN_CMPS         2.5f
#define H26_T2_CURVE_EXIT_TURN_CMPS          1.2f
#define H26_T2_CURVE_ENTER_HOLD_MS          50U
#define H26_T2_CURVE_EXIT_HOLD_MS          100U

/* Each straight/curve segment must cover this distance before its transition. */
#define H26_T2_CURVE_ENTER_STRAIGHT_CM     100.0f
#define H26_T2_CURVE_EXIT_LEFT_CM          100.0f
#define H26_T2_CURVE_ENTER_STRAIGHT_PULSE \
    ((int32_t)((H26_T2_CURVE_ENTER_STRAIGHT_CM / ECAR_CM_PER_PULSE) + 0.5f))
#define H26_T2_CURVE_EXIT_LEFT_PULSE \
    ((int32_t)((H26_T2_CURVE_EXIT_LEFT_CM / ECAR_CM_PER_PULSE) + 0.5f))

/* A is reached after the second stable curve-to-straight transition. */
#define H26_T2_A_DETECT_CURVE_EXIT_COUNT      2U

/* Per-10-ms command ramp; avoids an abrupt speed step at zone boundaries. */
#define H26_T2_SPEED_SLEW_CMPS_PER_TICK      0.8f
/* Applied only after A is latched; 42 cm/s reaches zero in about 1.05 s. */
#define H26_T2_FINISH_BRAKE_SLEW_CMPS_PER_TICK 0.4f

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
/* A short all-white sample keeps following the last valid line error. */
#define H26_T2_LINE_LOST_STOP_MS             300U

/*
 * H26 task-3 (stationary vehicle ball motion) parameters.
 *
 * The K230 sends the ball's longitudinal location in 0.01 cm units over
 * UART1.  Task 3 never uses that 0..25 cm camera coordinate directly: it is
 * converted to the signed competition coordinate where O = 0.00 cm, the
 * positive target is +5.00 cm, and the negative target is -5.00 cm.
 *
 * Default calibration is performed inside the MCU: when K2 starts Task 3,
 * the first valid K230 position is latched as physical O.  Subsequent raw
 * positions are converted with the K230's 0.01 cm unit, so O = 0.00 cm and
 * raw changes of +/-500 correspond to physical +/-5.00 cm.  The K230 keeps
 * sending its original 0..25 cm coordinate and needs no protocol change.
 *
 * Set H26_T3_AUTO_ZERO_AT_TASK_START to 0 only when a measured three-point
 * camera calibration is required.  The RAW_O/RAW_POS5/RAW_NEG5 values below
 * are then used as the independent manual calibration points.
 */
#define H26_T3_PIPE_LENGTH_CENTICM          2500U
#define H26_T3_RAW_MIN_CENTICM                  0U
#define H26_T3_RAW_MAX_CENTICM               2500U
#define H26_T3_AUTO_ZERO_AT_TASK_START          1U
#define H26_T3_CAMERA_CENTICM_PER_CM          100U
/* +1: K230 raw coordinate increases toward physical + direction; -1 reverses it. */
#define H26_T3_CAMERA_SIGN_FOR_POSITIVE_BALL    1

/* Manual three-point fallback calibration; used only when AUTO_ZERO is 0. */
#define H26_T3_RAW_O_CENTICM                 1250U
#define H26_T3_RAW_POS5_CENTICM              1750U
#define H26_T3_RAW_NEG5_CENTICM               750U
#define H26_T3_TARGET_POSITIVE_CM              5.00f
#define H26_T3_TARGET_NEGATIVE_CM             -5.00f
#define H26_T3_START_O_TOLERANCE_CM            0.60f
#define H26_T3_TARGET_TOLERANCE_CM             0.60f
#define H26_T3_TILT_DEADBAND_CM              0.12f
#define H26_T3_TILT_DEADBAND_SPEED_CMPS      0.30f
#define H26_T3_STABLE_SPEED_CMPS             0.80f

#define H26_T3_ACQUIRE_HOLD_MS              100U
#define H26_T3_TARGET_HOLD_MS               150U

/* K230 packets are emitted at 50 Hz. */
#define H26_T3_MIN_CONFIDENCE                50U
#define H26_T3_NOMINAL_FRAME_MS              20U
#define H26_T3_SPEED_FILTER_ALPHA          0.65f

/*
 * Chain/guide calibration measured on the vehicle:
 *   3200 STEP pulses = 1 motor revolution = 31.25 mm chain travel;
 *   4096 rod-encoder counts = 1 motor revolution.
 * The encoder and the ball-coordinate directions are intentionally kept
 * separate.  The verified encoder decreases for ROD_STEPPER_DIR_POSITIVE.
 * After the rack was mounted in the reverse orientation, a positive K230
 * ball-coordinate command requires ROD_STEPPER_DIR_NEGATIVE.  Therefore a
 * positive ball command produces a positive encoder target count.
 */
#define H26_T3_ROD_TRAVEL_MM_PER_REV       31.25f
#define H26_T3_ROD_ENCODER_COUNTS_PER_REV 4096.0f
#define H26_T3_ROD_ENCODER_COUNTS_PER_MM \
    (H26_T3_ROD_ENCODER_COUNTS_PER_REV / H26_T3_ROD_TRAVEL_MM_PER_REV)
#define H26_T3_ROD_ENCODER_SIGN_FOR_STEPPER_POSITIVE (-1)
#define H26_T3_STEPPER_SIGN_FOR_POSITIVE_BALL       (-1)
#define H26_T3_ROD_ENCODER_SIGN_FOR_POSITIVE_BALL \
    (H26_T3_ROD_ENCODER_SIGN_FOR_STEPPER_POSITIVE * \
     H26_T3_STEPPER_SIGN_FOR_POSITIVE_BALL)

/*
 * Outer ball PD: position/speed error -> requested height change at the
 * guide's 0 cm end.  This is intentionally a small relative height command,
 * not a direct stepper-frequency command.  A +5 cm ball error now requests
 * about +2.25 mm at the guide end; the wider +/-3.00 mm range gives enough
 * slope to overcome the measured mechanical rolling resistance.
 */
#define H26_T3_BALL_POSITION_TO_TILT_MM_PER_CM    0.45f
#define H26_T3_BALL_SPEED_TO_TILT_MM_PER_CMPS     0.10f
#define H26_T3_TILT_COMMAND_LIMIT_MM              3.00f

/* Inner rod-position loop: encoder count error -> STEP frequency. */
#define H26_T3_ROD_POSITION_DEADBAND_COUNT           3
#define H26_T3_ROD_POSITION_KP_HZ_PER_COUNT        5.0f
#define H26_T3_ROD_POSITION_MIN_HZ                  60U
#define H26_T3_ROD_POSITION_MAX_HZ                 700U

/*
 * H26 task-4: A -> B is the first 150 cm straight section.  The existing
 * ECAR_CM_PER_PULSE wheel conversion is deliberately reused for B detection;
 * do not create a second encoder scale here.  25 cm/s gives a nominal 6 s
 * AB traverse, leaving margin below the 8 s requirement while reducing the
 * acceleration disturbance presented to the ball controller.
 */
#define H26_T4_B_DISTANCE_CM                   150.0f
#define H26_T4_FORWARD_SPEED_CMPS               25.0f
#define H26_T4_SPEED_SLEW_CMPS_PER_TICK          0.5f
#define H26_T4_TURN_LIMIT_CMPS                   5.0f
#define H26_T4_O_TOLERANCE_CM                    0.60f
#define H26_T4_O_ACQUIRE_HOLD_MS                100U

/* Task 5 holds the ball at O while its chassis uses task 2 unchanged. */
#define H26_T5_O_TOLERANCE_CM                    0.60f
#define H26_T5_O_ACQUIRE_HOLD_MS                100U
/* From the first chassis line-control tick, rise from 0 to task-2 straight speed. */
#define H26_T5_STRAIGHT_ACCEL_RAMP_MS          2000U

#define H26_LED_SHOW_ON_MS                 200U
#define H26_LED_SHOW_OFF_MS                200U
#define H26_LED_FINISHED_TOGGLE_MS         500U
#define H26_LED_FAULT_TOGGLE_MS            100U

#define H26_DEBUG_PERIOD_MS                500U

#endif
