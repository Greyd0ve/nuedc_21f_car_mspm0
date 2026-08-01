#ifndef __APP_H26_CONFIG_H
#define __APP_H26_CONFIG_H

#include "app_config.h"

/* H26 task-2 control and confirmation parameters.  Calibrate on the real track. */
#define H26_T2_STRAIGHT_SPEED_CMPS          45.0f
/* Reduced speed gives the front-mounted grayscale sensor more correction time. */
#define H26_T2_CURVE_SPEED_CMPS             40.0f
#define H26_T2_FINISH_SPEED_CMPS            25.0f
#define H26_T2_STRAIGHT_TURN_LIMIT_CMPS     15.0f
#define H26_T2_CURVE_TURN_LIMIT_CMPS        35.0f

/* Turn-command hysteresis for straight/curve speed-zone selection. */
#define H26_T2_CURVE_ENTER_TURN_CMPS         2.5f
#define H26_T2_CURVE_EXIT_TURN_CMPS          1.2f
#define H26_T2_CURVE_ENTER_HOLD_MS          50U
#define H26_T2_CURVE_EXIT_HOLD_MS          100U

/* Each straight/curve segment must cover this distance before its transition. */
#define H26_T2_CURVE_ENTER_STRAIGHT_CM     100.0f
#define H26_T2_CURVE_EXIT_LEFT_CM          160.0f
#define H26_T2_CURVE_ENTER_STRAIGHT_PULSE \
    ((int32_t)((H26_T2_CURVE_ENTER_STRAIGHT_CM / ECAR_CM_PER_PULSE) + 0.5f))
#define H26_T2_CURVE_EXIT_LEFT_PULSE \
    ((int32_t)((H26_T2_CURVE_EXIT_LEFT_CM / ECAR_CM_PER_PULSE) + 0.5f))

/* Zero disables task-2 speed ramping: command speed follows its target directly. */
#define H26_T2_SPEED_SLEW_CMPS_PER_TICK      0.0f
/* After the A-point marker is confirmed, continue normal line following
 * briefly, then force PWM to zero without an unverified reverse brake. */
#define H26_T2_FINISH_EXIT_STOP_DELAY_MS     500U

#define H26_T2_LEAVE_DISTANCE_CM            15.0f
#define H26_T2_LEAVE_LINE_STABLE_MS         80U
#define H26_T2_LEAVE_CLEAR_BLACK_CHANNELS     4U

/* A-point marker qualification: detect a wide black line after one lap. */
#define H26_T2_MIN_FINISH_DISTANCE_CM      450.0f
#define H26_T2_MIN_FINISH_TIME_MS        13000U
#define H26_T2_MAX_RUN_TIME_MS           30000U

#define H26_T2_FINISH_BLACK_CHANNELS        6U
#define H26_T2_FINISH_HOLD_MS               10U

#define H26_T2_STOP_SPEED_CMPS               1.0f
#define H26_T2_STOP_HOLD_MS                 100U
/* A short all-white sample keeps following the last valid line error. */
#define H26_T2_LINE_LOST_TURN_MS             20U
#define H26_T2_LINE_LOST_TURN_CMPS           -30.0f
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
#define H26_T3_TILT_DEADBAND_CM              0.15f
#define H26_T3_TILT_DEADBAND_SPEED_CMPS      0.30f

/* K230 parameters are used by the O-point controllers in tasks 4 and 5. */
#define H26_T3_MIN_CONFIDENCE                50U
#define H26_T3_NOMINAL_FRAME_MS              20U
#define H26_T3_SPEED_FILTER_ALPHA          0.35f
#define H26_T4_BALL_POSITION_DEADBAND_CM    0.25f

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
 * Task 3: fixed open-loop chain sequence.
 *
 * One chain millimetre is 3200 / 31.25 = 102.4 STEP pulses.  The pulse
 * With the current verified mechanical direction mapping, the physical
 * sequence is extend 10 mm, retract 19 mm, extend 16 mm, then retract 7 mm.
 * Its absolute positions are +10, -9, +7, then 0 mm, so the guide returns to
 * its initial horizontal position after the final +7 mm compensation.
 *
 * If the first movement physically retracts rather than extends the chain,
 * change H26_T3_CHAIN_EXTEND_DIR_POSITIVE from 1U to 0U; no source change is
 * required.
 */
#define H26_T3_CHAIN_EXTEND_DIR_POSITIVE        1U
#define H26_T3_EXTEND_10MM_STEP_HZ            2000U
#define H26_T3_EXTEND_10MM_PULSES             1024U
#define H26_T3_RETRACT_10MM_STEP_HZ           2000U
#define H26_T3_RETRACT_10MM_PULSES             512U
#define H26_T3_HOLD_TO_STAGE1_MS                 0U
#define H26_T3_HOLD_TO_RETRACT_MS                0U
#define H26_T3_HOLD_TO_STAGE2_MS                 0U

/* Stage 1: PID to O-point -5 mm, 5 mm deadband, hold 50 ms. */
#define H26_T3_PID_STAGE1_TARGET_CM           -5.00f
#define H26_T3_PID_STAGE1_TOLERANCE_CM         0.80f
#define H26_T3_PID_STAGE1_STABLE_MS              50U

/* Stage 2: PID to +5 cm, 4 mm deadband, hold 200 ms then done. */
#define H26_T3_PID_STAGE2_TARGET_CM            5.00f
#define H26_T3_PID_STAGE2_TOLERANCE_CM         0.60f
#define H26_T3_PID_STAGE2_STABLE_MS             300U

#define H26_T3_FINAL_PID_KP_MM_PER_CM           1.30f
#define H26_T3_FINAL_PID_KI_MM_PER_CM_S         0.10f
#define H26_T3_FINAL_PID_KD_MM_PER_CMPS         0.75f
#define H26_T3_FINAL_PID_INTEGRAL_LIMIT_CM_S   30.00f
#define H26_T3_FINAL_PID_TILT_LIMIT_MM         10.00f

/* Default O-point hold PD used by task 4/task 5. */
#define H26_T3_BALL_POSITION_TO_TILT_MM_PER_CM    0.45f
#define H26_T3_BALL_SPEED_TO_TILT_MM_PER_CMPS     0.10f
#define H26_T3_TILT_COMMAND_LIMIT_MM              9.00f

/* Inner rod-position loop: encoder count error -> STEP frequency. */
#define H26_T3_ROD_POSITION_DEADBAND_COUNT           3
#define H26_T3_ROD_POSITION_KP_HZ_PER_COUNT        8.0f
#define H26_T3_ROD_POSITION_MIN_HZ                  60U
#define H26_T3_ROD_POSITION_MAX_HZ                5000U

/*
 * H26 task-4 supports two test entries after task 4 is selected:
 * K2 runs the 130 cm straight-line test with ball PID and acceleration
 * feed-forward; K3 leaves traction off for the hand-push feed-forward test.
 * Units: Kp = mm/cm, Ki = mm/(cm*s), Kd = mm/(cm/s).
 */
#define H26_T4_O_TARGET_CM                        0.0f
#define H26_T4_BALL_KP_MM_PER_CM                  0.60f
#define H26_T4_BALL_KI_MM_PER_CM_S                0.10f
#define H26_T4_BALL_KD_MM_PER_CMPS                0.10f
#define H26_T4_BALL_INTEGRAL_LIMIT_CM_S          30.00f
#define H26_T4_BALL_TILT_COMMAND_LIMIT_MM        20.00f

/* K2 combined-test chassis command.  No line-loss stop fault is used. */
#define H26_T4_DRIVE_STRAIGHT_SPEED_CMPS          25.00f
#define H26_T4_DRIVE_CURVE_SPEED_CMPS             20.00f
#define H26_T4_STRAIGHT_TURN_LIMIT_CMPS            8.00f
#define H26_T4_CURVE_TURN_LIMIT_CMPS              25.00f
#define H26_T4_STRAIGHT_DISTANCE_CM              120.00f
#define H26_T4_CURVE_ENTER_TURN_CMPS               2.50f
#define H26_T4_CURVE_EXIT_TURN_CMPS                1.20f
#define H26_T4_DRIVE_TIMEOUT_MS                  10000U
/* Keep the vehicle still after the first valid K230 frame defines O. */
#define H26_T4_O_LOCK_HOLD_MS                      100U

/*
 * Encoder acceleration feed-forward.  The + sign is verified from task 3:
 * a positive rod command tilts the pipe toward the vehicle forward direction.
 * The same estimator is used by both the K2 vehicle test and the K3
 * hand-push test.
 */
#define H26_T4_ENCODER_FF_ENABLE                    1U
#define H26_T4_FF_TILT_SIGN_FOR_FORWARD_ACCEL     -1.0f
#define H26_T4_FF_ACCEL_FILTER_ALPHA              0.25f
#define H26_T4_FF_ACCEL_LIMIT_CMPS2               25.0f
#define H26_T4_FF_K_MM_PER_CMPS2                   0.60f
#define H26_T4_FF_TILT_LIMIT_MM                    7.00f
#define H26_T4_FF_MAX_WHEEL_SPEED_DIFF_CMPS        8.00f

/*
 * Task 5 holds the ball at O and shares task 2's line-following calibration.
 * Its chassis settings share task 2's line-following calibration, while its
 * ball PID and A-point finish settings stay independent.
 */
#define H26_T5_O_TARGET_CM                     0.0f
/*
 * Signed ball-position bias used only while task 5 is driving.  The PID
 * target is H26_T5_O_TARGET_CM + this value; use 0.0f to disable it.
 * Example: with the camera's physical O at 12 cm and a desired PID centre
 * of 13 cm, set this to +1.0f (after O-coordinate calibration).
 */
#define H26_T5_DRIVE_TARGET_COMPENSATION_CM    0.0f
/*
 * Task 5 uses separate ball-position PID gains for straight and curve
 * tracking.  Keep the initial values identical to preserve the validated
 * all-course behaviour; tune each group independently on the real track.
 */
#define H26_T5_BALL_STRAIGHT_KP_MM_PER_CM      1.00f
#define H26_T5_BALL_STRAIGHT_KI_MM_PER_CM_S    0.09f
#define H26_T5_BALL_STRAIGHT_KD_MM_PER_CMPS    0.95f
#define H26_T5_BALL_CURVE_KP_MM_PER_CM         1.00f
#define H26_T5_BALL_CURVE_KI_MM_PER_CM_S       0.09f
#define H26_T5_BALL_CURVE_KD_MM_PER_CMPS       1.05f
#define H26_T5_BALL_POSITION_DEADBAND_CM       0.15f
#define H26_T5_BALL_INTEGRAL_LIMIT_CM_S       35.00f
#define H26_T5_BALL_TILT_COMMAND_LIMIT_MM H26_T4_BALL_TILT_COMMAND_LIMIT_MM

#define H26_T5_STRAIGHT_SPEED_CMPS             29.0f
#define H26_T5_CURVE_SPEED_CMPS                29.0f
#define H26_T5_STRAIGHT_TURN_LIMIT_CMPS H26_T2_STRAIGHT_TURN_LIMIT_CMPS
#define H26_T5_CURVE_TURN_LIMIT_CMPS    H26_T2_CURVE_TURN_LIMIT_CMPS
#define H26_T5_CURVE_ENTER_TURN_CMPS    H26_T2_CURVE_ENTER_TURN_CMPS
#define H26_T5_CURVE_EXIT_TURN_CMPS     H26_T2_CURVE_EXIT_TURN_CMPS
#define H26_T5_CURVE_ENTER_HOLD_MS      H26_T2_CURVE_ENTER_HOLD_MS
#define H26_T5_CURVE_EXIT_HOLD_MS       H26_T2_CURVE_EXIT_HOLD_MS
#define H26_T5_CURVE_ENTER_STRAIGHT_CM  H26_T2_CURVE_ENTER_STRAIGHT_CM
#define H26_T5_CURVE_EXIT_LEFT_CM       H26_T2_CURVE_EXIT_LEFT_CM
#define H26_T5_CURVE_ENTER_STRAIGHT_PULSE H26_T2_CURVE_ENTER_STRAIGHT_PULSE
#define H26_T5_CURVE_EXIT_LEFT_PULSE      H26_T2_CURVE_EXIT_LEFT_PULSE
#define H26_T5_A_DETECT_ENABLE_MS             23000U
#define H26_T5_A_DETECT_BLACK_CHANNELS            5U
#define H26_T5_A_DETECT_HOLD_MS                  10U
/* Task 5 retains its independent per-10-ms command ramp. */
#define H26_T5_SPEED_SLEW_CMPS_PER_TICK      0.8f
#define H26_T5_LEAVE_DISTANCE_CM        H26_T2_LEAVE_DISTANCE_CM
#define H26_T5_LEAVE_LINE_STABLE_MS     H26_T2_LEAVE_LINE_STABLE_MS
#define H26_T5_LEAVE_CLEAR_BLACK_CHANNELS H26_T2_LEAVE_CLEAR_BLACK_CHANNELS
#define H26_T5_MAX_RUN_TIME_MS                 35000U
#define H26_T5_LINE_LOST_STOP_MS        H26_T2_LINE_LOST_STOP_MS
#define H26_T5_LINE_LOST_TURN_MS        H26_T2_LINE_LOST_TURN_MS
#define H26_T5_LINE_LOST_TURN_CMPS      H26_T2_LINE_LOST_TURN_CMPS
#define H26_T5_FINISH_EXIT_STOP_DELAY_MS       2000U
#define H26_T5_STOP_SPEED_CMPS          H26_T2_STOP_SPEED_CMPS
#define H26_T5_STOP_HOLD_MS             H26_T2_STOP_HOLD_MS
#define H26_T5_O_TOLERANCE_CM                    0.60f
#define H26_T5_O_ACQUIRE_HOLD_MS                100U
/* 0 -> 29 cm/s in 3 s: constant 9.67 cm/s^2 acceleration. */
#define H26_T5_STRAIGHT_ACCEL_RAMP_MS          3000U
/* Start rod motion before traction, then hand the planned term to encoder FF. */
#define H26_T5_PRETILT_MS                        130U
#define H26_T5_FF_HANDOFF_MS                     140U
#define H26_T5_LAUNCH_ACCEL_CMPS2                 9.67f
/* Hold the existing I term during the commanded acceleration interval. */
#define H26_T5_INTEGRAL_FREEZE_MS H26_T5_STRAIGHT_ACCEL_RAMP_MS

#define H26_LED_SHOW_ON_MS                 200U
#define H26_LED_SHOW_OFF_MS                200U
#define H26_LED_FINISHED_TOGGLE_MS         500U
#define H26_LED_FAULT_TOGGLE_MS            100U

#define H26_DEBUG_PERIOD_MS                500U
/* The compact telemetry below retains all control and timing signals needed
 * for field tuning.  The old exhaustive UART/parser counters require more
 * than the remaining MDK Lite flash margin, so enable them only in a
 * separately size-checked diagnostic build. */
#ifndef H26_VERBOSE_TELEMETRY_ENABLE
#define H26_VERBOSE_TELEMETRY_ENABLE         0U
#endif

#endif
