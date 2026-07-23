#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include <stdint.h>

/*
 * Select motor type for slave car.
 * 0 = DC motor + TB6612 (legacy)
 * 1 = Stepper motor + D36A dual-channel driver
 */
#ifndef ECAR_MOTOR_TYPE_STEPPER
#define ECAR_MOTOR_TYPE_STEPPER 1
#endif

/* Neutral CAR_ aliases for template layer use.
 * ECAR_ macros are kept for backward compatibility with existing drivers. */
#define CAR_OLED_ENABLE                 ECAR_OLED_ENABLE
#define CAR_BOARD_TEST_MODE             ECAR_BOARD_TEST_MODE
#define CAR_TEST_RADIO_ENABLE           ECAR_TEST_RADIO_ENABLE
#define CAR_ENCODER_MINIMAL_DEBUG       ECAR_ENCODER_MINIMAL_DEBUG
#define CAR_ENCODER_SPEED_PERIOD_MS     ECAR_ENCODER_SPEED_PERIOD_MS
#define CAR_CONTROL_PERIOD_MS           ECAR_CONTROL_PERIOD_MS
#define CAR_SERIAL_PLOT_PERIOD_MS       ECAR_SERIAL_PLOT_PERIOD_MS
#define CAR_OLED_REFRESH_PERIOD_MS      ECAR_OLED_REFRESH_PERIOD_MS
#define CAR_TASK_COUNT_MAX              ECAR_TASK_COUNT_MAX

/* Board-test stepper mode enable. Only takes effect when board-test is on. */
#ifndef ECAR_TEST_STEPPER_ENABLE
#define ECAR_TEST_STEPPER_ENABLE 1
#endif

/* CarBase template behaviour switches. */
#ifndef CAR_BASE_SERIAL_MONITOR_ENABLE
#define CAR_BASE_SERIAL_MONITOR_ENABLE          1
#endif
#ifndef CAR_BASE_BOOT_PROMPT_ENABLE
#define CAR_BASE_BOOT_PROMPT_ENABLE             0
#endif

#ifndef CAR_ROLE_MASTER
#define CAR_ROLE_MASTER                         0
#endif
#ifndef CAR_ROLE_SLAVE
#define CAR_ROLE_SLAVE                          1
#endif
#ifndef CAR_ID
#define CAR_ID                                  2
#endif
#ifndef ENABLE_K230
#define ENABLE_K230                             1
#endif

#ifndef RADIO_DEBUG_ENABLE
#define RADIO_DEBUG_ENABLE                      1
#endif

/* Temporary encoder-only memory corruption diagnostic mode. */
#ifndef ECAR_ENCODER_MINIMAL_DEBUG
#define ECAR_ENCODER_MINIMAL_DEBUG              0
#endif

/* Safety switches. Keep remote start disabled unless deliberately enabled.
 * For IMU-only board tests, set ECAR_TEST_MOTOR_ENABLE = 0 to prevent
 * unexpected motor output during gyro calibration and yaw testing. */
#ifndef ECAR_ENABLE_REMOTE_START
#define ECAR_ENABLE_REMOTE_START                0
#endif
#ifndef ECAR_BOARD_TEST_MODE
#define ECAR_BOARD_TEST_MODE                    0
#endif
#ifndef ECAR_TEST_MOTOR_ENABLE
#define ECAR_TEST_MOTOR_ENABLE                  0
#endif
#ifndef ECAR_TEST_SERVO_ENABLE
#define ECAR_TEST_SERVO_ENABLE                  0
#endif
#ifndef ECAR_TEST_BEEP_ENABLE
#define ECAR_TEST_BEEP_ENABLE                   0
#endif
#ifndef ECAR_TEST_OLED_ENABLE
#define ECAR_TEST_OLED_ENABLE                   0
#endif
#ifndef ECAR_TEST_RADIO_ENABLE
#define ECAR_TEST_RADIO_ENABLE                  0
#endif

/* Master OLED switch. Set to 1 only when the display is physically connected.
 * When 0, all OLED_Init / OLED_Clear / status display calls are compiled out. */
#ifndef ECAR_OLED_ENABLE
#define ECAR_OLED_ENABLE                        0
#endif

/* Cooperative task periods driven by the 1ms timer ISR. */
#ifndef ECAR_TASK_COUNT_MAX
#define ECAR_TASK_COUNT_MAX                     5U
#endif
#define ECAR_ENCODER_SPEED_PERIOD_MS            5U
#define ECAR_CONTROL_PERIOD_MS                  10U
#define ECAR_SERIAL_PLOT_PERIOD_MS              100U
#define ECAR_OLED_REFRESH_PERIOD_MS             200U

/*
 * Nominal JGA25-370B + 65mm wheel conversion constants.
 * ECAR_ENCODER_PULSE_PER_REV is measured under Encoder.c default
 * A rising-edge counting mode. See app_config.h encoder comment.
 */
#define ECAR_PI_F                               3.1415926f
#define ECAR_WHEEL_DIAMETER_CM                 6.5f
#define ECAR_WHEEL_CIRCUMFERENCE_CM            (ECAR_WHEEL_DIAMETER_CM * ECAR_PI_F)

/*
 * Encoder counting mode:
 * - Current Encoder.c default counts only encoder A rising edges.
 * - Encoder B is sampled only for direction judgement.
 * - ECAR_ENCODER_PULSE_PER_REV is the measured wheel-end pulse count under
 *   this exact counting mode.
 *
 * If Encoder.c is changed to A both-edge counting or AB quadrature counting,
 * this value must be measured again.
 */
#define ECAR_ENCODER_PULSE_PER_REV             367.0f
#define ECAR_CM_PER_PULSE                      (ECAR_WHEEL_CIRCUMFERENCE_CM / ECAR_ENCODER_PULSE_PER_REV)

/* [legacy] E-topic square track nominal distance. */
#define ECAR_LAP_DISTANCE_CM                    400.0f

/*
 * [legacy] E-topic corner detection:
 * all-white (8-channel count == 0) for LOST_CONFIRM_MS triggers corner.
 */
#define ECAR_CORNER_ADVANCE_CM                  4.0f
#define ECAR_CORNER_MIN_STRAIGHT_CM             80.0f
#define ECAR_CORNER_LOST_CONFIRM_MS             100U
#define ECAR_LINE_LOST_FAULT_MS                 2500U

#define ECAR_CORNER_ADVANCE_PULSE \
    ((int32_t)(ECAR_CORNER_ADVANCE_CM / ECAR_CM_PER_PULSE + 0.5f))
#define ECAR_CORNER_MIN_STRAIGHT_PULSE \
    ((int32_t)(ECAR_CORNER_MIN_STRAIGHT_CM / ECAR_CM_PER_PULSE + 0.5f))

#define ECAR_DEFAULT_LAP_PULSE                  ((int32_t)((ECAR_LAP_DISTANCE_CM / ECAR_CM_PER_PULSE) + 0.5f))

/* Conservative first-run speed defaults, in cm/s. */
#define ECAR_DEFAULT_BASE_SPEED_CMPS            55.0f
#define ECAR_DEFAULT_RECOVER_SPEED_CMPS         10.0f
#define ECAR_DEFAULT_CORNER_FORWARD_CMPS        10.0f
#define ECAR_DEFAULT_CORNER_TURN_CMPS           12.0f
#define ECAR_DEFAULT_TURN_LIMIT_CMPS            8.0f

#ifndef ECAR_CORNER_CONFIRM_COUNT
#define ECAR_CORNER_CONFIRM_COUNT               2U
#endif
#ifndef ECAR_BOARD_TEST_PWM_LIMIT
#define ECAR_BOARD_TEST_PWM_LIMIT               260
#endif

/* =================== Stepper motor / D36A configuration ================= */

/* D36A micro-step: 3200 STEP pulses = 1 motor revolution. */
#define STEPPER_STEP_PER_REV            3200U

/* Encoder counts per wheel revolution.
 * Measured with 4× quadrature decoding on the stepper encoder. */
#define ENCODER_COUNT_PER_REV           4096U

/*
 * Ratio for health checks: encoder_counts / step_pulses.
 * 4096 / 3200 = 1.28 encoder counts per STEP pulse.
 */
#define STEPPER_ENC_PER_STEP_NUM        4096U
#define STEPPER_ENC_PER_STEP_DEN        3200U

/* Software full-speed RPM. */
#ifndef STEPPER_FULL_SPEED_RPM
#define STEPPER_FULL_SPEED_RPM          600U
#endif

/* Full-speed STEP frequency = RPM * STEP_PER_REV / 60. */
#define STEPPER_FULL_STEP_FREQ_HZ \
    ((uint32_t)(STEPPER_FULL_SPEED_RPM) * STEPPER_STEP_PER_REV / 60U)
/* 600 * 3200 / 60 = 32000 Hz */

/* Software max STEP frequency (Hz). Same as full-speed by default. */
#ifndef STEPPER_MAX_FREQ_HZ
#define STEPPER_MAX_FREQ_HZ STEPPER_FULL_STEP_FREQ_HZ
#endif

/* Accel / decel rate per 1ms tick for stepper ramp.
 * Smaller = softer ramp, larger = stiffer response. */
#ifndef STEPPER_ACCEL_HZ_PER_TICK
#define STEPPER_ACCEL_HZ_PER_TICK 20
#endif
#ifndef STEPPER_DECEL_HZ_PER_TICK
#define STEPPER_DECEL_HZ_PER_TICK 30
#endif

/* Minimum STEP frequency the 16-bit timer can represent at BUSCLK/1.
 * Below ~488 Hz the period overflows 65535; clamp to this floor. */
#ifndef STEPPER_MIN_TIMER_FREQ_HZ
#define STEPPER_MIN_TIMER_FREQ_HZ 500
#endif

/*
 * Board test runs at 50 % of full speed max.
 * 300 RPM → 16000 Hz.
 */
#define STEPPER_TEST_MAX_RPM            ((STEPPER_FULL_SPEED_RPM) / 2U)
#define STEPPER_TEST_MAX_FREQ_HZ        ((STEPPER_FULL_STEP_FREQ_HZ) / 2U)

/* Seven-level frequency table (linear from 7 % to 50 % of full speed).
 * Level:  1      2      3      4      5      6      7
 * %:       7.00  14.17  21.33  28.50  35.67  42.83  50.00
 * RPM:     42     85    128    171    214    257    300
 * Hz:    2240   4533   6827   9120  11413  13707  16000
 */
#define STEPPER_SPEED_LEVELS 7U

/*
 * Encoder cm-per-pulse with 4× quadrature. Override the legacy 1× value.
 */
#if ECAR_MOTOR_TYPE_STEPPER
#undef  ECAR_ENCODER_PULSE_PER_REV
#define ECAR_ENCODER_PULSE_PER_REV \
    ((float)(ENCODER_COUNT_PER_REV))  /* 4096.0f */
#endif

/* Legacy re-compute after redefining ECAR_ENCODER_PULSE_PER_REV. */
#undef  ECAR_CM_PER_PULSE
#define ECAR_CM_PER_PULSE \
    (ECAR_WHEEL_CIRCUMFERENCE_CM / ECAR_ENCODER_PULSE_PER_REV)

#endif
