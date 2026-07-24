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

#define CAR_OLED_ENABLE                 ECAR_OLED_ENABLE
#define CAR_BOARD_TEST_MODE             ECAR_BOARD_TEST_MODE
#define CAR_TEST_RADIO_ENABLE           ECAR_TEST_RADIO_ENABLE
#define CAR_ENCODER_MINIMAL_DEBUG       ECAR_ENCODER_MINIMAL_DEBUG
#define CAR_ENCODER_SPEED_PERIOD_MS     ECAR_ENCODER_SPEED_PERIOD_MS
#define CAR_CONTROL_PERIOD_MS           ECAR_CONTROL_PERIOD_MS
#define CAR_SERIAL_PLOT_PERIOD_MS       ECAR_SERIAL_PLOT_PERIOD_MS
#define CAR_OLED_REFRESH_PERIOD_MS      ECAR_OLED_REFRESH_PERIOD_MS
#define CAR_TASK_COUNT_MAX              ECAR_TASK_COUNT_MAX

#ifndef ECAR_TEST_STEPPER_ENABLE
#define ECAR_TEST_STEPPER_ENABLE 1
#endif

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

#ifndef ECAR_ENCODER_MINIMAL_DEBUG
#define ECAR_ENCODER_MINIMAL_DEBUG              0
#endif

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

#ifndef ECAR_OLED_ENABLE
#define ECAR_OLED_ENABLE                        0
#endif

#ifndef ECAR_TASK_COUNT_MAX
#define ECAR_TASK_COUNT_MAX                     5U
#endif
#define ECAR_ENCODER_SPEED_PERIOD_MS            5U
#define ECAR_CONTROL_PERIOD_MS                  10U
#define ECAR_SERIAL_PLOT_PERIOD_MS              100U
#define ECAR_OLED_REFRESH_PERIOD_MS             200U

#define ECAR_PI_F                               3.1415926f
#define ECAR_WHEEL_DIAMETER_CM                 6.5f
#define ECAR_WHEEL_CIRCUMFERENCE_CM            (ECAR_WHEEL_DIAMETER_CM * ECAR_PI_F)

/* Chassis mechanical parameters (mm → cm). */
#define ECAR_TRACK_WIDTH_CM                    15.9f
#define ECAR_LINE_SENSOR_AXLE_OFFSET_CM        17.7f

/*
 * Turn theory: encoder counts for 90° / 180° pivot turn.
 *
 *   arc_length = (track_width / 2) * theta_rad
 *   enc_counts = arc_length / wheel_circumference * ENCODER_COUNT_PER_REV
 *
 * For 90° (pi/2 rad):
 *   enc = (15.9/2) * (pi/2) / (6.5*pi) * 4096
 *       = 15.9 / (4 * 6.5) * 4096 ≈ 2504.86 → 2505
 *
 * For 180° (pi rad):
 *   ≈ 5010
 *
 * These are theoretical initial values; must be calibrated on real track.
 */
#define ECAR_TURN_90_ENCODER_COUNT_THEORY       2505U
#define ECAR_TURN_180_ENCODER_COUNT_THEORY      5010U

#define ECAR_ENCODER_PULSE_PER_REV             367.0f
#define ECAR_CM_PER_PULSE                      (ECAR_WHEEL_CIRCUMFERENCE_CM / ECAR_ENCODER_PULSE_PER_REV)

#define ECAR_LAP_DISTANCE_CM                    400.0f

#define ECAR_CORNER_ADVANCE_CM                  4.0f
#define ECAR_CORNER_MIN_STRAIGHT_CM             80.0f
#define ECAR_CORNER_LOST_CONFIRM_MS             100U
#define ECAR_LINE_LOST_FAULT_MS                 2500U

#define ECAR_CORNER_ADVANCE_PULSE \
    ((int32_t)(ECAR_CORNER_ADVANCE_CM / ECAR_CM_PER_PULSE + 0.5f))
#define ECAR_CORNER_MIN_STRAIGHT_PULSE \
    ((int32_t)(ECAR_CORNER_MIN_STRAIGHT_CM / ECAR_CM_PER_PULSE + 0.5f))

#define ECAR_DEFAULT_LAP_PULSE                  ((int32_t)((ECAR_LAP_DISTANCE_CM / ECAR_CM_PER_PULSE) + 0.5f))

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

#define STEPPER_HAS_ENABLE_GPIO         0U

#define STEPPER_STEP_PER_REV            3200U
#define ENCODER_COUNT_PER_REV           4096U

#define STEPPER_ENC_PER_STEP_NUM        4096U
#define STEPPER_ENC_PER_STEP_DEN        3200U

#ifndef STEPPER_FULL_SPEED_RPM
#define STEPPER_FULL_SPEED_RPM          600U
#endif

#define STEPPER_FULL_STEP_FREQ_HZ \
    ((uint32_t)(STEPPER_FULL_SPEED_RPM) * STEPPER_STEP_PER_REV / 60U)

#ifndef STEPPER_MAX_FREQ_HZ
#define STEPPER_MAX_FREQ_HZ STEPPER_FULL_STEP_FREQ_HZ
#endif

#ifndef STEPPER_ACCEL_HZ_PER_TICK
#define STEPPER_ACCEL_HZ_PER_TICK 20
#endif
#ifndef STEPPER_DECEL_HZ_PER_TICK
#define STEPPER_DECEL_HZ_PER_TICK 30
#endif

#ifndef STEPPER_MIN_TIMER_FREQ_HZ
#define STEPPER_MIN_TIMER_FREQ_HZ 500
#endif

#define STEPPER_TEST_MAX_RPM            ((STEPPER_FULL_SPEED_RPM) / 2U)
#define STEPPER_TEST_MAX_FREQ_HZ        ((STEPPER_FULL_STEP_FREQ_HZ) / 2U)

#define STEPPER_SPEED_LEVELS 7U

#if ECAR_MOTOR_TYPE_STEPPER
/*
 * Override encoder pulse-per-rev for 4x stepper encoder (4096 counts/rev).
 * This replaces the legacy 1x DC motor value (367 pulses/rev).
 */
#undef  ECAR_ENCODER_PULSE_PER_REV
#define ECAR_ENCODER_PULSE_PER_REV \
    ((float)(ENCODER_COUNT_PER_REV))

#undef  ECAR_CM_PER_PULSE
#define ECAR_CM_PER_PULSE \
    (ECAR_WHEEL_CIRCUMFERENCE_CM / ECAR_ENCODER_PULSE_PER_REV)
#endif

#endif
