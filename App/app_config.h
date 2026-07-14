#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include <stdint.h>

/* Temporary encoder-only memory corruption diagnostic mode. */
#ifndef ECAR_ENCODER_MINIMAL_DEBUG
#define ECAR_ENCODER_MINIMAL_DEBUG              0
#endif

/* Safety switches. Keep remote start disabled unless deliberately enabled. */
#ifndef ECAR_ENABLE_REMOTE_START
#define ECAR_ENABLE_REMOTE_START                0
#endif
#ifndef ECAR_BOARD_TEST_MODE
#define ECAR_BOARD_TEST_MODE                    1
#endif
#ifndef ECAR_TEST_MOTOR_ENABLE
#define ECAR_TEST_MOTOR_ENABLE                  1
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
#ifndef ECAR_TEST_IMU_ENABLE
#define ECAR_TEST_IMU_ENABLE                    0
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
 * ECAR_ENCODER_EDGE_MULTIPLIER must match the configured encoder edge mode:
 * 1.0=A single edge, 2.0=A both edges, 4.0=AB quadrature.
 */
#define ECAR_PI_F                               3.1415926f
#define ECAR_WHEEL_DIAMETER_CM                 6.5f
#define ECAR_WHEEL_CIRCUMFERENCE_CM            (ECAR_WHEEL_DIAMETER_CM * ECAR_PI_F)

/* New motor measured value: 367 pulses per wheel revolution. */
#define ECAR_ENCODER_PULSE_PER_REV             367.0f
#define ECAR_CM_PER_PULSE                      (ECAR_WHEEL_CIRCUMFERENCE_CM / ECAR_ENCODER_PULSE_PER_REV)

/* Square track nominal distances. Tune with telemetry after hardware tests. */
#define ECAR_LAP_DISTANCE_CM                    400.0f
#define ECAR_MIN_CORNER_INTERVAL_CM             55.0f
#define ECAR_DEFAULT_LAP_PULSE                  ((int32_t)((ECAR_LAP_DISTANCE_CM / ECAR_CM_PER_PULSE) + 0.5f))
#define ECAR_DEFAULT_MIN_CORNER_INTERVAL_PULSE  ((int32_t)((ECAR_MIN_CORNER_INTERVAL_CM / ECAR_CM_PER_PULSE) + 0.5f))

/* Conservative first-run speed defaults, in cm/s. */
#define ECAR_DEFAULT_BASE_SPEED_CMPS            16.0f
#define ECAR_DEFAULT_RECOVER_SPEED_CMPS         10.0f
#define ECAR_DEFAULT_CORNER_FORWARD_CMPS        8.0f
#define ECAR_DEFAULT_CORNER_TURN_CMPS           12.0f
#define ECAR_DEFAULT_TURN_LIMIT_CMPS            12.0f

#ifndef ECAR_CORNER_CONFIRM_COUNT
#define ECAR_CORNER_CONFIRM_COUNT               2U
#endif
#ifndef ECAR_BOARD_TEST_PWM_LIMIT
#define ECAR_BOARD_TEST_PWM_LIMIT               260
#endif

#endif
