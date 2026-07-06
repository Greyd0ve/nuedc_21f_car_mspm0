#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include <stdint.h>

/* Safety switches. Remote start and board-test motor output are opt-in. */
#ifndef ECAR_ENABLE_REMOTE_START
#define ECAR_ENABLE_REMOTE_START              0
#endif
#ifndef ECAR_BOARD_TEST_MODE
#define ECAR_BOARD_TEST_MODE                  0
#endif

/* 1 ms timer ISR only raises saturated task counters. */
#ifndef ECAR_TASK_COUNT_MAX
#define ECAR_TASK_COUNT_MAX                   5U
#endif
#define ECAR_ENCODER_SPEED_PERIOD_MS          5U
#define ECAR_CONTROL_PERIOD_MS                10U
#define ECAR_SERIAL_PLOT_PERIOD_MS            100U
#define ECAR_OLED_REFRESH_PERIOD_MS           200U

/* JGA25-370B + 65 mm wheel nominal conversion.
 * ECAR_ENCODER_EDGE_MULTIPLIER must match SysConfig edge mode:
 * 1.0 = A phase single edge, 2.0 = A phase double edge, 4.0 = AB x4.
 */
#define ECAR_PI_F                             3.1415926f
#define ECAR_WHEEL_DIAMETER_CM               6.5f
#define ECAR_WHEEL_CIRCUMFERENCE_CM          (ECAR_WHEEL_DIAMETER_CM * ECAR_PI_F)
#define ECAR_ENCODER_BASE_PPR                11.0f
#define ECAR_GEAR_RATIO                      35.5f
#ifndef ECAR_ENCODER_EDGE_MULTIPLIER
#define ECAR_ENCODER_EDGE_MULTIPLIER         1.0f
#endif
#define ECAR_ENCODER_PULSE_PER_REV           (ECAR_ENCODER_BASE_PPR * ECAR_GEAR_RATIO * ECAR_ENCODER_EDGE_MULTIPLIER)
#define ECAR_CM_PER_PULSE                    (ECAR_WHEEL_CIRCUMFERENCE_CM / ECAR_ENCODER_PULSE_PER_REV)

/* Square track nominal dimensions; keep serial tuning enabled for slip/error. */
#define ECAR_LAP_DISTANCE_CM                  400.0f
#define ECAR_MIN_CORNER_INTERVAL_CM           55.0f
#define ECAR_DEFAULT_LAP_PULSE                ((int32_t)((ECAR_LAP_DISTANCE_CM / ECAR_CM_PER_PULSE) + 0.5f))
#define ECAR_DEFAULT_MIN_CORNER_INTERVAL_PULSE ((int32_t)((ECAR_MIN_CORNER_INTERVAL_CM / ECAR_CM_PER_PULSE) + 0.5f))

/* Conservative first-run motion defaults, in cm/s. */
#define ECAR_DEFAULT_BASE_SPEED_CMPS          24.0f
#define ECAR_DEFAULT_RECOVER_SPEED_CMPS       14.0f
#define ECAR_DEFAULT_CORNER_FORWARD_CMPS      10.0f
#define ECAR_DEFAULT_CORNER_TURN_CMPS         28.0f
#define ECAR_DEFAULT_TURN_LIMIT_CMPS          35.0f

#ifndef ECAR_CORNER_CONFIRM_COUNT
#define ECAR_CORNER_CONFIRM_COUNT             2U
#endif
#ifndef ECAR_BOARD_TEST_PWM_LIMIT
#define ECAR_BOARD_TEST_PWM_LIMIT             180
#endif

#endif
