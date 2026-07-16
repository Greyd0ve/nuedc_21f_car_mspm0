#ifndef __APP_TUNING_H
#define __APP_TUNING_H

/*
 * E-car tuning parameters.
 *
 * Keep frequently changed control parameters here, so competition-time tuning
 * does not require searching app_config.h, app_e_car.c, app_control.c and
 * app_line.c at the same time.
 */

#include "app_config.h"

/* Motor speed closed-loop tuning. */
#define TUNE_SPEED_FILTER_ALPHA             0.20f
#define TUNE_PWM_SLEW_STEP                  5

#define TUNE_SPEED_PI_KP                    1.2f
#define TUNE_SPEED_PI_KI                    0.08f
#define TUNE_SPEED_I_LIMIT                  45.0f
#define TUNE_WHEEL_TARGET_LIMIT_CMPS        80.0f

/*
 * Piecewise speed feed-forward PWM model.
 *
 * Current measured reference:
 * - 15 cm/s needs about PWM 160.
 * - 18 cm/s needs about PWM 168.
 */
#define TUNE_SPEED_FF_DEAD_BAND_CMPS        0.5f
#define TUNE_SPEED_FF_BREAK_CMPS            15.0f
#define TUNE_SPEED_FF_LOW_BASE_PWM          135.0f
#define TUNE_SPEED_FF_LOW_K                 1.7f
#define TUNE_SPEED_FF_HIGH_BASE_PWM         160.5f
#define TUNE_SPEED_FF_HIGH_K                2.2f

/* Line-following default values. */
#define TUNE_LINE_KP                        0.040f
#define TUNE_LINE_KD                        0.025f
#define TUNE_LINE_TURN_LIMIT_CMPS           ECAR_DEFAULT_TURN_LIMIT_CMPS
#define TUNE_LINE_FILTER_ALPHA              0.58f

#endif
