#ifndef __APP_21F_CAR_H
#define __APP_21F_CAR_H

#include <stdint.h>
#include "app_config.h"

#define F21_CROSS_CONFIRM_MS            20U

/*
 * Cross advance is an empirically verified state-machine distance after
 * detecting a cross.  It is intentionally independent of the 17.7 cm
 * sensor-to-axle mechanical dimension in app_config.h.
 */
#define F21_CROSS_ADVANCE_BASE_CM       6.0f
#define F21_FAR_CROSS_ADVANCE_BASE_CM   3.0f
#define F21_CROSS_ADVANCE_TRIM_CM       0.0f
#define F21_FAR_CROSS_ADVANCE_TRIM_CM   0.0f

#define F21_CROSS_ADVANCE_CM \
    (F21_CROSS_ADVANCE_BASE_CM + F21_CROSS_ADVANCE_TRIM_CM)
#define F21_FAR_CROSS_ADVANCE_CM \
    (F21_FAR_CROSS_ADVANCE_BASE_CM + F21_FAR_CROSS_ADVANCE_TRIM_CM)

#define F21_UNLOAD_WAIT_MS              2000U
#define F21_STEPPER_STOP_TIMEOUT_MS      500U

#define F21_SIDE_CROSS_BLACK_MIN        4U
#define F21_SIDE_CROSS_LEFT_MASK        0x3FU
#define F21_SIDE_CROSS_RIGHT_MASK       0xFCU

#define F21_LINE_BASE_SPEED_CMPS        18.0f
#define F21_FAR_LINE_BASE_SPEED_CMPS    22.0f
#define F21_FAR_FAST_LINE_SPEED_CMPS    27.0f
#define F21_CROSS_ADVANCE_SPEED_CMPS    12.0f
#define F21_TURN_SPEED_CMPS             12.0f

/*
 * Turn encoder count thresholds.
 *
 * Theoretical values based on chassis geometry:
 *   90°:  ECAR_TURN_90_ENCODER_COUNT_THEORY  = 2505
 *   180°: ECAR_TURN_180_ENCODER_COUNT_THEORY = 5010
 *
 * Control initial values preserve the verified withoutK230 behavior, scaled
 * independently from 367 to 4096 encoder counts/revolution:
 *   90°:  180 * 4096 / 367 ~= 2009 -> 2010
 *   180°: 440 * 4096 / 367 ~= 4911 -> 4910
 *
 * These are not measured values for this chassis.  Calibrate the 90° and
 * 180° thresholds independently on the real vehicle.
 */
#define F21_TURN_90_PULSE               2010U
#define F21_TURN_180_PULSE              4910U

/* [legacy] F21_TURN_PWM was raw PWM direct drive; replaced by speed closed-loop. */
#define F21_TURN_PWM                    140

typedef enum
{
    F21_CAR_IDLE = 0,
    F21_CAR_WAIT_START,

    F21_CAR_MAIN_LINE_RUN,
    F21_CAR_FIRST_CROSS_ADVANCE,
    F21_CAR_FIRST_TURN,

    F21_CAR_AFTER_FIRST_TURN_RUN,
    F21_CAR_SECOND_CROSS_ADVANCE,
    F21_CAR_SECOND_TURN,

    F21_CAR_FINAL_ROOM_RUN,
    F21_CAR_ARRIVED_ROOM,
    F21_CAR_UNLOAD_WAIT,

    F21_CAR_TURN_AROUND,

    F21_CAR_RETURN_MAIN_LINE_RUN,
    F21_CAR_RETURN_CROSS_ADVANCE,
    F21_CAR_RETURN_TURN,
    F21_CAR_RETURN_FINAL_RUN,
    F21_CAR_RETURN_FINAL_TURN_AROUND,

    F21_CAR_WAIT_STEPPER_STOP,
    F21_CAR_FINISH,
    F21_CAR_STOP,
    F21_CAR_FAULT
} F21CarState_t;

typedef enum
{
    F21_ROUTE_SIMPLE = 0,
    F21_ROUTE_FAR
} F21RouteType_t;

typedef enum
{
    F21_TURN_LEFT = 0,
    F21_TURN_RIGHT
} F21TurnDir_t;

typedef struct
{
    F21RouteType_t type;
    float firstDetectStartCm;
    F21TurnDir_t firstTurn;
    float secondDetectStartCm;
    F21TurnDir_t secondTurn;
    float finalRunCm;
} F21Route_t;

typedef struct
{
    float detectStartCm;
    F21TurnDir_t turnDir;
    float finalRunCm;
} F21ReturnRoute_t;

typedef struct
{
    float firstDetectStartCm;
    F21TurnDir_t firstTurn;
    float secondDetectStartCm;
    F21TurnDir_t secondTurn;
    float finalRunCm;
} F21FarReturnRoute_t;

void F21Car_Init(void);
void F21Car_Tick1ms(void);
void F21Car_Task10ms(void);
void F21Car_HandleKey(uint8_t key);
void F21Car_KeyProcess(void);
void F21Car_Task100ms(void);
void F21Car_Task200ms(void);

void F21Car_ResetTask(void);
uint8_t F21Car_IsModeSwitchAllowed(void);
void F21Car_CancelLedDisplay(void);

void F21Car_SetTargetRoom(uint8_t room);
uint8_t F21Car_GetTargetRoom(void);

#endif
