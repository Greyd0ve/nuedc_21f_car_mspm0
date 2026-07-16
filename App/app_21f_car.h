#ifndef __APP_21F_CAR_H
#define __APP_21F_CAR_H

#include <stdint.h>

#define F21_CROSS_CONFIRM_MS            30U
#define F21_CROSS_ADVANCE_CM            4.0f
#define F21_UNLOAD_WAIT_MS              3000U

#define F21_LINE_BASE_SPEED_CMPS        10.0f
#define F21_CROSS_ADVANCE_SPEED_CMPS    10.0f
#define F21_TURN_SPEED_CMPS             12.0f

/* F21_TURN_90_PULSE: referenced from E-car corner_turn_pulse = 90U */
#define F21_TURN_90_PULSE               90U
#define F21_TURN_180_PULSE              0

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

void F21Car_Init(void);
void F21Car_Tick1ms(void);
void F21Car_Task10ms(void);
void F21Car_KeyProcess(void);
void F21Car_Task100ms(void);
void F21Car_Task200ms(void);

#endif
