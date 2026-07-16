#ifndef __APP_F_CAR_H
#define __APP_F_CAR_H

#include <stdint.h>

#define FCAR_OLED_ENABLE                ECAR_OLED_ENABLE
#define FCAR_TEST_IMU_ENABLE            ECAR_TEST_IMU_ENABLE
#define FCAR_BOARD_TEST_MODE            ECAR_BOARD_TEST_MODE
#define FCAR_ENCODER_SPEED_PERIOD_MS    ECAR_ENCODER_SPEED_PERIOD_MS
#define FCAR_ENCODER_MINIMAL_DEBUG      ECAR_ENCODER_MINIMAL_DEBUG

typedef enum
{
    F_CAR_IDLE = 0,
    F_CAR_WAIT_ROOM_ID,
    F_CAR_WAIT_LOAD,
    F_CAR_DELIVER_START,
    F_CAR_LINE_RUN,
    F_CAR_INTERSECTION_HANDLE,
    F_CAR_ROOM_APPROACH,
    F_CAR_WAIT_UNLOAD,
    F_CAR_RETURN_START,
    F_CAR_RETURN_RUN,
    F_CAR_FINISH,
    F_CAR_FAULT
} FCarState_t;

#define FCAR_FAULT_NONE             0U
#define FCAR_FAULT_LINE_LOST        1U

void FCar_Init(void);
void FCar_Reset(void);
void FCar_Start(void);
void FCar_Stop(void);
void FCar_Task10ms(void);
void FCar_KeyProcess(void);
void FCar_ShowStatus(void);

uint8_t FCar_GetTargetRoom(void);
FCarState_t FCar_GetState(void);
uint32_t FCar_GetRunningTimeMs(void);
uint8_t FCar_GetFaultCode(void);

void FCar_PromptTick1ms(void);

void FCar_SetTargetRoom(uint8_t room);
void FCar_SimulateLoadComplete(void);
void FCar_SimulateUnloadComplete(void);

void FCar_SetRedLed(uint8_t on);
void FCar_SetGreenLed(uint8_t on);
void FCar_SetYellowLed(uint8_t on);
void FCar_AllLedOff(void);

#endif
