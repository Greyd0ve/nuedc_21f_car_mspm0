#ifndef __APP_RADIO_H
#define __APP_RADIO_H

#include <stdint.h>
#include "app_config.h"

#define RADIO_HEADER             0xA5

#define RADIO_CMD_TARGET_ROOM    0x01
#define RADIO_CMD_ACK            0x02
#define RADIO_CMD_PING           0x03
#define RADIO_CMD_PONG           0x04
#define RADIO_CMD_SLAVE_START    0x05
#define RADIO_CMD_SLAVE_RELEASE  0x06
#define RADIO_CMD_SLAVE_AT_WAIT  0x07

typedef struct {
    uint8_t header;
    uint8_t sender_id;
    uint8_t target_id;
    uint8_t cmd;
    uint8_t room_id;
    uint8_t seq;
    uint8_t checksum;
    uint8_t reserved;
} RadioPacket_t;

typedef struct {
    uint8_t sender_id;
    uint8_t target_id;
    uint8_t cmd;
    uint8_t room_id;
    uint8_t seq;
} AppRadioCommand_t;

void App_Radio_Init(void);
void App_Radio_Task10ms(void);
uint8_t App_Radio_PopCommand(AppRadioCommand_t *cmd);
void App_Radio_ClearPendingCommands(void);

uint8_t App_Radio_IsReady(void);

#if CAR_ROLE_MASTER
uint8_t App_Radio_SendCommand(uint8_t cmd, uint8_t room);
uint8_t App_Radio_SendTargetRoom(uint8_t room);
uint8_t App_Radio_SendSlaveStart(uint8_t room);
uint8_t App_Radio_SendSlaveRelease(uint8_t room);
uint8_t App_Radio_SendPing(uint8_t token);
#endif

#if CAR_ROLE_SLAVE
uint8_t App_Radio_SendSlaveAtWait(uint8_t room);
uint8_t App_Radio_SendPong(uint8_t token);
uint8_t App_Radio_HasNewTarget(uint8_t *room);
uint8_t App_Radio_GetSavedTargetRoom(void);
#endif

#endif
