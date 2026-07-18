#ifndef __APP_RADIO_H
#define __APP_RADIO_H

#include <stdint.h>
#include "app_config.h"

#define RADIO_HEADER             0xA5

#define RADIO_CMD_TARGET_ROOM    0x01
#define RADIO_CMD_ACK            0x02
#define RADIO_CMD_PING           0x03
#define RADIO_CMD_PONG           0x04

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

void App_Radio_Init(void);
void App_Radio_Task10ms(void);

#if CAR_ROLE_MASTER
uint8_t App_Radio_SendTargetRoom(uint8_t room);
#endif

#if CAR_ROLE_SLAVE
uint8_t App_Radio_HasNewTarget(uint8_t *room);
uint8_t App_Radio_GetSavedTargetRoom(void);
#endif

#endif
