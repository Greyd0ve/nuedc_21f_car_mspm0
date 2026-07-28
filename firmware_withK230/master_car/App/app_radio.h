#ifndef __APP_RADIO_H
#define __APP_RADIO_H

#include <stdint.h>
#include "app_config.h"

#define RADIO_HEADER             0xA5

#define RADIO_CMD_PING           0x03U
#define RADIO_CMD_PONG           0x04U
#define RADIO_CMD_USER           0x10U

typedef struct {
    uint8_t header;
    uint8_t sender_id;
    uint8_t target_id;
    uint8_t cmd;
    uint8_t value;
    uint8_t seq;
    uint8_t checksum;
    uint8_t reserved;
} RadioPacket_t;

typedef struct {
    uint8_t sender_id;
    uint8_t target_id;
    uint8_t cmd;
    uint8_t value;
    uint8_t seq;
} AppRadioCommand_t;

void App_Radio_Init(void);
void App_Radio_Task10ms(void);
uint8_t App_Radio_PopCommand(AppRadioCommand_t *cmd);
void App_Radio_ClearPendingCommands(void);

uint8_t App_Radio_IsReady(void);

#if CAR_ROLE_MASTER
uint8_t App_Radio_SendCommand(uint8_t cmd, uint8_t value);
uint8_t App_Radio_SendPing(uint8_t token);
#endif

#if CAR_ROLE_SLAVE
uint8_t App_Radio_SendPong(uint8_t token);
#endif

#endif
