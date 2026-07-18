#include "app_radio.h"
#include "NRF24L01.h"
#include "DebugSerial.h"
#include <stdint.h>

#ifndef RADIO_DEBUG_ENABLE
#define RADIO_DEBUG_ENABLE 1
#endif

#if RADIO_DEBUG_ENABLE
#define RADIO_PRINTF(...) DebugSerial_Printf(__VA_ARGS__)
#else
#define RADIO_PRINTF(...) do { } while (0)
#endif

#define NRF_PAYLOAD_SIZE  8U
#define NRF_RF_CHANNEL    40U
#define RADIO_RX_QUEUE_SIZE 4U

static const uint8_t NRF_ADDR_MASTER[5] = {0x34, 0x43, 0x10, 0x10, 0x01};
static const uint8_t NRF_ADDR_SLAVE[5]  = {0x34, 0x43, 0x10, 0x10, 0x02};

static uint8_t s_radioReady = 0U;

#if CAR_ROLE_MASTER
static uint8_t s_seq = 0U;
#endif

#if CAR_ROLE_SLAVE
static uint8_t s_savedTargetRoom = 0U;
static AppRadioCommand_t s_rxQueue[RADIO_RX_QUEUE_SIZE];
static uint8_t s_rxHead = 0U;
static uint8_t s_rxTail = 0U;
static uint8_t s_rxCount = 0U;
#endif

static uint8_t Radio_ComputeChecksum(const RadioPacket_t *pkt)
{
    return (uint8_t)(pkt->header ^ pkt->sender_id ^ pkt->target_id
        ^ pkt->cmd ^ pkt->room_id ^ pkt->seq ^ pkt->reserved);
}

#if CAR_ROLE_SLAVE
static uint8_t Radio_ValidatePacket(const RadioPacket_t *pkt)
{
    if (pkt->header != RADIO_HEADER) return 0U;
    if (pkt->sender_id != 1U) return 0U;
    if (pkt->target_id != 2U) return 0U;
    if (Radio_ComputeChecksum(pkt) != pkt->checksum) return 0U;

    switch (pkt->cmd)
    {
    case RADIO_CMD_TARGET_ROOM:
        if (pkt->room_id < 1U || pkt->room_id > 8U) return 0U;
        break;
    case RADIO_CMD_SLAVE_START:
    case RADIO_CMD_SLAVE_RELEASE:
        if (pkt->room_id != 3U && pkt->room_id != 4U) return 0U;
        break;
    default:
        return 0U;
    }

    return 1U;
}

static void Radio_ClearQueue(void)
{
    s_rxHead = 0U;
    s_rxTail = 0U;
    s_rxCount = 0U;
}

static uint8_t Radio_PushCommand(const RadioPacket_t *pkt)
{
    AppRadioCommand_t *slot;

    if (s_rxCount >= RADIO_RX_QUEUE_SIZE)
    {
        RADIO_PRINTF("[radio,rx,drop,cmd=%u]\r\n", (unsigned int)pkt->cmd);
        return 0U;
    }

    slot = &s_rxQueue[s_rxHead];
    slot->sender_id = pkt->sender_id;
    slot->target_id = pkt->target_id;
    slot->cmd = pkt->cmd;
    slot->room_id = pkt->room_id;
    slot->seq = pkt->seq;

    s_rxHead++;
    if (s_rxHead >= RADIO_RX_QUEUE_SIZE) s_rxHead = 0U;
    s_rxCount++;

    return 1U;
}
#endif

void App_Radio_Init(void)
{
    NRF24L01_Init();

    if (!NRF24L01_Check())
    {
        RADIO_PRINTF("[radio,init,fail]\r\n");
        return;
    }

    NRF24L01_CE_Set(0);

    NRF24L01_WriteReg(NRF_REG_EN_AA, 0x01);
    NRF24L01_WriteReg(NRF_REG_EN_RXADDR, 0x01);
    NRF24L01_WriteReg(NRF_REG_SETUP_AW, 0x03);
    NRF24L01_WriteReg(NRF_REG_SETUP_RETR, 0x1A);
    NRF24L01_WriteReg(NRF_REG_RF_CH, NRF_RF_CHANNEL);
    NRF24L01_WriteReg(NRF_REG_RF_SETUP, 0x07);
    NRF24L01_WriteReg(NRF_REG_RX_PW_P0, NRF_PAYLOAD_SIZE);
    NRF24L01_WriteReg(NRF_REG_DYNPD, 0x00);
    NRF24L01_WriteReg(NRF_REG_FEATURE, 0x00);

    NRF24L01_WriteBuf(NRF_REG_RX_ADDR_P0, NRF_ADDR_SLAVE, 5);
    NRF24L01_WriteBuf(NRF_REG_TX_ADDR, NRF_ADDR_SLAVE, 5);

    NRF24L01_FlushTX();
    NRF24L01_FlushRX();
    NRF24L01_ClearIRQFlags();

#if CAR_ROLE_SLAVE
    Radio_ClearQueue();
    NRF24L01_RX_Mode();
#else
    NRF24L01_WriteReg(NRF_REG_CONFIG, 0x0E);
#endif

    s_radioReady = 1U;
    RADIO_PRINTF("[radio,init,ok]\r\n");
}

#if CAR_ROLE_MASTER
uint8_t App_Radio_SendCommand(uint8_t cmd, uint8_t room)
{
    RadioPacket_t pkt;
    if (!s_radioReady) return 0U;

    switch (cmd)
    {
    case RADIO_CMD_TARGET_ROOM:
        if (room < 1U || room > 8U) return 0U;
        break;
    case RADIO_CMD_SLAVE_START:
    case RADIO_CMD_SLAVE_RELEASE:
        if (room != 3U && room != 4U) return 0U;
        break;
    default:
        return 0U;
    }

    pkt.header    = RADIO_HEADER;
    pkt.sender_id = 1U;
    pkt.target_id = 2U;
    pkt.cmd       = cmd;
    pkt.room_id   = room;
    pkt.seq       = s_seq++;
    pkt.reserved  = 0U;
    pkt.checksum  = Radio_ComputeChecksum(&pkt);

    if (NRF24L01_SendPacket((const uint8_t *)&pkt, NRF_PAYLOAD_SIZE))
    {
        RADIO_PRINTF("[radio,tx,cmd=%u,room=%u,ack]\r\n",
            (unsigned int)cmd, (unsigned int)room);
        return 1U;
    }
    RADIO_PRINTF("[radio,tx,cmd=%u,room=%u,fail]\r\n",
        (unsigned int)cmd, (unsigned int)room);
    return 0U;
}

uint8_t App_Radio_SendTargetRoom(uint8_t room)
{
    return App_Radio_SendCommand(RADIO_CMD_TARGET_ROOM, room);
}

uint8_t App_Radio_SendSlaveStart(uint8_t room)
{
    return App_Radio_SendCommand(RADIO_CMD_SLAVE_START, room);
}

uint8_t App_Radio_SendSlaveRelease(uint8_t room)
{
    return App_Radio_SendCommand(RADIO_CMD_SLAVE_RELEASE, room);
}
#endif

#if CAR_ROLE_SLAVE
uint8_t App_Radio_PopCommand(AppRadioCommand_t *cmd)
{
    if (s_rxCount == 0U) return 0U;

    if (cmd != 0)
    {
        *cmd = s_rxQueue[s_rxTail];
    }

    s_rxTail++;
    if (s_rxTail >= RADIO_RX_QUEUE_SIZE) s_rxTail = 0U;
    s_rxCount--;

    return 1U;
}

void App_Radio_ClearPendingCommands(void)
{
    Radio_ClearQueue();
}

uint8_t App_Radio_HasNewTarget(uint8_t *room)
{
    AppRadioCommand_t cmd;

    if (!App_Radio_PopCommand(&cmd))
    {
        return 0U;
    }

    if (cmd.cmd != RADIO_CMD_TARGET_ROOM)
    {
        return 0U;
    }

    s_savedTargetRoom = cmd.room_id;
    if (room) *room = cmd.room_id;
    return 1U;
}

uint8_t App_Radio_GetSavedTargetRoom(void)
{
    return s_savedTargetRoom;
}
#else
uint8_t App_Radio_PopCommand(AppRadioCommand_t *cmd)
{
    (void)cmd;
    return 0U;
}

void App_Radio_ClearPendingCommands(void)
{
}
#endif

void App_Radio_Task10ms(void)
{
#if CAR_ROLE_SLAVE
    RadioPacket_t pkt;
    uint8_t rxLimit = 3U;

    while (rxLimit > 0U && NRF24L01_ReceivePacket((uint8_t *)&pkt, NRF_PAYLOAD_SIZE))
    {
        rxLimit--;

        if (!Radio_ValidatePacket(&pkt))
        {
            RADIO_PRINTF("[radio,rx,invalid]\r\n");
            continue;
        }

        if (Radio_PushCommand(&pkt))
        {
            if (pkt.cmd == RADIO_CMD_TARGET_ROOM)
            {
                s_savedTargetRoom = pkt.room_id;
            }

            RADIO_PRINTF("[radio,rx,cmd=%u,room=%u]\r\n",
                (unsigned int)pkt.cmd, (unsigned int)pkt.room_id);
        }
    }
#endif
}
