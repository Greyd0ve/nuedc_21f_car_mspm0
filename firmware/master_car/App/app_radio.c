#include "app_radio.h"
#include "app_config.h"
#include "NRF24L01.h"
#include "BeepLed.h"
#include "Serial.h"
#include <stdint.h>

#ifndef RADIO_DEBUG_ENABLE
#define RADIO_DEBUG_ENABLE 0
#endif

#if RADIO_DEBUG_ENABLE
#define RADIO_PRINTF(...) Serial_Printf(__VA_ARGS__)
#else
#define RADIO_PRINTF(...) do { } while (0)
#endif

#define NRF_PAYLOAD_SIZE  8U
#define NRF_RF_CHANNEL    40U

static const uint8_t NRF_ADDR_MASTER[5] = {0x34, 0x43, 0x10, 0x10, 0x01};
static const uint8_t NRF_ADDR_SLAVE[5]  = {0x34, 0x43, 0x10, 0x10, 0x02};

static uint8_t s_radioReady = 0U;
static uint8_t s_seq = 0U;

#if CAR_ROLE_SLAVE
static uint8_t s_savedTargetRoom = 0U;
#endif

static uint8_t Radio_ComputeChecksum(const RadioPacket_t *pkt)
{
    return (uint8_t)(pkt->header ^ pkt->sender_id ^ pkt->target_id
        ^ pkt->cmd ^ pkt->room_id ^ pkt->seq ^ pkt->reserved);
}

static uint8_t Radio_ValidatePacket(const RadioPacket_t *pkt)
{
    if (pkt->header != RADIO_HEADER) return 0U;

#if CAR_ROLE_SLAVE
    if (pkt->target_id != 2U) return 0U;
#else
    if (pkt->target_id != 1U) return 0U;
#endif

    if (Radio_ComputeChecksum(pkt) != pkt->checksum) return 0U;
    return 1U;
}

void App_Radio_Init(void)
{
    NRF24L01_Init();

    NRF24L01_CE_Set(0);
    NRF24L01_FlushTX();
    NRF24L01_FlushRX();
    NRF24L01_ClearIRQFlags();

    NRF24L01_WriteReg(0x01, 0x01);
    NRF24L01_WriteReg(0x02, 0x01);
    NRF24L01_WriteReg(0x03, 0x03);
    NRF24L01_WriteReg(0x04, 0x00);
    NRF24L01_WriteReg(0x05, NRF_RF_CHANNEL);
    NRF24L01_WriteReg(0x06, 0x07);

    NRF24L01_WriteBuf(0x0A, NRF_ADDR_MASTER, 5);
    NRF24L01_WriteBuf(0x10, NRF_ADDR_MASTER, 5);
    NRF24L01_WriteReg(0x11, NRF_PAYLOAD_SIZE);

#if CAR_ROLE_SLAVE
    NRF24L01_WriteBuf(0x0A, NRF_ADDR_SLAVE, 5);
    {
        uint8_t cfg = 0x0B;
        NRF24L01_WriteReg(0x00, cfg);
    }
#else
    {
        uint8_t cfg = 0x0A;
        NRF24L01_WriteReg(0x00, cfg);
    }
#endif
    NRF24L01_CE_Set(1);

    if (NRF24L01_Check())
    {
        s_radioReady = 1U;
        RADIO_PRINTF("[radio,init,ok]\r\n");
    }
    else
    {
        RADIO_PRINTF("[radio,init,fail]\r\n");
    }
}

#if CAR_ROLE_MASTER
uint8_t App_Radio_SendTargetRoom(uint8_t room)
{
    RadioPacket_t pkt;

    if (!s_radioReady) return 0U;

    pkt.header    = RADIO_HEADER;
    pkt.sender_id = 1U;
    pkt.target_id = 2U;
    pkt.cmd       = RADIO_CMD_TARGET_ROOM;
    pkt.room_id   = room;
    pkt.seq       = s_seq++;
    pkt.reserved  = 0U;
    pkt.checksum  = Radio_ComputeChecksum(&pkt);

    NRF24L01_CE_Set(0);
    NRF24L01_WriteBuf(0x10, NRF_ADDR_SLAVE, 5);
    NRF24L01_CE_Set(1);

    if (NRF24L01_SendPacket((const uint8_t *)&pkt, NRF_PAYLOAD_SIZE))
    {
        RADIO_PRINTF("[radio,tx,room=%u,ok]\r\n", (unsigned int)room);
        return 1U;
    }

    RADIO_PRINTF("[radio,tx,room=%u,fail]\r\n", (unsigned int)room);
    return 0U;
}
#endif

#if CAR_ROLE_SLAVE
uint8_t App_Radio_HasNewTarget(uint8_t *room)
{
    RadioPacket_t pkt;

    if (!s_radioReady) return 0U;

    if (!NRF24L01_ReceivePacket((uint8_t *)&pkt, NRF_PAYLOAD_SIZE))
        return 0U;

    if (!Radio_ValidatePacket(&pkt)) return 0U;

    if (pkt.cmd == RADIO_CMD_TARGET_ROOM)
    {
        s_savedTargetRoom = pkt.room_id;
        RADIO_PRINTF("[radio,rx,target=%u]\r\n", (unsigned int)pkt.room_id);

        {
            RadioPacket_t ack;
            ack.header    = RADIO_HEADER;
            ack.sender_id = 2U;
            ack.target_id = 1U;
            ack.cmd       = RADIO_CMD_ACK;
            ack.room_id   = pkt.room_id;
            ack.seq       = pkt.seq;
            ack.reserved  = 0U;
            ack.checksum  = Radio_ComputeChecksum(&ack);

            NRF24L01_CE_Set(0);
            NRF24L01_WriteBuf(0x10, NRF_ADDR_MASTER, 5);
            NRF24L01_CE_Set(1);

            if (NRF24L01_SendPacket((const uint8_t *)&ack, NRF_PAYLOAD_SIZE))
            {
                RADIO_PRINTF("[radio,tx,ack,ok]\r\n");
            }
        }

        if (room) *room = pkt.room_id;
        return 1U;
    }

    return 0U;
}

uint8_t App_Radio_GetSavedTargetRoom(void)
{
    return s_savedTargetRoom;
}
#endif

void App_Radio_Task10ms(void)
{
#if CAR_ROLE_SLAVE
    uint8_t room;
    App_Radio_HasNewTarget(&room);
#endif
}
