#include "app_radio.h"
#include "app_21f_car.h"
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

static const uint8_t NRF_ADDR_MASTER[5] = {0x34, 0x43, 0x10, 0x10, 0x01};
static const uint8_t NRF_ADDR_SLAVE[5]  = {0x34, 0x43, 0x10, 0x10, 0x02};

static uint8_t s_radioReady = 0U;

#if CAR_ROLE_MASTER
static uint8_t s_seq = 0U;
#endif

#if CAR_ROLE_SLAVE
static uint8_t s_savedTargetRoom = 0U;
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
    if (pkt->room_id < 1U || pkt->room_id > 8U) return 0U;
    if (pkt->cmd != RADIO_CMD_TARGET_ROOM) return 0U;
    if (Radio_ComputeChecksum(pkt) != pkt->checksum) return 0U;
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
    NRF24L01_RX_Mode();
#else
    NRF24L01_WriteReg(NRF_REG_CONFIG, 0x0E);
#endif

    s_radioReady = 1U;
    RADIO_PRINTF("[radio,init,ok]\r\n");
}

#if CAR_ROLE_MASTER
uint8_t App_Radio_SendTargetRoom(uint8_t room)
{
    RadioPacket_t pkt;
    if (!s_radioReady) return 0U;
    if (room < 1U || room > 8U) return 0U;

    pkt.header    = RADIO_HEADER;
    pkt.sender_id = 1U;
    pkt.target_id = 2U;
    pkt.cmd       = RADIO_CMD_TARGET_ROOM;
    pkt.room_id   = room;
    pkt.seq       = s_seq++;
    pkt.reserved  = 0U;
    pkt.checksum  = Radio_ComputeChecksum(&pkt);

    if (NRF24L01_SendPacket((const uint8_t *)&pkt, NRF_PAYLOAD_SIZE))
    {
        RADIO_PRINTF("[radio,tx,target=%u,ack]\r\n", (unsigned int)room);
        return 1U;
    }
    RADIO_PRINTF("[radio,tx,target=%u,fail]\r\n", (unsigned int)room);
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

    if (!Radio_ValidatePacket(&pkt))
    {
        RADIO_PRINTF("[radio,rx,invalid]\r\n");
        return 0U;
    }

    s_savedTargetRoom = pkt.room_id;
    RADIO_PRINTF("[radio,rx,target=%u]\r\n", (unsigned int)pkt.room_id);
    if (room) *room = pkt.room_id;
    return 1U;
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
    if (App_Radio_HasNewTarget(&room))
    {
        F21Car_SetTargetRoom(room);
    }
#endif
}
