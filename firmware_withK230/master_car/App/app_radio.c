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

#define NRF_PAYLOAD_SIZE       8U
#define NRF_RF_CHANNEL         40U
#define RADIO_RX_QUEUE_SIZE    4U

static const uint8_t NRF_ADDR_MASTER[5] = {0x34, 0x43, 0x10, 0x10, 0x01};
static const uint8_t NRF_ADDR_SLAVE[5]  = {0x34, 0x43, 0x10, 0x10, 0x02};

static uint8_t s_radioReady = 0U;
static uint8_t s_seq = 0U;

static AppRadioCommand_t s_rxQueue[RADIO_RX_QUEUE_SIZE];
static uint8_t s_rxHead = 0U;
static uint8_t s_rxTail = 0U;
static uint8_t s_rxCount = 0U;

static const uint8_t *Radio_GetLocalAddress(void)
{
#if CAR_ROLE_MASTER
    return NRF_ADDR_MASTER;
#else
    return NRF_ADDR_SLAVE;
#endif
}

static const uint8_t *Radio_GetPeerAddress(void)
{
#if CAR_ROLE_MASTER
    return NRF_ADDR_SLAVE;
#else
    return NRF_ADDR_MASTER;
#endif
}

static uint8_t Radio_GetLocalId(void)
{
#if CAR_ROLE_MASTER
    return 1U;
#else
    return 2U;
#endif
}

static uint8_t Radio_GetPeerId(void)
{
#if CAR_ROLE_MASTER
    return 2U;
#else
    return 1U;
#endif
}

static uint8_t Radio_ComputeChecksum(const RadioPacket_t *pkt)
{
    return (uint8_t)(pkt->header ^ pkt->sender_id ^ pkt->target_id
        ^ pkt->cmd ^ pkt->value ^ pkt->seq ^ pkt->reserved);
}

static void Radio_ClearQueue(void)
{
    s_rxHead = 0U;
    s_rxTail = 0U;
    s_rxCount = 0U;
}

static void Radio_RestoreLocalRxMode(void)
{
    NRF24L01_CE_Set(0U);
    NRF24L01_WriteBuf(NRF_REG_RX_ADDR_P0, Radio_GetLocalAddress(), 5U);
    NRF24L01_WriteBuf(NRF_REG_TX_ADDR, Radio_GetPeerAddress(), 5U);
    NRF24L01_FlushRX();
    NRF24L01_ClearIRQFlags();
    NRF24L01_RX_Mode();
}

static uint8_t Radio_ValidatePacket(const RadioPacket_t *pkt)
{
    if (pkt->header != RADIO_HEADER) return 0U;
    if (Radio_ComputeChecksum(pkt) != pkt->checksum) return 0U;

#if CAR_ROLE_MASTER
    if (pkt->sender_id != 2U) return 0U;
    if (pkt->target_id != 1U) return 0U;
    if (pkt->cmd == RADIO_CMD_PONG)
    {
        if (pkt->value == 0U) return 0U;
    }
    else
    {
        return 0U;
    }
#elif CAR_ROLE_SLAVE
    if (pkt->sender_id != 1U) return 0U;
    if (pkt->target_id != 2U) return 0U;

    switch (pkt->cmd)
    {
    case RADIO_CMD_PING:
        if (pkt->value == 0U) return 0U;
        break;
    default:
        return 0U;
    }
#else
    return 0U;
#endif

    return 1U;
}

static uint8_t Radio_IsSendAllowed(uint8_t cmd, uint8_t value)
{
    if (value == 0U)
    {
        return 0U;
    }

#if CAR_ROLE_MASTER
    if (cmd == RADIO_CMD_PING)
        return 1U;
    return 0U;
#elif CAR_ROLE_SLAVE
    if (cmd == RADIO_CMD_PONG)
        return 1U;
    return 0U;
#else
    (void)cmd;
    return 0U;
#endif
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
    slot->value = pkt->value;
    slot->seq = pkt->seq;

    s_rxHead++;
    if (s_rxHead >= RADIO_RX_QUEUE_SIZE) s_rxHead = 0U;
    s_rxCount++;

    return 1U;
}

void App_Radio_Init(void)
{
    s_radioReady = 0U;
    s_seq = 0U;
    Radio_ClearQueue();

    NRF24L01_Init();

    if (!NRF24L01_Check())
    {
        RADIO_PRINTF("[radio,init,fail]\r\n");
        return;
    }

    NRF24L01_CE_Set(0U);

    NRF24L01_WriteReg(NRF_REG_EN_AA, 0x01U);
    NRF24L01_WriteReg(NRF_REG_EN_RXADDR, 0x01U);
    NRF24L01_WriteReg(NRF_REG_SETUP_AW, 0x03U);
    NRF24L01_WriteReg(NRF_REG_SETUP_RETR, 0x1AU);
    NRF24L01_WriteReg(NRF_REG_RF_CH, NRF_RF_CHANNEL);
    NRF24L01_WriteReg(NRF_REG_RF_SETUP, 0x07U);
    NRF24L01_WriteReg(NRF_REG_RX_PW_P0, NRF_PAYLOAD_SIZE);
    NRF24L01_WriteReg(NRF_REG_DYNPD, 0x00U);
    NRF24L01_WriteReg(NRF_REG_FEATURE, 0x00U);

    NRF24L01_WriteBuf(NRF_REG_RX_ADDR_P0, Radio_GetLocalAddress(), 5U);
    NRF24L01_WriteBuf(NRF_REG_TX_ADDR, Radio_GetPeerAddress(), 5U);

    NRF24L01_FlushTX();
    NRF24L01_FlushRX();
    NRF24L01_ClearIRQFlags();
    Radio_ClearQueue();
    NRF24L01_RX_Mode();

    s_radioReady = 1U;
    RADIO_PRINTF("[radio,init,ok]\r\n");
}

static uint8_t Radio_SendCommand(uint8_t cmd, uint8_t value)
{
    RadioPacket_t pkt;
    uint8_t ok;

    if (!s_radioReady) return 0U;
    if (!Radio_IsSendAllowed(cmd, value)) return 0U;

    pkt.header    = RADIO_HEADER;
    pkt.sender_id = Radio_GetLocalId();
    pkt.target_id = Radio_GetPeerId();
    pkt.cmd       = cmd;
    pkt.value     = value;
    pkt.seq       = s_seq++;
    pkt.reserved  = 0U;
    pkt.checksum  = Radio_ComputeChecksum(&pkt);

    NRF24L01_CE_Set(0U);
    NRF24L01_WriteBuf(NRF_REG_TX_ADDR, Radio_GetPeerAddress(), 5U);
    NRF24L01_WriteBuf(NRF_REG_RX_ADDR_P0, Radio_GetPeerAddress(), 5U);

    ok = NRF24L01_SendPacket((const uint8_t *)&pkt, NRF_PAYLOAD_SIZE);

    Radio_RestoreLocalRxMode();

    RADIO_PRINTF("[radio,tx,cmd=%u,value=%u,%s]\r\n",
        (unsigned int)cmd, (unsigned int)value, ok ? "ack" : "fail");

    return ok;
}

#if CAR_ROLE_MASTER
uint8_t App_Radio_SendPing(uint8_t token)
{
    return Radio_SendCommand(RADIO_CMD_PING, token);
}
#endif

#if CAR_ROLE_SLAVE
uint8_t App_Radio_SendPong(uint8_t token)
{
    return Radio_SendCommand(RADIO_CMD_PONG, token);
}
#endif

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

uint8_t App_Radio_IsReady(void)
{
    return s_radioReady;
}

void App_Radio_Task10ms(void)
{
    RadioPacket_t pkt;
    uint8_t rxLimit = 3U;

    if (!s_radioReady) return;

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
            RADIO_PRINTF("[radio,rx,cmd=%u,value=%u]\r\n",
                (unsigned int)pkt.cmd, (unsigned int)pkt.value);
        }
    }
}
