#include "app_board_test.h"
#include "app_config.h"
#include "Board_Config.h"
#include "app_radio.h"
#include "app_control.h"
#include "StepperEncoder.h"
#include "DebugSerial.h"
#include "Key.h"
#include "Motor.h"
#include <stdint.h>

#if CAR_TEST_RADIO_ENABLE

#define RADIO_TEST_PING_PERIOD_MS     500U
#define RADIO_TEST_PONG_TIMEOUT_MS    300U
#define RADIO_TEST_STATS_PERIOD_MS   5000U

static uint32_t s_testMs = 0U;
static uint32_t s_pingTimer = 0U;
static uint32_t s_pongTimer = 0U;
static uint32_t s_statTimer = 0U;

static uint8_t s_nextToken = 1U;
static uint8_t s_pendingToken = 0U;
static uint8_t s_waitPong = 0U;

static uint32_t s_txCount = 0U;
static uint32_t s_hwAckCount = 0U;
static uint32_t s_txFailCount = 0U;
static uint32_t s_pongCount = 0U;
static uint32_t s_timeoutCount = 0U;
static uint32_t s_unexpectedCount = 0U;
static uint32_t s_lastRttMs = 0U;
static uint32_t s_pongStartMs = 0U;

static uint32_t s_pingRxCount = 0U;
static uint32_t s_pongAckCount = 0U;
static uint32_t s_pongFailCount = 0U;

void BoardTest_Init(void)
{
    App_Control_ForcePWMZero();
    Motor_StopAll();

    App_Radio_ClearPendingCommands();

    s_testMs = 0U;
    s_pingTimer = 0U;
    s_pongTimer = 0U;
    s_statTimer = 0U;
    s_nextToken = 1U;
    s_pendingToken = 0U;
    s_waitPong = 0U;
    s_txCount = 0U; s_hwAckCount = 0U; s_txFailCount = 0U;
    s_pongCount = 0U; s_timeoutCount = 0U; s_unexpectedCount = 0U;
    s_lastRttMs = 0U; s_pongStartMs = 0U;
    s_pingRxCount = 0U; s_pongAckCount = 0U; s_pongFailCount = 0U;

    DebugSerial_SendString("[board-test,start]\r\n");
    DebugSerial_SendString("[board-test,mode=radio]\r\n");

#if CAR_ROLE_MASTER
    DebugSerial_Printf("[radio-test,role=master,id=%u]\r\n", (unsigned int)CAR_ID);
    DebugSerial_Printf("[radio-test,period_ms=%u,timeout_ms=%u]\r\n",
        (unsigned int)RADIO_TEST_PING_PERIOD_MS, (unsigned int)RADIO_TEST_PONG_TIMEOUT_MS);
#elif CAR_ROLE_SLAVE
    DebugSerial_Printf("[radio-test,role=slave,id=%u]\r\n", (unsigned int)CAR_ID);
#endif

    if (App_Radio_IsReady())
        DebugSerial_SendString("[radio-test,radio=ready]\r\n");
    else
        DebugSerial_SendString("[radio-test,radio=not_ready]\r\n");
}

void BoardTest_Task10ms(void)
{
    AppRadioCommand_t cmd;

    App_Control_ForcePWMZero();
    Motor_StopAll();

    s_testMs += 10U;

    if (!App_Radio_IsReady()) return;

    App_Radio_Task10ms();

    while (App_Radio_PopCommand(&cmd))
    {
#if CAR_ROLE_MASTER
        if (cmd.cmd == RADIO_CMD_PONG && cmd.room_id != 0U)
        {
            if (s_waitPong && cmd.room_id == s_pendingToken)
            {
                s_pongCount++;
                s_lastRttMs = s_testMs - s_pongStartMs;
                s_waitPong = 0U;
                DebugSerial_Printf("[radio-test,master,rx,pong,token=%u,rtt_ms=%u,ok]\r\n",
                    (unsigned int)cmd.room_id, (unsigned int)s_lastRttMs);
            }
            else
            {
                s_unexpectedCount++;
                DebugSerial_Printf("[radio-test,master,rx,pong,token=%u,expected=%u,unexpected]\r\n",
                    (unsigned int)cmd.room_id, (unsigned int)s_pendingToken);
            }
        }
        else
        {
            DebugSerial_Printf("[radio-test,master,ignore,cmd=%u,value=%u]\r\n",
                (unsigned int)cmd.cmd, (unsigned int)cmd.room_id);
        }
#elif CAR_ROLE_SLAVE
        if (cmd.cmd == RADIO_CMD_PING && cmd.room_id != 0U)
        {
            uint8_t token = cmd.room_id;
            s_pingRxCount++;
            DebugSerial_Printf("[radio-test,slave,rx,ping,token=%u]\r\n", (unsigned int)token);

            if (App_Radio_SendPong(token))
            {
                s_pongAckCount++;
                DebugSerial_Printf("[radio-test,slave,tx,pong,token=%u,hw_ack=1]\r\n", (unsigned int)token);
            }
            else
            {
                s_pongFailCount++;
                DebugSerial_Printf("[radio-test,slave,tx,pong,token=%u,hw_ack=0]\r\n", (unsigned int)token);
            }
        }
        else
        {
            DebugSerial_Printf("[radio-test,slave,ignore,cmd=%u,value=%u]\r\n",
                (unsigned int)cmd.cmd, (unsigned int)cmd.room_id);
        }
#endif
    }

#if CAR_ROLE_MASTER
    s_pingTimer += 10U;
    if (s_waitPong)
    {
        s_pongTimer += 10U;
        if (s_pongTimer >= RADIO_TEST_PONG_TIMEOUT_MS)
        {
            s_timeoutCount++;
            s_waitPong = 0U;
            DebugSerial_Printf("[radio-test,master,timeout,token=%u]\r\n", (unsigned int)s_pendingToken);
        }
    }

    if (!s_waitPong && s_pingTimer >= RADIO_TEST_PING_PERIOD_MS)
    {
        uint8_t token = s_nextToken;
        s_pingTimer = 0U;
        s_nextToken++;
        if (s_nextToken == 0U) s_nextToken = 1U;

        s_txCount++;
        if (App_Radio_SendPing(token))
        {
            s_hwAckCount++;
            s_pendingToken = token;
            s_waitPong = 1U;
            s_pongTimer = 0U;
            s_pongStartMs = s_testMs;
            DebugSerial_Printf("[radio-test,master,tx,ping,token=%u,hw_ack=1]\r\n", (unsigned int)token);
        }
        else
        {
            s_txFailCount++;
            DebugSerial_Printf("[radio-test,master,tx,ping,token=%u,hw_ack=0]\r\n", (unsigned int)token);
        }
    }
#endif

    s_statTimer += 10U;
    if (s_statTimer >= RADIO_TEST_STATS_PERIOD_MS)
    {
        s_statTimer = 0U;
#if CAR_ROLE_MASTER
        DebugSerial_Printf("[radio-test,master,stat,tx=%lu,hw_ack=%lu,pong=%lu,tx_fail=%lu,timeout=%lu,unexpected=%lu,last_rtt_ms=%u]\r\n",
            (unsigned long)s_txCount, (unsigned long)s_hwAckCount, (unsigned long)s_pongCount,
            (unsigned long)s_txFailCount, (unsigned long)s_timeoutCount, (unsigned long)s_unexpectedCount,
            (unsigned int)s_lastRttMs);
#elif CAR_ROLE_SLAVE
        DebugSerial_Printf("[radio-test,slave,stat,ping_rx=%lu,pong_ack=%lu,pong_fail=%lu]\r\n",
            (unsigned long)s_pingRxCount, (unsigned long)s_pongAckCount, (unsigned long)s_pongFailCount);
#endif
    }
}

void BoardTest_Task100ms(void) { }
void BoardTest_Task200ms(void) { }

#elif CAR_TEST_STEPPER_ENCODER_ENABLE

#include "Servo.h"

static StepperEncoderSnapshot_t s_lastSnap;
static uint8_t s_snapValid = 0U;

void BoardTest_Init(void)
{
    App_Control_ForcePWMZero();
    Motor_StopAll();
    Servo_DisableAll();

    StepperEncoder_Init();
    StepperEncoder_ResetCounts();
    s_snapValid = 0U;

    DebugSerial_SendString("[board-test,start]\r\n");
    DebugSerial_SendString("[board-test,mode=stepper-encoder]\r\n");
    DebugSerial_Printf("[step-enc,cpr=%u]\r\n", (unsigned int)ECAR_STEPPER_ENCODER_CPR);
    DebugSerial_SendString("[step-enc,key,k1=reset,k4=stop]\r\n");
}

void BoardTest_Task10ms(void)
{
    uint8_t key = Key_GetNum();
    App_Control_ForcePWMZero();
    Motor_StopAll();

    if (key == 1U)
    {
        StepperEncoder_ResetCounts();
        s_snapValid = 0U;
        DebugSerial_SendString("[step-enc,reset]\r\n");
    }
}

void BoardTest_Task100ms(void) { }

void BoardTest_Task200ms(void)
{
    StepperEncoderSnapshot_t snap;
    int32_t xWin, yWin, xRev, yRev, xRem, yRem;

    StepperEncoder_GetSnapshot(&snap);

    if (s_snapValid)
    {
        xWin = snap.xCount - s_lastSnap.xCount;
        yWin = snap.yCount - s_lastSnap.yCount;
    }
    else
    {
        xWin = 0;
        yWin = 0;
        s_snapValid = 1U;
    }

    s_lastSnap = snap;

    if (xWin < 0) xWin = -xWin;
    if (yWin < 0) yWin = -yWin;

    xRev = snap.xCount / (int32_t)ECAR_STEPPER_ENCODER_CPR;
    xRem = snap.xCount % (int32_t)ECAR_STEPPER_ENCODER_CPR;
    if (xRem < 0) { xRem = -xRem; }

    yRev = snap.yCount / (int32_t)ECAR_STEPPER_ENCODER_CPR;
    yRem = snap.yCount % (int32_t)ECAR_STEPPER_ENCODER_CPR;
    if (yRem < 0) { yRem = -yRem; }

    DebugSerial_Printf("[step-enc,x=%ld,x_win=%ld,x_rev=%ld,x_rem=%ld,x_bad=%lu,y=%ld,y_win=%ld,y_rev=%ld,y_rem=%ld,y_bad=%lu]\r\n",
        (long)snap.xCount, (long)xWin, (long)xRev, (long)xRem, (unsigned long)snap.xBad,
        (long)snap.yCount, (long)yWin, (long)yRev, (long)yRem, (unsigned long)snap.yBad);
}

#else

void BoardTest_Init(void) { Motor_StopAll(); DebugSerial_SendString("[board-test,start]\r\n"); DebugSerial_SendString("[board-test,mode=none]\r\n"); }
void BoardTest_Task10ms(void) { Motor_StopAll(); }
void BoardTest_Task100ms(void) { }
void BoardTest_Task200ms(void) { }

#endif
