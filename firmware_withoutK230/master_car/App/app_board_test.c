#include "app_board_test.h"
#include "app_config.h"
#include "Board_Config.h"
#include "app_radio.h"
#include "app_control.h"
#include "StepperEncoder.h"
#include "DebugSerial.h"
#include "JY61P.h"
#include "Key.h"
#include "Motor.h"
#include "Servo.h"
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
    Servo_DisableAll();

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
    else if (key == 4U)
    {
        App_Control_ForcePWMZero();
        Motor_StopAll();
        Servo_DisableAll();
        DebugSerial_SendString("[step-enc,stop]\r\n");
    }
}

void BoardTest_Task100ms(void) { }

void BoardTest_Task200ms(void)
{
    StepperEncoderSnapshot_t snap;
    int32_t xWin, yWin, xRev, yRev, xRem, yRem;
    int32_t cpr = (int32_t)ECAR_STEPPER_ENCODER_CPR;

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

    xRev = snap.xCount / cpr;
    xRem = snap.xCount % cpr;
    if (xRem < 0) { xRem = -xRem; }

    yRev = snap.yCount / cpr;
    yRem = snap.yCount % cpr;
    if (yRem < 0) { yRem = -yRem; }

    DebugSerial_Printf("[step-enc,x=%ld,x_win=%ld,x_rev=%ld,x_rem=%ld,x_bad=%lu,y=%ld,y_win=%ld,y_rev=%ld,y_rem=%ld,y_bad=%lu]\r\n",
        (long)snap.xCount, (long)xWin, (long)xRev, (long)xRem, (unsigned long)snap.xBad,
        (long)snap.yCount, (long)yWin, (long)yRev, (long)yRem, (unsigned long)snap.yBad);
}

#elif CAR_TEST_JY61P_ENABLE

#include "OLED.h"

#define JY61P_PRINT_PERIOD_MS  500U

static uint32_t s_jy61pPrintTimer = 0U;
static uint8_t s_jy61pPrintPaused = 0U;
static uint8_t s_jy61pPage = 0U;
static uint8_t s_jy61pPrevOnline = 0U;
static uint8_t s_jy61pEverOnline = 0U;
static uint8_t s_jy61pYawZeroDone = 0U;

#if CAR_OLED_ENABLE
static void BoardTest_FormatAngleX100(char *buf, int16_t value)
{
    int32_t magnitude = (int32_t)value;
    uint32_t degrees;
    uint32_t fraction;
    uint8_t pos = 0U;

    if (magnitude < 0)
    {
        buf[pos++] = '-';
        magnitude = -magnitude;
    }

    degrees = (uint32_t)magnitude / 100U;
    fraction = (uint32_t)magnitude % 100U;

    if (degrees >= 100U)
    {
        buf[pos++] = (char)('0' + (degrees / 100U));
        buf[pos++] = (char)('0' + ((degrees / 10U) % 10U));
    }
    else if (degrees >= 10U)
    {
        buf[pos++] = (char)('0' + (degrees / 10U));
    }

    buf[pos++] = (char)('0' + (degrees % 10U));
    buf[pos++] = '.';
    buf[pos++] = (char)('0' + (fraction / 10U));
    buf[pos++] = (char)('0' + (fraction % 10U));
    buf[pos] = '\0';
}

static void BoardTest_ShowAngleRow(uint8_t row, const char *label,
                                   int16_t value, uint8_t valid)
{
    char buf[8];

    OLED_ShowString(0, row, label, OLED_6X8);
    if (valid)
    {
        BoardTest_FormatAngleX100(buf, value);
        OLED_ShowString(18, row, buf, OLED_6X8);
    }
    else
    {
        OLED_ShowString(18, row, "--", OLED_6X8);
    }
}
#endif

void BoardTest_Init(void)
{
    App_Control_ForcePWMZero();
    Motor_StopAll();
    Servo_DisableAll();

    s_jy61pPrintTimer = 0U;
    s_jy61pPrintPaused = 0U;
    s_jy61pPage = 0U;
    s_jy61pPrevOnline = 0U;
    s_jy61pEverOnline = 0U;
    s_jy61pYawZeroDone = 0U;

    JY61P_Init();

    DebugSerial_SendString("[board-test,start]\r\n");
    DebugSerial_SendString("[board-test,mode=jy61p]\r\n");
    DebugSerial_SendString("[jy61p,uart=uart0,tx=pa0,rx=pa1,baud=9600]\r\n");
    DebugSerial_SendString("[jy61p,key,k1=yaw_zero,k2=print_pause,k3=page,k4=clear_stats]\r\n");
}

void BoardTest_Task10ms(void)
{
    uint8_t key;
    JY61P_Data_t jdata;
    uint8_t online;

    App_Control_ForcePWMZero();
    Motor_StopAll();

    JY61P_Task10ms();

    JY61P_GetData(&jdata);
    online = JY61P_IsOnline();

    if (!s_jy61pYawZeroDone && online && jdata.angle_valid)
    {
        if (JY61P_ResetRelativeYaw())
        {
            DebugSerial_Printf("[jy61p,yaw_zero,ok,raw_x100=%d]\r\n",
                (int)jdata.yaw_x100);
        }
        else
        {
            DebugSerial_SendString("[jy61p,yaw_zero,rejected,no_valid_angle]\r\n");
        }
        s_jy61pYawZeroDone = 1U;
    }

    if (online && !s_jy61pPrevOnline)
    {
        if (s_jy61pEverOnline)
        {
            DebugSerial_SendString("[jy61p,status,online,recovered]\r\n");
        }
        else
        {
            DebugSerial_SendString("[jy61p,status,online]\r\n");
            s_jy61pEverOnline = 1U;
        }
    }
    else if (!online && s_jy61pPrevOnline)
    {
        DebugSerial_SendString("[jy61p,status,offline,timeout_ms=500]\r\n");
    }
    s_jy61pPrevOnline = online;

    key = Key_GetNum();
    switch (key)
    {
        case 1U:
            {
                JY61P_Data_t zdata;
                JY61P_GetData(&zdata);
                if (JY61P_ResetRelativeYaw())
                {
                    DebugSerial_Printf("[jy61p,yaw_zero,ok,raw_x100=%d]\r\n",
                        (int)zdata.yaw_x100);
                }
                else
                {
                    if ((zdata.last_angle_frame_ms != 0U) &&
                        (zdata.angle_valid == 0U))
                    {
                        DebugSerial_SendString("[jy61p,yaw_zero,rejected,angle_stale]\r\n");
                    }
                    else if (zdata.online == 0U)
                    {
                        DebugSerial_SendString("[jy61p,yaw_zero,rejected,offline]\r\n");
                    }
                    else
                    {
                        DebugSerial_SendString("[jy61p,yaw_zero,rejected,no_valid_angle]\r\n");
                    }
                }
            }
            break;
        case 2U:
            s_jy61pPrintPaused ^= 1U;
            if (s_jy61pPrintPaused)
            {
                DebugSerial_SendString("[jy61p,print,paused]\r\n");
            }
            else
            {
                DebugSerial_SendString("[jy61p,print,resumed]\r\n");
            }
            break;
        case 3U:
            s_jy61pPage ^= 1U;
            if (s_jy61pPage == 0U)
            {
                DebugSerial_SendString("[jy61p,page=angle]\r\n");
            }
            else
            {
                DebugSerial_SendString("[jy61p,page=diagnostic]\r\n");
            }
            break;
        case 4U:
            JY61P_ClearStatistics();
            DebugSerial_SendString("[jy61p,stats,cleared]\r\n");
            break;
        default:
            break;
    }
}

void BoardTest_Task100ms(void)
{
    JY61P_Data_t jdata;

    s_jy61pPrintTimer += 100U;
    if (s_jy61pPrintTimer < JY61P_PRINT_PERIOD_MS)
    {
        return;
    }
    s_jy61pPrintTimer -= JY61P_PRINT_PERIOD_MS;

    if (s_jy61pPrintPaused)
    {
        return;
    }

    JY61P_GetData(&jdata);
    if (s_jy61pPage == 0U)
    {
        if (jdata.angle_valid)
        {
            if (jdata.gyro_valid)
            {
                DebugSerial_Printf("[jy61p,online=%u,roll=%d,pitch=%d,yaw=%d,rel=%d,gz=%d,angle_age=%lu]\r\n",
                    (unsigned int)jdata.online,
                    (int)jdata.roll_x100, (int)jdata.pitch_x100,
                    (int)jdata.yaw_x100, (int)jdata.relative_yaw_x100,
                    (int)jdata.gyro_z_dps_x10,
                    (unsigned long)jdata.angle_age_ms);
            }
            else
            {
                DebugSerial_Printf("[jy61p,online=%u,roll=%d,pitch=%d,yaw=%d,rel=%d,gz=stale,angle_age=%lu]\r\n",
                    (unsigned int)jdata.online,
                    (int)jdata.roll_x100, (int)jdata.pitch_x100,
                    (int)jdata.yaw_x100, (int)jdata.relative_yaw_x100,
                    (unsigned long)jdata.angle_age_ms);
            }
        }
        else
        {
            DebugSerial_Printf("[jy61p,online=%u,angle=stale,angle_age=%lu]\r\n",
                (unsigned int)jdata.online,
                (unsigned long)jdata.angle_age_ms);
        }
    }
    else
    {
        DebugSerial_Printf("[jy61p,af=%lu,gf=%lu,cs=%lu,sync=%lu,ign=%lu,ovf=%lu,age=%lu]\r\n",
            (unsigned long)jdata.angle_frame_count,
            (unsigned long)jdata.gyro_frame_count,
            (unsigned long)jdata.checksum_error_count,
            (unsigned long)jdata.sync_error_count,
            (unsigned long)jdata.unsupported_frame_count,
            (unsigned long)jdata.rx_overflow_count,
            (unsigned long)jdata.link_age_ms);
    }
}

void BoardTest_Task200ms(void)
{
#if CAR_OLED_ENABLE
    JY61P_Data_t odata;
    JY61P_GetData(&odata);

    OLED_Clear();
    if (odata.online == 0U)
    {
        OLED_ShowString(0, 0, "JY61P OFF", OLED_6X8);
    }
    else if (odata.angle_valid == 0U)
    {
        OLED_ShowString(0, 0, "ANGLE STALE", OLED_6X8);
    }
    else
    {
        OLED_ShowString(0, 0, "JY61P OK", OLED_6X8);
    }

    if (s_jy61pPage == 0U)
    {
        BoardTest_ShowAngleRow(2, "R:", odata.roll_x100, odata.angle_valid);
        BoardTest_ShowAngleRow(4, "P:", odata.pitch_x100, odata.angle_valid);
        BoardTest_ShowAngleRow(6, "Y:", odata.relative_yaw_x100, odata.angle_valid);
    }
    else
    {
        OLED_ShowString(0, 2, "A/G:", OLED_6X8);
        OLED_ShowNum(30, 2, odata.angle_frame_count, 4, OLED_6X8);
        OLED_ShowString(60, 2, "/", OLED_6X8);
        OLED_ShowNum(66, 2, odata.gyro_frame_count, 4, OLED_6X8);
        OLED_ShowString(0, 4, "ERR:", OLED_6X8);
        OLED_ShowNum(30, 4,
            odata.checksum_error_count + odata.sync_error_count, 4, OLED_6X8);
        OLED_ShowString(0, 6, "AGE:", OLED_6X8);
        if (odata.link_age_ms == JY61P_AGE_UNKNOWN_MS)
        {
            OLED_ShowString(30, 6, "----", OLED_6X8);
        }
        else
        {
            OLED_ShowNum(30, 6, odata.link_age_ms, 4, OLED_6X8);
        }
    }
#endif
}

#else

void BoardTest_Init(void) { Motor_StopAll(); Servo_DisableAll(); DebugSerial_SendString("[board-test,start]\r\n"); DebugSerial_SendString("[board-test,mode=none]\r\n"); }
void BoardTest_Task10ms(void) { Motor_StopAll(); }
void BoardTest_Task100ms(void) { }
void BoardTest_Task200ms(void) { }

#endif
