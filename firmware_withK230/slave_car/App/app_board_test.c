#include "app_board_test.h"
#include "app_config.h"
#include "Board_Config.h"
#include "app_radio.h"
#include "app_control.h"
#include "app_line.h"
#include "app_car_state.h"
#include "BeepLed.h"
#include "DebugSerial.h"
#include "Grayscale.h"
#include "Key.h"
#include "Motor.h"
#include "StepperMotor.h"
#include "Encoder.h"
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
static uint32_t s_txCount = 0U, s_hwAckCount = 0U, s_txFailCount = 0U;
static uint32_t s_pongCount = 0U, s_timeoutCount = 0U, s_unexpectedCount = 0U;
static uint32_t s_lastRttMs = 0U, s_pongStartMs = 0U;
static uint32_t s_pingRxCount = 0U, s_pongAckCount = 0U, s_pongFailCount = 0U;

void BoardTest_Init(void)
{
    App_Control_ForcePWMZero(); Motor_StopAll();
    App_Radio_ClearPendingCommands();
    s_testMs = 0U; s_pingTimer = 0U; s_pongTimer = 0U; s_statTimer = 0U;
    s_nextToken = 1U; s_pendingToken = 0U; s_waitPong = 0U;
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
    if (App_Radio_IsReady()) DebugSerial_SendString("[radio-test,radio=ready]\r\n");
    else DebugSerial_SendString("[radio-test,radio=not_ready]\r\n");
}

void BoardTest_Task10ms(void)
{
    AppRadioCommand_t cmd;
    App_Control_ForcePWMZero(); Motor_StopAll();
    s_testMs += 10U;
    if (!App_Radio_IsReady()) return;
    App_Radio_Task10ms();
    while (App_Radio_PopCommand(&cmd))
    {
#if CAR_ROLE_MASTER
        if (cmd.cmd == RADIO_CMD_PONG && cmd.room_id != 0U)
        { if (s_waitPong && cmd.room_id == s_pendingToken) { s_pongCount++; s_lastRttMs = s_testMs - s_pongStartMs; s_waitPong = 0U; DebugSerial_Printf("[radio-test,master,rx,pong,token=%u,rtt_ms=%u,ok]\r\n", (unsigned int)cmd.room_id, (unsigned int)s_lastRttMs); }
          else { s_unexpectedCount++; DebugSerial_Printf("[radio-test,master,rx,pong,token=%u,expected=%u,unexpected]\r\n", (unsigned int)cmd.room_id, (unsigned int)s_pendingToken); } }
        else DebugSerial_Printf("[radio-test,master,ignore,cmd=%u,value=%u]\r\n", (unsigned int)cmd.cmd, (unsigned int)cmd.room_id);
#elif CAR_ROLE_SLAVE
        if (cmd.cmd == RADIO_CMD_PING && cmd.room_id != 0U)
        { uint8_t t = cmd.room_id; s_pingRxCount++; DebugSerial_Printf("[radio-test,slave,rx,ping,token=%u]\r\n", (unsigned int)t);
          if (App_Radio_SendPong(t)) { s_pongAckCount++; DebugSerial_Printf("[radio-test,slave,tx,pong,token=%u,hw_ack=1]\r\n", (unsigned int)t); }
          else { s_pongFailCount++; DebugSerial_Printf("[radio-test,slave,tx,pong,token=%u,hw_ack=0]\r\n", (unsigned int)t); } }
        else DebugSerial_Printf("[radio-test,slave,ignore,cmd=%u,value=%u]\r\n", (unsigned int)cmd.cmd, (unsigned int)cmd.room_id);
#endif
    }
#if CAR_ROLE_MASTER
    s_pingTimer += 10U;
    if (s_waitPong) { s_pongTimer += 10U; if (s_pongTimer >= RADIO_TEST_PONG_TIMEOUT_MS) { s_timeoutCount++; s_waitPong = 0U; DebugSerial_Printf("[radio-test,master,timeout,token=%u]\r\n", (unsigned int)s_pendingToken); } }
    if (!s_waitPong && s_pingTimer >= RADIO_TEST_PING_PERIOD_MS)
    { uint8_t t = s_nextToken; s_pingTimer = 0U; s_nextToken++; if (s_nextToken == 0U) s_nextToken = 1U;
      s_txCount++; if (App_Radio_SendPing(t)) { s_hwAckCount++; s_pendingToken = t; s_waitPong = 1U; s_pongTimer = 0U; s_pongStartMs = s_testMs; DebugSerial_Printf("[radio-test,master,tx,ping,token=%u,hw_ack=1]\r\n", (unsigned int)t); }
      else { s_txFailCount++; DebugSerial_Printf("[radio-test,master,tx,ping,token=%u,hw_ack=0]\r\n", (unsigned int)t); } }
#endif
    s_statTimer += 10U;
    if (s_statTimer >= RADIO_TEST_STATS_PERIOD_MS)
    { s_statTimer = 0U;
#if CAR_ROLE_MASTER
      DebugSerial_Printf("[radio-test,master,stat,tx=%lu,hw_ack=%lu,pong=%lu,tx_fail=%lu,timeout=%lu,unexpected=%lu,last_rtt_ms=%u]\r\n",
        (unsigned long)s_txCount, (unsigned long)s_hwAckCount, (unsigned long)s_pongCount, (unsigned long)s_txFailCount, (unsigned long)s_timeoutCount, (unsigned long)s_unexpectedCount, (unsigned int)s_lastRttMs);
#elif CAR_ROLE_SLAVE
      DebugSerial_Printf("[radio-test,slave,stat,ping_rx=%lu,pong_ack=%lu,pong_fail=%lu]\r\n", (unsigned long)s_pingRxCount, (unsigned long)s_pongAckCount, (unsigned long)s_pongFailCount);
#endif
    }
}

void BoardTest_Task100ms(void) { }
void BoardTest_Task200ms(void) { }

#elif ECAR_TEST_STEPPER_ENABLE

/* Stepper board test with 500ms status output. */

typedef enum
{
    STEP_TEST_STOP = 0,
    STEP_TEST_RIGHT_ONLY,
    STEP_TEST_LEFT_ONLY,
    STEP_TEST_BOTH
} StepTestMode_t;

static const uint32_t s_stepFreqTable[STEPPER_SPEED_LEVELS] =
{
    2240U, 4533U, 6827U, 9120U, 11413U, 13707U, 16000U
};

static StepTestMode_t s_mode = STEP_TEST_STOP;
static uint8_t s_level = 0U;
static uint8_t s_keyPending = 0U;

static uint16_t s_periodMs = 0U;

static int32_t s_lastLeftEncTotal  = 0;
static int32_t s_lastRightEncTotal = 0;

static void BoardTest_PrintBanner(void)
{
    DebugSerial_SendString("\r\n[step-test,start]\r\n");
    DebugSerial_Printf("[step-test,step_l=PB15,timer=TIMG7_C0]\r\n");
    DebugSerial_Printf("[step-test,step_r=PB16,timer=TIMG8_C1]\r\n");
    DebugSerial_Printf("[step-test,dir_l=PB18]\r\n");
    DebugSerial_Printf("[step-test,dir_r=PB25]\r\n");
    DebugSerial_Printf("[step-test,enc_l_a=PB05,enc_l_b=PB12]\r\n");
    DebugSerial_Printf("[step-test,enc_r_a=PB08,enc_r_b=PB00]\r\n");
    DebugSerial_Printf("[step-test,en=hardware_always_enabled]\r\n");
    DebugSerial_Printf("[step-test,step_per_rev=%u]\r\n",
        (unsigned int)STEPPER_STEP_PER_REV);
    DebugSerial_Printf("[step-test,encoder_count_per_rev=%u]\r\n",
        (unsigned int)ENCODER_COUNT_PER_REV);
    DebugSerial_Printf("[step-test,level=%u,target_hz=%lu]\r\n",
        (unsigned int)(s_level + 1U),
        (unsigned long)s_stepFreqTable[s_level]);
}

void BoardTest_Init(void)
{
    s_mode = STEP_TEST_STOP;
    s_level = 0U;
    s_keyPending = 0U;
    s_periodMs = 0U;
    s_lastLeftEncTotal  = 0;
    s_lastRightEncTotal = 0;

    StepperMotor_StopAll();
    BoardTest_PrintBanner();
}

static void BoardTest_ApplyKey(void)
{
    uint32_t freq;
    const char *modeStr;

    freq = s_stepFreqTable[s_level];

    switch (s_keyPending)
    {
    case 1U:
        s_mode = STEP_TEST_RIGHT_ONLY;
        StepperMotor_StopLeft();
        StepperMotor_SetRightTargetFrequency(
            (int32_t)freq * (int32_t)RIGHT_STEPPER_DIR_SIGN);
        modeStr = "right_only";
        break;

    case 2U:
        s_mode = STEP_TEST_LEFT_ONLY;
        StepperMotor_StopRight();
        StepperMotor_SetLeftTargetFrequency(
            (int32_t)freq * (int32_t)LEFT_STEPPER_DIR_SIGN);
        modeStr = "left_only";
        break;

    case 3U:
        if (s_mode == STEP_TEST_BOTH)
        {
            s_mode = STEP_TEST_STOP;
            StepperMotor_StopAll();
            modeStr = "stop";
        }
        else
        {
            s_mode = STEP_TEST_BOTH;
            StepperMotor_SetTargetFrequency(
                (int32_t)freq * (int32_t)LEFT_STEPPER_DIR_SIGN,
                (int32_t)freq * (int32_t)RIGHT_STEPPER_DIR_SIGN);
            modeStr = "both";
        }
        break;

    case 4U:
        s_level++;
        if (s_level >= STEPPER_SPEED_LEVELS) { s_level = 0U; }
        freq = s_stepFreqTable[s_level];

        switch (s_mode)
        {
        case STEP_TEST_RIGHT_ONLY:
            StepperMotor_SetRightTargetFrequency(
                (int32_t)freq * (int32_t)RIGHT_STEPPER_DIR_SIGN);
            break;
        case STEP_TEST_LEFT_ONLY:
            StepperMotor_SetLeftTargetFrequency(
                (int32_t)freq * (int32_t)LEFT_STEPPER_DIR_SIGN);
            break;
        case STEP_TEST_BOTH:
            StepperMotor_SetTargetFrequency(
                (int32_t)freq * (int32_t)LEFT_STEPPER_DIR_SIGN,
                (int32_t)freq * (int32_t)RIGHT_STEPPER_DIR_SIGN);
            break;
        default:
            break;
        }

        DebugSerial_Printf(
            "[step-test,speed,level=%u,target_hz=%lu]\r\n",
            (unsigned int)(s_level + 1U), (unsigned long)freq);
        s_keyPending = 0U;
        return;

    default:
        s_keyPending = 0U;
        return;
    }

    DebugSerial_Printf(
        "[step-test,mode=%s,level=%u,target_hz=%lu]\r\n",
        modeStr, (unsigned int)(s_level + 1U), (unsigned long)freq);

    s_keyPending = 0U;
}

void BoardTest_Task10ms(void)
{
    uint8_t key;

    key = Key_GetNum();
    if (key >= 1U && key <= 4U)
    {
        s_keyPending = key;
        BoardTest_ApplyKey();
    }
}

void BoardTest_Task100ms(void)
{
    s_periodMs += 100U;
    if (s_periodMs < 500U) { return; }
    s_periodMs = 0U;

    {
        int32_t leftTgtFreq, rightTgtFreq;
        int32_t leftCurFreq, rightCurFreq;
        int32_t leftEncDelta, rightEncDelta;
        int32_t leftEncTotal, rightEncTotal;
        uint32_t leftStepCount, rightStepCount;
        int32_t leftEncExpected, rightEncExpected;
        int32_t leftEncError, rightEncError;

        leftTgtFreq  = StepperMotor_GetLeftTargetFrequency();
        rightTgtFreq = StepperMotor_GetRightTargetFrequency();
        leftCurFreq  = StepperMotor_GetLeftCurrentFrequency();
        rightCurFreq = StepperMotor_GetRightCurrentFrequency();

        leftStepCount  = StepperMotor_GetLeftStepCount();
        rightStepCount = StepperMotor_GetRightStepCount();

        /* Use encoder total diff to avoid double-consuming deltas
         * (App_Control_UpdateEncoderSpeed already calls Encoder_Get*Delta
         *  every 5ms via the main loop). */
        leftEncTotal  = g_leftEncoderTotal;
        rightEncTotal = g_rightEncoderTotal;
        leftEncDelta  = leftEncTotal  - s_lastLeftEncTotal;
        rightEncDelta = rightEncTotal - s_lastRightEncTotal;
        s_lastLeftEncTotal  = leftEncTotal;
        s_lastRightEncTotal = rightEncTotal;

        /* Expected encoder count = emitted_steps * 4096 / 3200.
         * Accumulate over the whole interval (not per-tick) so the
         * ratio is exact for the run-total step counter. */
        leftEncExpected = (int32_t)(
            (uint64_t)leftStepCount
            * (uint64_t)STEPPER_ENC_PER_STEP_NUM
            / (uint64_t)STEPPER_ENC_PER_STEP_DEN);
        rightEncExpected = (int32_t)(
            (uint64_t)rightStepCount
            * (uint64_t)STEPPER_ENC_PER_STEP_NUM
            / (uint64_t)STEPPER_ENC_PER_STEP_DEN);

        leftEncError  = leftEncTotal  - leftEncExpected;
        rightEncError = rightEncTotal - rightEncExpected;

        DebugSerial_Printf(
            "[step-test,status,"
            "mode=%u,level=%u,"
            "ltgt=%ld,lcur=%ld,rtgt=%ld,rcur=%ld,"
            "lstp=%lu,rstp=%lu,"
            "lenc=%ld,renc=%ld,ltot=%ld,rtot=%ld,"
            "lexp=%ld,rexp=%ld,lerr=%ld,rerr=%ld,"
            "lill=%lu,rill=%lu]\r\n",
            (unsigned int)s_mode, (unsigned int)(s_level + 1U),
            (long)leftTgtFreq, (long)leftCurFreq,
            (long)rightTgtFreq, (long)rightCurFreq,
            (unsigned long)leftStepCount, (unsigned long)rightStepCount,
            (long)leftEncDelta, (long)rightEncDelta,
            (long)leftEncTotal, (long)rightEncTotal,
            (long)leftEncExpected, (long)rightEncExpected,
            (long)leftEncError, (long)rightEncError,
            (unsigned long)Encoder_GetLeftIllegalCount(),
            (unsigned long)Encoder_GetRightIllegalCount());
    }
}

void BoardTest_Task200ms(void) { }

#else

#define BOARD_TEST_MOTOR_PWM 120

void BoardTest_Init(void)
{
    Motor_StopAll();
    DebugSerial_SendString("[board-test,start]\r\n");
    DebugSerial_SendString("[board-test,motor=enabled,pwm=120]\r\n");
    DebugSerial_SendString("[board-test,gray=enabled]\r\n");
    DebugSerial_SendString("[board-test,key,k1=left,k2=right,k3=both,k4=stop]\r\n");
}

static void BoardTest_PrintGray(void)
{
    uint8_t raw[8]; uint8_t i;
    Grayscale_ReadAll(raw);
    App_Line_Update();
    DebugSerial_Printf("[gray,ch=");
    for (i = 0U; i < 8U; i++) { DebugSerial_Printf("%u", (unsigned int)raw[i]); if (i < 7U) DebugSerial_SendByte(','); }
    DebugSerial_SendString("]\r\n");
    DebugSerial_Printf("[gray,raw=0x%02X,mask=0x%02X,valid=%u,error=%d,count=%u]\r\n",
        (unsigned int)g_lineRawMask, (unsigned int)g_lineMask, (unsigned int)g_lineValid, (int)g_lineError, (unsigned int)g_lineBlackCount);
}

void BoardTest_Task10ms(void)
{
    uint8_t key = Key_GetNum();
    if (key != 0U)
    {
        switch (key)
        {
        case 1U: Motor_SetPWM((int16_t)BOARD_TEST_MOTOR_PWM, 0); DebugSerial_SendString("[test,motor,left,pwm=120]\r\n"); break;
        case 2U: Motor_SetPWM(0, (int16_t)BOARD_TEST_MOTOR_PWM); DebugSerial_SendString("[test,motor,right,pwm=120]\r\n"); break;
        case 3U: Motor_SetPWM((int16_t)BOARD_TEST_MOTOR_PWM, (int16_t)BOARD_TEST_MOTOR_PWM); DebugSerial_SendString("[test,motor,both,pwm=120]\r\n"); break;
        case 4U: Motor_StopAll(); DebugSerial_SendString("[test,motor,stop]\r\n"); break;
        default: break;
        }
    }
}

void BoardTest_Task100ms(void) { LED_User_Off(); }
void BoardTest_Task200ms(void) { BoardTest_PrintGray(); }

#endif
