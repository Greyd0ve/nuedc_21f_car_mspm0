#include "app_board_test.h"
#include "app_config.h"
#include "Board_Config.h"
#include "app_radio.h"
#include "app_control.h"
#include "app_car_state.h"
#include "Encoder.h"
#include "RodEncoder.h"
#include "RodStepper.h"
#include "DebugSerial.h"
#include "Key.h"
#include "Motor.h"
#include "PWM.h"
#include "cmsis_compiler.h"
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
        if (cmd.cmd == RADIO_CMD_PONG && cmd.value != 0U)
        {
            if (s_waitPong && cmd.value == s_pendingToken)
            {
                s_pongCount++;
                s_lastRttMs = s_testMs - s_pongStartMs;
                s_waitPong = 0U;
                DebugSerial_Printf("[radio-test,master,rx,pong,token=%u,rtt_ms=%u,ok]\r\n",
                    (unsigned int)cmd.value, (unsigned int)s_lastRttMs);
            }
            else
            {
                s_unexpectedCount++;
                DebugSerial_Printf("[radio-test,master,rx,pong,token=%u,expected=%u,unexpected]\r\n",
                    (unsigned int)cmd.value, (unsigned int)s_pendingToken);
            }
        }
        else
        {
            DebugSerial_Printf("[radio-test,master,ignore,cmd=%u,value=%u]\r\n",
                (unsigned int)cmd.cmd, (unsigned int)cmd.value);
        }
#elif CAR_ROLE_SLAVE
        if (cmd.cmd == RADIO_CMD_PING && cmd.value != 0U)
        {
            uint8_t token = cmd.value;
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
                (unsigned int)cmd.cmd, (unsigned int)cmd.value);
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

#elif CAR_TEST_MOTOR_ENABLE

#define MOTOR_TEST_PRINT_PERIOD_100MS  10U

static uint8_t s_motorTestLeftEnable = 0U;
static uint8_t s_motorTestRightEnable = 0U;
static uint8_t s_motorTestGear = 0U;
static uint8_t s_motorTestPrintCount = 0U;

static uint8_t BoardTest_MotorGetPercent(void)
{
    static const uint8_t percentTable[3] = { 10U, 20U, 30U };

    if (s_motorTestGear >= (uint8_t)(sizeof(percentTable) / sizeof(percentTable[0])))
    {
        s_motorTestGear = 0U;
    }

    return percentTable[s_motorTestGear];
}

static int16_t BoardTest_MotorGetPwm(void)
{
    return (int16_t)(((int32_t)PWM_MAX_DUTY * (int32_t)BoardTest_MotorGetPercent()) / 100);
}

static void BoardTest_MotorApplyOutput(void)
{
    int16_t pwm = BoardTest_MotorGetPwm();
    int16_t leftPwm = (s_motorTestLeftEnable != 0U) ? pwm : 0;
    int16_t rightPwm = (s_motorTestRightEnable != 0U) ? pwm : 0;

    if ((s_motorTestLeftEnable == 0U) && (s_motorTestRightEnable == 0U))
    {
        Motor_StopAll();
        return;
    }

    Motor_SetPWM(leftPwm, rightPwm);
}

static void BoardTest_MotorPrintState(const char *event)
{
    DebugSerial_Printf(
        "[motor-test,event=%s,gear=%u,pwm=%d,left_en=%u,right_en=%u]\r\n",
        event,
        (unsigned int)(s_motorTestGear + 1U),
        (int)BoardTest_MotorGetPwm(),
        (unsigned int)s_motorTestLeftEnable,
        (unsigned int)s_motorTestRightEnable);
}

static void BoardTest_MotorPrintDiagnostic(void)
{
    int32_t leftSpeed10 = (int32_t)(g_leftSpeed * 10.0f);
    int32_t rightSpeed10 = (int32_t)(g_rightSpeed * 10.0f);

    DebugSerial_Printf(
        "[motor-test,gear=%u,pwm=%d,le=%u,re=%u,ld=%d,rd=%d,lt=%ld,rt=%ld,ls10=%ld,rs10=%ld]\r\n",
        (unsigned int)(s_motorTestGear + 1U),
        (int)BoardTest_MotorGetPwm(),
        (unsigned int)s_motorTestLeftEnable,
        (unsigned int)s_motorTestRightEnable,
        (int)g_leftEncoderDelta,
        (int)g_rightEncoderDelta,
        (long)g_leftEncoderTotal,
        (long)g_rightEncoderTotal,
        (long)leftSpeed10,
        (long)rightSpeed10);
}

static void BoardTest_MotorResetEncoderState(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    Encoder_ClearAll();

    g_leftEncoderDelta = 0;
    g_rightEncoderDelta = 0;
    g_leftEncoderTotal = 0;
    g_rightEncoderTotal = 0;
    g_forwardEncoderTotal = 0;
    g_turnEncoderTotal = 0;
    g_leftSpeed = 0.0f;
    g_rightSpeed = 0.0f;
    g_forwardSpeed = 0.0f;
    g_turnSpeed = 0.0f;

    g_carEnable = 0U;
    g_leftPwm = 0;
    g_rightPwm = 0;
    g_speedPwm = 0.0f;
    g_diffPwm = 0.0f;
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;

    if (primask == 0U)
    {
        __enable_irq();
    }
}

void BoardTest_Init(void)
{
    Motor_StopAll();

    s_motorTestLeftEnable = 0U;
    s_motorTestRightEnable = 0U;
    s_motorTestGear = 0U;
    s_motorTestPrintCount = 0U;
    BoardTest_MotorResetEncoderState();

    DebugSerial_SendString("[board-test,start]\r\n");
    DebugSerial_SendString("[board-test,mode=dc-motor-encoder]\r\n");
    DebugSerial_SendString("[motor-test,key,k1=right_toggle,k2=left_toggle,k3=both_run_or_stop,k4=gear]\r\n");
    DebugSerial_SendString("[motor-test,gear1=10,gear2=20,gear3=30]\r\n");
    DebugSerial_Printf("[motor-test,rear_mode=%u]\r\n", (unsigned int)ECAR_REAR_LINE_SENSOR_MODE);
    DebugSerial_SendString("[motor-test,safety=lift_wheels_before_run]\r\n");
}

void BoardTest_Task10ms(void)
{
    uint8_t key = Key_GetNum();
    const char *event = 0;

    switch (key)
    {
        case 1U:
            s_motorTestRightEnable ^= 1U;
            event = "k1";
            break;

        case 2U:
            s_motorTestLeftEnable ^= 1U;
            event = "k2";
            break;

        case 3U:
            if ((s_motorTestLeftEnable != 0U) || (s_motorTestRightEnable != 0U))
            {
                s_motorTestLeftEnable = 0U;
                s_motorTestRightEnable = 0U;
            }
            else
            {
                s_motorTestLeftEnable = 1U;
                s_motorTestRightEnable = 1U;
            }
            event = "k3";
            break;

        case 4U:
            s_motorTestGear++;
            if (s_motorTestGear >= 3U)
            {
                s_motorTestGear = 0U;
            }
            event = "k4";
            break;

        default:
            break;
    }

    BoardTest_MotorApplyOutput();

    if (event != 0)
    {
        BoardTest_MotorPrintState(event);
    }
}

void BoardTest_Task100ms(void)
{
    s_motorTestPrintCount++;
    if (s_motorTestPrintCount >= MOTOR_TEST_PRINT_PERIOD_100MS)
    {
        s_motorTestPrintCount = 0U;
        BoardTest_MotorPrintDiagnostic();
    }
}

void BoardTest_Task200ms(void) { }

#elif CAR_TEST_SPEED_PID_ENABLE

#define SPEED_PID_TEST_DEFAULT_TARGET_X10   200
#define SPEED_PID_TEST_MAX_TARGET_X10       800

static uint8_t s_speedPidTestRun = 0U;
static int32_t s_speedPidTestTargetX10 = SPEED_PID_TEST_DEFAULT_TARGET_X10;

static int32_t BoardTest_SpeedPidFloatToX10(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value * 10.0f + 0.5f);
    }

    return (int32_t)(value * 10.0f - 0.5f);
}

static int32_t BoardTest_SpeedPidFloatToX100(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value * 100.0f + 0.5f);
    }

    return (int32_t)(value * 100.0f - 0.5f);
}

static uint8_t BoardTest_SpeedPidLineIs(const char *line, const char *expected)
{
    while (*line != '\0' && *expected != '\0')
    {
        if (*line != *expected)
        {
            return 0U;
        }
        line++;
        expected++;
    }

    return (*line == '\0' && *expected == '\0') ? 1U : 0U;
}

static uint8_t BoardTest_SpeedPidParseValue(const char *line,
                                             const char *prefix,
                                             int32_t *value)
{
    uint32_t parsed = 0U;
    uint8_t digitCount = 0U;

    while (*prefix != '\0')
    {
        if (*line != *prefix)
        {
            return 0U;
        }
        line++;
        prefix++;
    }

    while (*line >= '0' && *line <= '9')
    {
        if (parsed > 100000000U)
        {
            return 0U;
        }
        parsed = parsed * 10U + (uint32_t)(*line - '0');
        digitCount++;
        line++;
    }

    if ((digitCount == 0U) || (*line != ']'))
    {
        return 0U;
    }

    *value = (int32_t)parsed;
    return 1U;
}

static void BoardTest_SpeedPidPrintStatus(void)
{
    float kp;
    float ki;
    float kd;

    App_Control_GetSpeedPIDParam(&kp, &ki, &kd);
    DebugSerial_Printf(
        "[pid-test,status,run=%u,target10=%ld,kp100=%ld,ki100=%ld,kd100=%ld]\r\n",
        (unsigned int)s_speedPidTestRun,
        (long)s_speedPidTestTargetX10,
        (long)BoardTest_SpeedPidFloatToX100(kp),
        (long)BoardTest_SpeedPidFloatToX100(ki),
        (long)BoardTest_SpeedPidFloatToX100(kd));
}

static void BoardTest_SpeedPidStop(void)
{
    s_speedPidTestRun = 0U;
    App_Control_ForcePWMZero();
    DebugSerial_SendString("[pid-test,run=0]\r\n");
}

static void BoardTest_SpeedPidStart(void)
{
    if (s_speedPidTestTargetX10 <= 0)
    {
        DebugSerial_SendString("[pid-test,run=rejected,target10=0]\r\n");
        return;
    }

    App_Control_ResetPID();
    App_Control_SetLowSpeedTrackSafety(0U);
    s_speedPidTestRun = 1U;
    DebugSerial_SendString("[pid-test,run=1]\r\n");
}

static void BoardTest_SpeedPidSetTarget(int32_t targetX10)
{
    if ((targetX10 <= 0) || (targetX10 > SPEED_PID_TEST_MAX_TARGET_X10))
    {
        DebugSerial_Printf("[pid-test,target=rejected,value10=%ld]\r\n", (long)targetX10);
        return;
    }

    s_speedPidTestTargetX10 = targetX10;
    App_Control_ResetPID();
    DebugSerial_Printf("[pid-test,target10=%ld]\r\n", (long)s_speedPidTestTargetX10);
}

static void BoardTest_SpeedPidSetGain(uint8_t gainIndex, int32_t gainX100)
{
    float kp;
    float ki;
    float kd;

    if (gainX100 > 2000)
    {
        DebugSerial_Printf("[pid-test,gain=rejected,value100=%ld]\r\n", (long)gainX100);
        return;
    }

    App_Control_GetSpeedPIDParam(&kp, &ki, &kd);
    if (gainIndex == 0U) kp = (float)gainX100 / 100.0f;
    else if (gainIndex == 1U) ki = (float)gainX100 / 100.0f;
    else kd = (float)gainX100 / 100.0f;

    App_Control_SetSpeedPIDParam(kp, ki, kd);
    App_Control_ResetPID();
    BoardTest_SpeedPidPrintStatus();
}

static void BoardTest_SpeedPidHandleCommand(const char *line)
{
    int32_t value;

    if (BoardTest_SpeedPidLineIs(line, "[pid,run]"))
    {
        BoardTest_SpeedPidStart();
    }
    else if (BoardTest_SpeedPidLineIs(line, "[pid,stop]"))
    {
        BoardTest_SpeedPidStop();
    }
    else if (BoardTest_SpeedPidLineIs(line, "[pid,status]"))
    {
        BoardTest_SpeedPidPrintStatus();
    }
    else if (BoardTest_SpeedPidParseValue(line, "[pid,target,", &value))
    {
        BoardTest_SpeedPidSetTarget(value);
    }
    else if (BoardTest_SpeedPidParseValue(line, "[pid,kp,", &value))
    {
        BoardTest_SpeedPidSetGain(0U, value);
    }
    else if (BoardTest_SpeedPidParseValue(line, "[pid,ki,", &value))
    {
        BoardTest_SpeedPidSetGain(1U, value);
    }
    else if (BoardTest_SpeedPidParseValue(line, "[pid,kd,", &value))
    {
        BoardTest_SpeedPidSetGain(2U, value);
    }
    else if (!BoardTest_SpeedPidLineIs(line, "[ping]"))
    {
        DebugSerial_SendString("[pid-test,cmd=invalid]\r\n");
    }
}

void BoardTest_Init(void)
{
    App_Control_Init();
    App_Control_ForcePWMZero();
    App_Control_SetLowSpeedTrackSafety(0U);

    s_speedPidTestRun = 0U;
    s_speedPidTestTargetX10 = SPEED_PID_TEST_DEFAULT_TARGET_X10;
    DebugSerial_SetLineHandler(BoardTest_SpeedPidHandleCommand);

    DebugSerial_SendString("[board-test,start]\r\n");
    DebugSerial_SendString("[board-test,mode=speed-pid]\r\n");
    DebugSerial_SendString("[pid-test,safety=lift_wheels_before_run]\r\n");
    DebugSerial_SendString("[pid-test,plot=[p,target10,actual10,error10],unit=0.1cmps,period_ms=100]\r\n");
    DebugSerial_SendString("[pid-test,cmd=[pid,run]|[pid,stop]|[pid,status]\r\n");
    DebugSerial_SendString("[pid-test,cmd=[pid,target,200]|[pid,kp,150]|[pid,ki,8]|[pid,kd,0]\r\n");
    DebugSerial_SendString("[pid-test,key,k1=run_or_stop,k3=stop]\r\n");
    BoardTest_SpeedPidPrintStatus();
}

void BoardTest_Task10ms(void)
{
    uint8_t key = Key_GetNum();

    if (key == 1U)
    {
        if (s_speedPidTestRun != 0U) BoardTest_SpeedPidStop();
        else BoardTest_SpeedPidStart();
    }
    else if (key == 3U)
    {
        BoardTest_SpeedPidStop();
    }

    if (s_speedPidTestRun != 0U)
    {
        g_targetForwardSpeed = (float)s_speedPidTestTargetX10 / 10.0f;
        g_targetTurnSpeed = 0.0f;
        g_carEnable = 1U;
        App_Control_ApplyMotorOutput();
    }
}

void BoardTest_Task100ms(void)
{
    int32_t actualX10;
    int32_t errorX10;

    if (s_speedPidTestRun == 0U)
    {
        return;
    }

    actualX10 = BoardTest_SpeedPidFloatToX10(g_forwardSpeed);
    errorX10 = BoardTest_SpeedPidFloatToX10(g_targetForwardSpeed - g_forwardSpeed);
    DebugSerial_Printf("[p,%ld,%ld,%ld]\r\n",
        (long)s_speedPidTestTargetX10,
        (long)actualX10,
        (long)errorX10);
}

void BoardTest_Task200ms(void) { }

#elif CAR_TEST_ROD_STEPPER_ENABLE

/* 3200 STEP pulses/rev: 640 pulses equals one 72-degree jog. */
#define ROD_TEST_JOG_PULSES            640U
#define ROD_TEST_STEP_FREQ_HZ          400U

typedef enum
{
    ROD_TEST_IDLE = 0,
    ROD_TEST_MOVING
} RodStepperTestState_t;

static RodStepperTestState_t s_rodTestState = ROD_TEST_IDLE;
static int32_t s_commandPulse = 0;
static uint8_t s_statDivider = 0U;

static void BoardTest_RodStartMove(RodStepperDirection_t direction)
{
    (void)RodStepper_MovePulses(direction, ROD_TEST_JOG_PULSES,
                                 ROD_TEST_STEP_FREQ_HZ);
    s_commandPulse = (direction == ROD_STEPPER_DIR_POSITIVE) ?
        (int32_t)ROD_TEST_JOG_PULSES : -(int32_t)ROD_TEST_JOG_PULSES;
    s_rodTestState = ROD_TEST_MOVING;

    DebugSerial_Printf("[rod-test,move,dir=%d,pulses=%u,freq=%u]\r\n",
        (int)direction, (unsigned int)ROD_TEST_JOG_PULSES,
        (unsigned int)ROD_TEST_STEP_FREQ_HZ);
}

void BoardTest_Init(void)
{
    App_Control_ForcePWMZero();
    Motor_StopAll();

    RodStepper_Init();
    RodEncoder_Init();
    RodEncoder_Reset();
    s_rodTestState = ROD_TEST_IDLE;
    s_statDivider = 0U;

    DebugSerial_SendString("[rod-test,start]\r\n");
    DebugSerial_SendString("[rod-test,pin,step=PA02/TIMG7_C1,dir=PB25,en=PB21,enc_a=PB00,enc_b=PB18]\r\n");
    DebugSerial_SendString("[rod-test,en=active-low,constant-enabled]\r\n");
    DebugSerial_Printf("[rod-test,cpr=%u,step_per_rev=%u]\r\n",
        (unsigned int)ECAR_STEPPER_ENCODER_CPR,
        (unsigned int)ECAR_STEPPER_DRIVER_PULSE_PER_REV);
    DebugSerial_SendString("[rod-test,key,k1=positive-jog,k2=negative-jog,k4=zero]\r\n");
}

void BoardTest_Task10ms(void)
{
    uint8_t key = Key_GetNum();
    RodEncoderSnapshot_t snapshot;

    App_Control_ForcePWMZero();
    Motor_StopAll();

    if (key == 4U)
    {
        RodEncoder_Reset();
        DebugSerial_SendString("[rod-test,zero]\r\n");
        return;
    }

    if (key == 1U)
    {
        BoardTest_RodStartMove(ROD_STEPPER_DIR_POSITIVE);
        return;
    }
    if (key == 2U)
    {
        BoardTest_RodStartMove(ROD_STEPPER_DIR_NEGATIVE);
        return;
    }

    if (s_rodTestState == ROD_TEST_MOVING)
    {
        if (RodStepper_TakeCompletionEvent() != 0U)
        {
            RodEncoder_GetSnapshot(&snapshot);
            DebugSerial_Printf("[rod-test,done,cmd_pulse=%ld,count=%ld,bad=%lu]\r\n",
                (long)s_commandPulse, (long)snapshot.count,
                (unsigned long)snapshot.badTransitionCount);
            s_rodTestState = ROD_TEST_IDLE;
        }
    }
}

void BoardTest_Task100ms(void)
{
    RodEncoderSnapshot_t snapshot;

    s_statDivider++;
    if (s_statDivider < 5U)
    {
        return;
    }
    s_statDivider = 0U;

    RodEncoder_GetSnapshot(&snapshot);
    DebugSerial_Printf("[rod-test,stat,state=%u,en=%u,busy=%u,dir=%d,remaining=%lu,cmd_total=%ld,done_total=%lu,count=%ld,bad=%lu]\r\n",
        (unsigned int)s_rodTestState, (unsigned int)RodStepper_IsEnabled(),
        (unsigned int)RodStepper_IsBusy(), (int)RodStepper_GetDirection(),
        (unsigned long)RodStepper_GetRemainingPulses(),
        (long)RodStepper_GetSignedCommandPulseTotal(),
        (unsigned long)RodStepper_GetCompletedPulseCount(),
        (long)snapshot.count, (unsigned long)snapshot.badTransitionCount);
}

void BoardTest_Task200ms(void) { }

#elif CAR_TEST_STEPPER_ENCODER_ENABLE

/* Historical dual-axis module is retained on disk but no longer compiled. */
void BoardTest_Init(void) { Motor_StopAll(); DebugSerial_SendString("[board-test,mode=stepper-encoder-removed]\r\n"); }
void BoardTest_Task10ms(void) { Motor_StopAll(); }
void BoardTest_Task100ms(void) { }
void BoardTest_Task200ms(void) { }

#elif CAR_TEST_JY61P_ENABLE

#include "OLED.h"
#include "JY61P.h"
#include "JY61P_Serial.h"
#include "Servo.h"
#include "Timer.h"

#define JY61P_PRINT_PERIOD_MS  1000U
#define JY61P_RAW_PRINT_PERIOD_MS 2000U
#define JY61P_PAGE_ANGLE       0U
#define JY61P_PAGE_DIAGNOSTIC  1U
#define JY61P_PAGE_RAW         2U
#define JY61P_PAGE_MAX         3U

static uint32_t s_jy61pLastPrintMs = 0U;
static uint32_t s_jy61pLastRawPrintMs = 0U;
static uint8_t s_jy61pPrintPaused = 0U;
static uint8_t s_jy61pPage = 0U;
static uint8_t s_jy61pPrevOnline = 0U;
static uint8_t s_jy61pEverOnline = 0U;
static uint32_t s_jy61pBootCount = 0U;

#if JY61P_AUTO_ZERO_ON_FIRST_VALID
static uint8_t s_jy61pAutoZeroDone = 0U;
#endif

static void BoardTest_FormatAge(char *buf, uint32_t age)
{
    char reverse[10];
    uint8_t len = 0U;
    uint8_t pos = 0U;

    if (age == JY61P_AGE_UNKNOWN_MS)
    {
        buf[0] = 'n';
        buf[1] = 'a';
        buf[2] = '\0';
        return;
    }
    do
    {
        reverse[len++] = (char)('0' + (age % 10U));
        age /= 10U;
    } while ((age != 0U) && (len < sizeof(reverse)));
    while (len > 0U) { buf[pos++] = reverse[--len]; }
    buf[pos] = '\0';
}

#if CAR_OLED_ENABLE
static void BoardTest_FormatAngleX100(char *buf, int16_t value)
{
    int32_t magnitude = (int32_t)value;
    uint32_t degrees;
    uint32_t fraction;
    uint8_t pos = 0U;
    if (magnitude < 0) { buf[pos++] = '-'; magnitude = -magnitude; }
    degrees = (uint32_t)magnitude / 100U;
    fraction = (uint32_t)magnitude % 100U;
    if (degrees >= 100U) { buf[pos++] = (char)('0' + (degrees / 100U)); buf[pos++] = (char)('0' + ((degrees / 10U) % 10U)); }
    else if (degrees >= 10U) { buf[pos++] = (char)('0' + (degrees / 10U)); }
    buf[pos++] = (char)('0' + (degrees % 10U)); buf[pos++] = '.';
    buf[pos++] = (char)('0' + (fraction / 10U)); buf[pos++] = (char)('0' + (fraction % 10U));
    buf[pos] = '\0';
}
static void BoardTest_ShowAngleRow(uint8_t row, const char *label, int16_t value, uint8_t valid)
{
    char buf[8];
    OLED_ShowString(0, row, label, OLED_6X8);
    if (valid) { BoardTest_FormatAngleX100(buf, value); OLED_ShowString(18, row, buf, OLED_6X8); }
    else { OLED_ShowString(18, row, "--", OLED_6X8); }
}
#endif

static void BoardTest_SendHexByte(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    DebugSerial_SendByte((uint8_t)hex[(value >> 4U) & 0x0FU]);
    DebugSerial_SendByte((uint8_t)hex[value & 0x0FU]);
}

static const char *BoardTest_GetDiagnosis(const JY61P_Data_t *jdata,
                                          uint32_t irqCount,
                                          uint32_t rxByteCount)
{
    if (irqCount == 0U && rxByteCount == 0U) return "no_uart_irq";
    if (irqCount > 0U && rxByteCount == 0U) return "irq_without_rx_data";

    if (jdata->online == 0U)
    {
        if (jdata->link_age_ms == JY61P_AGE_UNKNOWN_MS)
            return "no_valid_frame";
        return "offline";
    }

    if (jdata->angle_valid == 0U && jdata->gyro_valid == 0U)
        return "frames_stale";

    if (jdata->angle_valid != 0U || jdata->gyro_valid != 0U)
        return "receiving";

    if (jdata->sync_error_count > 0U &&
        jdata->angle_frame_count == 0U &&
        jdata->gyro_frame_count == 0U)
        return "baud_or_protocol_mismatch";

    if (jdata->checksum_error_count > 0U &&
        jdata->angle_frame_count == 0U &&
        jdata->gyro_frame_count == 0U)
        return "checksum_fail";

    if (jdata->unsupported_frame_count > 0U &&
        jdata->angle_frame_count == 0U &&
        jdata->gyro_frame_count == 0U)
        return "unsupported_frames_only";

    return "unknown";
}

void BoardTest_Init(void)
{
    uint32_t resetCause;
    App_Control_ForcePWMZero();
    Motor_StopAll();
    Servo_DisableAll();

    s_jy61pLastPrintMs = Timer_GetMillis();
    s_jy61pLastRawPrintMs = s_jy61pLastPrintMs;
    s_jy61pPrintPaused = 0U;
    s_jy61pPage = 0U;
    s_jy61pPrevOnline = 0U;
    s_jy61pEverOnline = 0U;
    s_jy61pBootCount++;
#if JY61P_AUTO_ZERO_ON_FIRST_VALID
    s_jy61pAutoZeroDone = 0U;
#endif

    resetCause = (uint32_t)DL_SYSCTL_getResetCause();
    DebugSerial_Printf("[fw,jy61p-test,rev=4,boot=%lu]\r\n", (unsigned long)s_jy61pBootCount);
    DebugSerial_Printf("[reset,cause=0x%08lX]\r\n", (unsigned long)resetCause);
    DebugSerial_SendString("[board-test,mode=jy61p]\r\n");
    DebugSerial_SendString("[jy61p,uart2,tx=pa23,rx=pa24,baud=9600]\r\n");
    DebugSerial_SendString("[jy61p,key,k1=yaw_zero,k2=print_pause,k3=page,k4=clear_stats]\r\n");
    DebugSerial_SendString("[jy61p,page,0=angle,1=diag,2=raw]\r\n");
    JY61P_Init();
    DebugSerial_SendString("[jy61p,selfcheck,inst=uart2,tx=pa23,rx=pa24,baud=9600]\r\n");
    DebugSerial_Printf("[jy61p,selfcheck,irqn=uart2_int,raw=%lu]\r\n", (unsigned long)(uint32_t)UART2_INT_IRQn);
    DebugSerial_Printf("[jy61p,selfcheck,rx_fifo_discard=%lu]\r\n", (unsigned long)JY61P_Serial_GetInitDiscardCount());
    DebugSerial_SendString("[jy61p,selfcheck,serial_ready=1]\r\n");
}

void BoardTest_Task10ms(void)
{
    uint8_t key;
    JY61P_Data_t jdata;
    uint8_t online;
    App_Control_ForcePWMZero();
    Motor_StopAll();
    Servo_DisableAll();
    JY61P_Task10ms();
    JY61P_GetData(&jdata);
    online = JY61P_IsOnline();

#if JY61P_AUTO_ZERO_ON_FIRST_VALID
    if (!s_jy61pAutoZeroDone && online && jdata.angle_valid)
    {
        if (JY61P_ResetRelativeYaw())
        {
            JY61P_GetData(&jdata);
            DebugSerial_Printf("[jy61p,yaw_zero,auto,yaw=%d,offset=%d]\r\n", (int)jdata.yaw_x100, (int)jdata.yaw_zero_offset_x100);
        }
        s_jy61pAutoZeroDone = 1U;
    }
#endif

    if (online && !s_jy61pPrevOnline)
    {
        if (s_jy61pEverOnline)
            DebugSerial_SendString("[jy61p,status,online,recovered]\r\n");
        else { DebugSerial_SendString("[jy61p,status,online]\r\n"); s_jy61pEverOnline = 1U; }
    }
    else if (!online && s_jy61pPrevOnline)
    {
        if (jdata.link_age_ms == JY61P_AGE_UNKNOWN_MS)
            DebugSerial_SendString("[jy61p,status,offline,age=na]\r\n");
        else
            DebugSerial_Printf("[jy61p,status,offline,age=%lu]\r\n", (unsigned long)jdata.link_age_ms);
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
                JY61P_GetData(&zdata);
                DebugSerial_Printf("[jy61p,yaw_zero,ok,yaw=%d,offset=%d]\r\n", (int)zdata.yaw_x100, (int)zdata.yaw_zero_offset_x100);
            }
            else
            {
                if (zdata.online == 0U) DebugSerial_SendString("[jy61p,yaw_zero,rejected,offline]\r\n");
                else if (zdata.angle_valid == 0U) DebugSerial_SendString("[jy61p,yaw_zero,rejected,angle_stale]\r\n");
                else DebugSerial_SendString("[jy61p,yaw_zero,rejected,no_valid_angle]\r\n");
            }
            break;
        }
        case 2U:
            s_jy61pPrintPaused ^= 1U;
            DebugSerial_SendString(s_jy61pPrintPaused ? "[jy61p,print,paused]\r\n" : "[jy61p,print,resumed]\r\n");
            break;
        case 3U:
            s_jy61pPage++;
            if (s_jy61pPage >= JY61P_PAGE_MAX) s_jy61pPage = 0U;
            if (s_jy61pPage == JY61P_PAGE_ANGLE) DebugSerial_SendString("[jy61p,page=angle]\r\n");
            else if (s_jy61pPage == JY61P_PAGE_DIAGNOSTIC) DebugSerial_SendString("[jy61p,page=diagnostic]\r\n");
            else DebugSerial_SendString("[jy61p,page=raw]\r\n");
            break;
        case 4U:
            JY61P_ClearStatistics();
            DebugSerial_SendString("[jy61p,stats,cleared]\r\n");
            break;
        default: break;
    }
}

void BoardTest_Task100ms(void)
{
    JY61P_Data_t jdata;
    uint32_t now = Timer_GetMillis();
    char angleAge[11], gyroAge[11], linkAge[11];
    if ((uint32_t)(now - s_jy61pLastPrintMs) < JY61P_PRINT_PERIOD_MS) return;
    s_jy61pLastPrintMs = now;
    if (s_jy61pPrintPaused) return;
    JY61P_GetData(&jdata);
    BoardTest_FormatAge(angleAge, jdata.angle_age_ms);
    BoardTest_FormatAge(gyroAge, jdata.gyro_age_ms);
    BoardTest_FormatAge(linkAge, jdata.link_age_ms);

    if (s_jy61pPage == JY61P_PAGE_ANGLE)
    {
        if (jdata.angle_valid == 0U)
        {
            DebugSerial_Printf("[jy61p,on=%u,angle=stale,aa=%s,ga=%s,z=%u,zo=%d]\r\n",
                (unsigned int)jdata.online, angleAge, gyroAge, (unsigned int)jdata.yaw_zero_valid, (int)jdata.yaw_zero_offset_x100);
        }
        else if (jdata.gyro_valid == 0U)
        {
            DebugSerial_Printf("[jy61p,on=%u,r=%d,p=%d,y=%d,rel=%d,gz=stale,aa=%s,ga=%s,z=%u,zo=%d]\r\n",
                (unsigned int)jdata.online, (int)jdata.roll_x100, (int)jdata.pitch_x100,
                (int)jdata.yaw_x100, (int)jdata.relative_yaw_x100, angleAge, gyroAge,
                (unsigned int)jdata.yaw_zero_valid, (int)jdata.yaw_zero_offset_x100);
        }
        else
        {
            DebugSerial_Printf("[jy61p,on=%u,r=%d,p=%d,y=%d,rel=%d,gz=%d,aa=%s,ga=%s,z=%u,zo=%d]\r\n",
                (unsigned int)jdata.online, (int)jdata.roll_x100, (int)jdata.pitch_x100,
                (int)jdata.yaw_x100, (int)jdata.relative_yaw_x100, (int)jdata.gyro_z_dps_x10,
                angleAge, gyroAge, (unsigned int)jdata.yaw_zero_valid, (int)jdata.yaw_zero_offset_x100);
        }
    }
    else if (s_jy61pPage == JY61P_PAGE_DIAGNOSTIC)
    {
        uint32_t irqCount = JY61P_Serial_GetIrqCount();
        uint32_t rxIrqCount = JY61P_Serial_GetRxIrqCount();
        uint32_t otherIrqCount = JY61P_Serial_GetOtherIrqCount();
        uint32_t lastIidx = JY61P_Serial_GetLastInterruptIndex();
        uint32_t rxByteCount = JY61P_Serial_GetRxByteCount();
        uint32_t initDiscard = JY61P_Serial_GetInitDiscardCount();
        uint32_t ovfCount = jdata.rx_overflow_count;
        const char *diag = BoardTest_GetDiagnosis(&jdata, irqCount, rxByteCount);

        DebugSerial_Printf("[jy61p,af=%lu,gf=%lu,ign=%lu,cs=%lu,sync=%lu,age=%s]\r\n",
            (unsigned long)jdata.angle_frame_count, (unsigned long)jdata.gyro_frame_count,
            (unsigned long)jdata.unsupported_frame_count, (unsigned long)jdata.checksum_error_count,
            (unsigned long)jdata.sync_error_count, linkAge);
        DebugSerial_Printf("[uart2,irq=%lu,rxirq=%lu,other=%lu,last=%lu,bytes=%lu,discard=%lu,ovf=%lu,pend=%u,max=%u]\r\n",
            (unsigned long)irqCount, (unsigned long)rxIrqCount, (unsigned long)otherIrqCount,
            (unsigned long)lastIidx, (unsigned long)rxByteCount, (unsigned long)initDiscard,
            (unsigned long)ovfCount, (unsigned int)JY61P_Serial_GetRxPendingCount(),
            (unsigned int)JY61P_Serial_GetRxHighWaterMark());
        DebugSerial_Printf("[jy61p,diagnosis=%s]\r\n", diag);
    }
    else
    {
        uint32_t irqCount = JY61P_Serial_GetIrqCount();
        uint32_t rxByteCount = JY61P_Serial_GetRxByteCount();
        const char *diag = BoardTest_GetDiagnosis(&jdata, irqCount, rxByteCount);
        DebugSerial_Printf("[jy61p,on=%u,diag=%s,raw=(see_next)]\r\n", (unsigned int)jdata.online, diag);
    }
}

void BoardTest_Task200ms(void)
{
#if CAR_OLED_ENABLE
    JY61P_Data_t odata;
    JY61P_GetData(&odata);
    OLED_Clear();
    if (odata.online == 0U) OLED_ShowString(0, 0, "JY61P OFF", OLED_6X8);
    else if (odata.angle_valid == 0U) OLED_ShowString(0, 0, "ANGLE STALE", OLED_6X8);
    else OLED_ShowString(0, 0, "JY61P OK", OLED_6X8);

    if (s_jy61pPage == JY61P_PAGE_ANGLE)
    {
        BoardTest_ShowAngleRow(2, "R:", odata.roll_x100, odata.angle_valid);
        BoardTest_ShowAngleRow(4, "P:", odata.pitch_x100, odata.angle_valid);
        BoardTest_ShowAngleRow(6, "Y:", odata.relative_yaw_x100, odata.angle_valid);
    }
    else
    {
        OLED_ShowString(0, 2, "A/G:", OLED_6X8); OLED_ShowNum(30, 2, odata.angle_frame_count, 4, OLED_6X8);
        OLED_ShowString(60, 2, "/", OLED_6X8); OLED_ShowNum(66, 2, odata.gyro_frame_count, 4, OLED_6X8);
        OLED_ShowString(0, 4, "ERR:", OLED_6X8);
        OLED_ShowNum(30, 4, odata.checksum_error_count + odata.sync_error_count, 4, OLED_6X8);
        OLED_ShowString(0, 6, "AGE:", OLED_6X8);
        if (odata.link_age_ms == JY61P_AGE_UNKNOWN_MS) OLED_ShowString(30, 6, "----", OLED_6X8);
        else OLED_ShowNum(30, 6, odata.link_age_ms, 4, OLED_6X8);
    }
#endif
    if (s_jy61pPage == JY61P_PAGE_RAW && !s_jy61pPrintPaused)
    {
        uint32_t now = Timer_GetMillis();
        if ((uint32_t)(now - s_jy61pLastRawPrintMs) >= JY61P_RAW_PRINT_PERIOD_MS)
        {
            uint8_t rawBuf[JY61P_RAW_TRACE_SIZE];
            uint8_t rawCount = 0U;
            uint8_t i;
            s_jy61pLastRawPrintMs = now;
            JY61P_GetRawTrace(rawBuf, JY61P_RAW_TRACE_SIZE, &rawCount);
            if (rawCount > 0U)
            {
                DebugSerial_SendString("[raw");
                for (i = 0U; i < rawCount; i++) { DebugSerial_SendByte(','); BoardTest_SendHexByte(rawBuf[i]); }
                DebugSerial_SendString("]\r\n");
            }
            else { DebugSerial_SendString("[raw,<empty>]\r\n"); }
        }
    }
}

#else

void BoardTest_Init(void) { Motor_StopAll(); DebugSerial_SendString("[board-test,start]\r\n"); DebugSerial_SendString("[board-test,mode=none]\r\n"); }
void BoardTest_Task10ms(void) { Motor_StopAll(); }
void BoardTest_Task100ms(void) { }
void BoardTest_Task200ms(void) { }

#endif
