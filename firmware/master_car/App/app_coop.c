#include "app_coop.h"
#include "app_21f_car.h"
#include "app_config.h"
#include "app_radio.h"
#include "app_car_state.h"
#include "app_control.h"
#include "app_line.h"
#include "BeepLed.h"
#include "DebugSerial.h"
#include "Encoder.h"
#include "Motor.h"
#include "Serial.h"
#include "cmsis_compiler.h"
#include <stdint.h>

#define F21_COOP_LOAD_DELAY_MS              500U
#define F21_COOP_FINISH_QUIET_MS            800U
#define F21_COOP_VISION_BUF_SIZE            32U
#define F21_COOP_CROSS_BLACK_THRESH         8U
#define F21_COOP_CROSS_TO_TARGET_ADVANCE_CM F21_CROSS_ADVANCE_CM
#define F21_COOP_ROOM_RUN_CM                35.0f
#define F21_COOP_MID_DETECT_CM              120.0f
#define F21_COOP_RETURN_DETECT_CM           20.0f
#define F21_COOP_RETURN_HOME_CM             135.0f
#define F21_COOP_RADIO_RETRY_MS             100U

#define F21_COOP_CROSS_CONFIRM_TICKS \
    ((F21_CROSS_CONFIRM_MS + CAR_CONTROL_PERIOD_MS - 1U) / CAR_CONTROL_PERIOD_MS)

static volatile uint8_t s_targetRoom = 3U;
static volatile uint8_t s_waitRoom = 4U;
static volatile uint8_t s_faultCode = 0U;

static volatile uint32_t s_stateMs = 0U;
static volatile int32_t s_stateStartPulse = 0;
static volatile int32_t s_crossPulse = 0;
static volatile int32_t s_turnStartPulse = 0;
static volatile uint8_t s_crossMonitoring = 0U;
static volatile uint8_t s_crossConfirmCnt = 0U;

static volatile int32_t s_detectStartPulse = 0;
static volatile int32_t s_roomRunPulse = 0;
static volatile F21TurnDir_t s_outboundTurnDir = F21_TURN_LEFT;
static volatile int32_t s_returnDetectStartPulse = 0;
static volatile int32_t s_returnFinalRunPulse = 0;
static volatile F21TurnDir_t s_returnTurnDir = F21_TURN_RIGHT;

#if CAR_ROLE_MASTER
static volatile uint8_t s_targetSent = 0U;
static volatile uint8_t s_slaveStartSent = 0U;
static volatile uint8_t s_slaveReleaseSent = 0U;
static volatile uint8_t s_slaveAtWaitReceived = 0U;
static volatile uint8_t s_waitSlaveLogged = 0U;
static volatile uint8_t s_finishUnlockSent = 0U;
static volatile int32_t s_returnHomeRemainPulse = 0;
#endif

static volatile uint8_t s_yellowWaitActive = 0U;
static volatile uint32_t s_radioRetryMs = F21_COOP_RADIO_RETRY_MS;
static volatile uint8_t s_radioRetryFailCount = 0U;

#if CAR_ROLE_MASTER
static volatile F21CoopMasterState_t s_masterState = F21_COOP_MASTER_IDLE;
#if ENABLE_K230
static char s_visionRxBuf[F21_COOP_VISION_BUF_SIZE];
static uint8_t s_visionRxIdx = 0U;
#endif
#endif

#if CAR_ROLE_SLAVE
static volatile F21CoopSlaveState_t s_slaveState = F21_COOP_SLAVE_IDLE;
static volatile uint8_t s_stateEntry = 0U;
static volatile uint8_t s_atWaitSent = 0U;
static volatile uint8_t s_releasePending = 0U;
#endif

static uint8_t Coop_IsRoom34(uint8_t room)
{
    return (room == 3U || room == 4U) ? 1U : 0U;
}

static uint8_t Coop_GetOppositeRoom(uint8_t room)
{
    return (room == 3U) ? 4U : 3U;
}

static int32_t Coop_CmToPulse(float cm)
{
    return (int32_t)(cm / ECAR_CM_PER_PULSE + 0.5f);
}

static int32_t Coop_GetDistanceFromPulse(int32_t startPulse)
{
    int32_t delta = g_forwardEncoderTotal - startPulse;
    return (delta >= 0) ? delta : -delta;
}

static int32_t Coop_GetTurnDistanceFromPulse(int32_t startPulse)
{
    int32_t delta = g_turnEncoderTotal - startPulse;
    return (delta >= 0) ? delta : -delta;
}

static F21TurnDir_t Coop_GetOutboundTurn(uint8_t room)
{
    return (room == 3U) ? F21_TURN_LEFT : F21_TURN_RIGHT;
}

static F21TurnDir_t Coop_GetReturnTurn(uint8_t room)
{
    return (room == 3U) ? F21_TURN_RIGHT : F21_TURN_LEFT;
}

static float Coop_TurnDirToSign(F21TurnDir_t dir)
{
    return (dir == F21_TURN_LEFT) ? 1.0f : -1.0f;
}

static void Coop_ApplyOutboundRoute(uint8_t room)
{
    s_detectStartPulse = Coop_CmToPulse(F21_COOP_MID_DETECT_CM);
    s_outboundTurnDir = Coop_GetOutboundTurn(room);
    s_roomRunPulse = Coop_CmToPulse(F21_COOP_ROOM_RUN_CM);
}

static void Coop_ApplyReturnRoute(uint8_t room)
{
    s_returnDetectStartPulse = Coop_CmToPulse(F21_COOP_RETURN_DETECT_CM);
    s_returnTurnDir = Coop_GetReturnTurn(room);
    s_returnFinalRunPulse = Coop_CmToPulse(F21_COOP_RETURN_HOME_CM);
}

static void Coop_SafeStop(void)
{
    App_Control_ForcePWMZero();
    Motor_StopAll();
}

static void Coop_ClearEncoderTotals(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

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

    g_leftPwm = 0;
    g_rightPwm = 0;
    g_speedPwm = 0.0f;
    g_diffPwm = 0.0f;
    g_forwardSpeedError = 0.0f;

    Encoder_ClearAll();

    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void Coop_ResetRunData(void)
{
    Coop_SafeStop();
    Coop_ClearEncoderTotals();
    App_Line_ResetState();
    App_Control_ResetPID();

    s_stateMs = 0U;
#if CAR_ROLE_SLAVE
    s_stateEntry = 0U;
#endif
    s_stateStartPulse = 0;
    s_crossPulse = 0;
    s_turnStartPulse = 0;
    s_crossMonitoring = 0U;
    s_crossConfirmCnt = 0U;
}

static void Coop_BeginLineRun(void)
{
    Coop_ResetRunData();
    s_stateStartPulse = g_forwardEncoderTotal;
}

static uint8_t Coop_IsCrossDetected(void)
{
    return (g_lineBlackCount >= F21_COOP_CROSS_BLACK_THRESH) ? 1U : 0U;
}

static void Coop_SetMotionCmd(float forward, float turn)
{
    g_targetForwardSpeed = forward;
    g_targetTurnSpeed = turn;
    g_carEnable = 1U;
    App_Control_ApplyMotorOutput();
}

static uint8_t Coop_LineRunUntilCross(int32_t detectStartPulse)
{
    App_Line_Update();

    if (!g_lineValid)
    {
        App_Control_ForcePWMZero();
        return 0U;
    }

    Coop_SetMotionCmd(F21_LINE_BASE_SPEED_CMPS, App_Line_CalcTurnCmd());

    if (!s_crossMonitoring)
    {
        if (Coop_GetDistanceFromPulse(s_stateStartPulse) >= detectStartPulse)
        {
            s_crossMonitoring = 1U;
            s_crossConfirmCnt = 0U;
        }
        return 0U;
    }

    if (Coop_IsCrossDetected())
    {
        s_crossConfirmCnt++;
        if (s_crossConfirmCnt >= F21_COOP_CROSS_CONFIRM_TICKS)
        {
            s_crossPulse = g_forwardEncoderTotal;
            return 1U;
        }
    }
    else
    {
        s_crossConfirmCnt = 0U;
    }

    return 0U;
}

static uint8_t Coop_LineRunDistance(int32_t startPulse, int32_t targetPulse, float speed)
{
    if (targetPulse <= 0)
    {
        return 1U;
    }

    if (Coop_GetDistanceFromPulse(startPulse) >= targetPulse)
    {
        return 1U;
    }

    App_Line_Update();
    if (!g_lineValid)
    {
        App_Control_ForcePWMZero();
        return 0U;
    }

    Coop_SetMotionCmd(speed, App_Line_CalcTurnCmd());
    return 0U;
}

static uint8_t Coop_DriveStraightDistance(int32_t startPulse, int32_t targetPulse, float speed)
{
    if (targetPulse <= 0)
    {
        return 1U;
    }

    Coop_SetMotionCmd(speed, 0.0f);
    return (Coop_GetDistanceFromPulse(startPulse) >= targetPulse) ? 1U : 0U;
}

static uint8_t Coop_IsTurnComplete(uint16_t targetPulse)
{
    if (targetPulse == 0U) return 0U;
    return (Coop_GetTurnDistanceFromPulse(s_turnStartPulse) >= (int32_t)targetPulse) ? 1U : 0U;
}

static uint8_t Coop_Turn90(F21TurnDir_t dir)
{
    if (F21_TURN_90_PULSE == 0U) return 0U;

    Coop_SetMotionCmd(0.0f, Coop_TurnDirToSign(dir) * F21_TURN_SPEED_CMPS);
    return Coop_IsTurnComplete(F21_TURN_90_PULSE);
}

static uint8_t Coop_Turn180Right(void)
{
    if (F21_TURN_180_PULSE == 0U) return 0U;

    Coop_SetMotionCmd(0.0f, Coop_TurnDirToSign(F21_TURN_RIGHT) * F21_TURN_SPEED_CMPS);
    return Coop_IsTurnComplete(F21_TURN_180_PULSE);
}

static void Coop_ResetRadioRetry(void)
{
    s_radioRetryMs = F21_COOP_RADIO_RETRY_MS;
    s_radioRetryFailCount = 0U;
}

static uint8_t Coop_IsRadioRetryDue(void)
{
    if (s_radioRetryMs < F21_COOP_RADIO_RETRY_MS)
    {
        s_radioRetryMs += CAR_CONTROL_PERIOD_MS;
        return 0U;
    }

    s_radioRetryMs = 0U;
    return 1U;
}

static void Coop_LogRetryFailure(const char *tag)
{
    s_radioRetryFailCount++;
    if (s_radioRetryFailCount == 1U || (s_radioRetryFailCount % 10U) == 0U)
    {
        DebugSerial_Printf("%s\r\n", tag);
    }
}

static void Coop_SetYellowWait(uint8_t enable)
{
    if (enable)
    {
        if (s_yellowWaitActive == 0U)
        {
            LED_User_CancelBlink();
            F21Car_CancelLedDisplay();
            s_yellowWaitActive = 1U;
        }
        LED_User_On();
    }
    else
    {
        s_yellowWaitActive = 0U;
        LED_User_Off();
    }
}

void F21Coop_CancelYellowWait(void)
{
    Coop_SetYellowWait(0U);
}

static void Coop_EnterFault(uint8_t code)
{
    Coop_SafeStop();
    s_faultCode = code;
    Coop_SetYellowWait(0U);
    DebugSerial_Printf("[coop,fault=%u]\r\n", (unsigned int)code);

#if CAR_ROLE_MASTER
    s_masterState = F21_COOP_MASTER_FAULT;
#elif CAR_ROLE_SLAVE
    s_releasePending = 0U;
    s_atWaitSent = 0U;
    s_slaveState = F21_COOP_SLAVE_FAULT;
#endif
}

#if CAR_ROLE_MASTER
static void Coop_MasterSetState(F21CoopMasterState_t state)
{
    if (s_masterState == state) return;

    s_masterState = state;
    s_stateMs = 0U;

    if (state == F21_COOP_MASTER_SEND_TARGET ||
        state == F21_COOP_MASTER_SEND_SLAVE_START ||
        state == F21_COOP_MASTER_SEND_RELEASE)
    {
        Coop_ResetRadioRetry();
    }

    switch (state)
    {
    case F21_COOP_MASTER_LOAD_DELAY:
        DebugSerial_Printf("[coop,master,start]\r\n");
        break;
    case F21_COOP_MASTER_ROOM_RUN:
        DebugSerial_Printf("[coop,master,room]\r\n");
        break;
    case F21_COOP_MASTER_FINISH:
        DebugSerial_Printf("[coop,finish]\r\n");
        break;
    default:
        break;
    }
}

static uint8_t Coop_MasterCanStart(void)
{
    return (s_masterState == F21_COOP_MASTER_IDLE ||
            s_masterState == F21_COOP_MASTER_WAIT_TARGET) ? 1U : 0U;
}

static void Coop_MasterStartTask(uint8_t room)
{
    if (!Coop_IsRoom34(room)) return;
    if (!Coop_MasterCanStart()) return;

    s_targetRoom = room;
    s_waitRoom = Coop_GetOppositeRoom(room);
    s_targetSent = 0U;
    s_slaveStartSent = 0U;
    s_slaveReleaseSent = 0U;
    s_slaveAtWaitReceived = 0U;
    s_waitSlaveLogged = 0U;
    s_finishUnlockSent = 0U;
    s_faultCode = 0U;

    Coop_SetYellowWait(0U);
    Coop_ApplyOutboundRoute(s_targetRoom);
    Coop_ApplyReturnRoute(s_targetRoom);
    Coop_ResetRunData();

    DebugSerial_Printf("[coop,target=%u]\r\n", (unsigned int)s_targetRoom);
    Coop_MasterSetState(F21_COOP_MASTER_SEND_TARGET);
}

#if ENABLE_K230
static void Coop_MasterVision_SendConfirm(uint8_t room)
{
    Serial_SendString("[num,");
    Serial_SendByte((uint8_t)('0' + room));
    Serial_SendString("]\r\n");
}

static void Coop_MasterVision_ParseCommand(const char *buf)
{
    int32_t num = 0;
    const char *p;

    if (buf[0] != '[') return;
    if (buf[1] != 'n' || buf[2] != 'u' || buf[3] != 'm' || buf[4] != ',') return;

    p = buf + 5;
    while (*p >= '0' && *p <= '9')
    {
        num = num * 10 + (int32_t)(*p - '0');
        p++;
    }

    if (*p != ']') return;

    if (num == 3 || num == 4)
    {
        Coop_MasterVision_SendConfirm((uint8_t)num);
        Coop_MasterStartTask((uint8_t)num);
    }
    else if (num >= 1 && num <= 8)
    {
        DebugSerial_Printf("[coop,ignore,k230=%u]\r\n", (unsigned int)num);
    }
}

static void Coop_MasterVision_Process(void)
{
    uint8_t byte;

    while (Serial_ReadByte(&byte))
    {
        if (byte == '[')
        {
            s_visionRxIdx = 0U;
            s_visionRxBuf[s_visionRxIdx++] = (char)byte;
        }
        else if (s_visionRxIdx > 0U)
        {
            if (s_visionRxIdx < sizeof(s_visionRxBuf) - 1U)
            {
                s_visionRxBuf[s_visionRxIdx++] = (char)byte;
            }

            if (byte == ']')
            {
                s_visionRxBuf[s_visionRxIdx] = '\0';
                Coop_MasterVision_ParseCommand(s_visionRxBuf);
                s_visionRxIdx = 0U;
            }
        }
    }
}
#endif

static void Coop_MasterProcessRadio10ms(void)
{
    AppRadioCommand_t cmd;

    while (App_Radio_PopCommand(&cmd))
    {
        if (cmd.cmd != RADIO_CMD_SLAVE_AT_WAIT)
        {
            DebugSerial_Printf("[coop,ignore,cmd=%u]\r\n", (unsigned int)cmd.cmd);
            continue;
        }

        if (cmd.room_id != s_targetRoom)
        {
            DebugSerial_Printf("[coop,ignore,at_wait,room=%u]\r\n",
                (unsigned int)cmd.room_id);
            continue;
        }

        if (s_slaveStartSent == 0U)
        {
            continue;
        }

        if (s_masterState == F21_COOP_MASTER_UNLOAD_WAIT ||
            s_masterState == F21_COOP_MASTER_SEND_SLAVE_START)
        {
            if (s_slaveAtWaitReceived == 0U)
            {
                s_slaveAtWaitReceived = 1U;
                DebugSerial_Printf("[coop,master,slave_at_wait]\r\n");
            }
        }
    }
}

static void Coop_MasterSendTarget10ms(void)
{
    Coop_SafeStop();

    if (!Coop_IsRadioRetryDue()) return;

    if (App_Radio_SendTargetRoom(s_targetRoom))
    {
        s_targetSent = 1U;
        DebugSerial_Printf("[coop,master,target_sent]\r\n");
        Coop_MasterSetState(F21_COOP_MASTER_LOAD_DELAY);
    }
    else
    {
        Coop_LogRetryFailure("[coop,master,target_retry]");
    }
}

static void Coop_MasterSendSlaveStart10ms(void)
{
    Coop_SafeStop();

    if (!Coop_IsRadioRetryDue()) return;

    if (App_Radio_SendSlaveStart(s_targetRoom))
    {
        s_slaveStartSent = 1U;
        s_slaveAtWaitReceived = 0U;
        s_waitSlaveLogged = 0U;
        DebugSerial_Printf("[coop,master,start_sent]\r\n");
        Coop_MasterSetState(F21_COOP_MASTER_UNLOAD_WAIT);
    }
    else
    {
        Coop_LogRetryFailure("[coop,master,start_retry]");
    }
}

static void Coop_MasterSendRelease10ms(void)
{
    Coop_SafeStop();

    if (!Coop_IsRadioRetryDue()) return;

    if (App_Radio_SendSlaveRelease(s_targetRoom))
    {
        s_slaveReleaseSent = 1U;
        DebugSerial_Printf("[coop,master,release_sent]\r\n");
        s_stateStartPulse = g_forwardEncoderTotal;
        Coop_MasterSetState(F21_COOP_MASTER_RETURN_HOME);
    }
    else
    {
        Coop_LogRetryFailure("[coop,master,release_retry]");
    }
}

static void Coop_MasterFinishTask10ms(void)
{
    Coop_SafeStop();
    Coop_SetYellowWait(0U);

    if (s_finishUnlockSent == 0U)
    {
#if ENABLE_K230
        Serial_SendString("[num,unlock]\r\n");
#endif
        s_finishUnlockSent = 1U;
    }

    if (s_stateMs >= F21_COOP_FINISH_QUIET_MS)
    {
        Coop_MasterSetState(F21_COOP_MASTER_WAIT_TARGET);
    }
}

static void Coop_MasterTask10ms(void)
{
    Coop_MasterProcessRadio10ms();

#if ENABLE_K230
    if (Coop_MasterCanStart())
    {
        Coop_MasterVision_Process();
    }
#endif

    switch (s_masterState)
    {
    case F21_COOP_MASTER_IDLE:
    case F21_COOP_MASTER_WAIT_TARGET:
        Coop_SafeStop();
        break;

    case F21_COOP_MASTER_SEND_TARGET:
        Coop_MasterSendTarget10ms();
        break;

    case F21_COOP_MASTER_LOAD_DELAY:
        Coop_SafeStop();
        if (s_stateMs >= F21_COOP_LOAD_DELAY_MS)
        {
            Coop_BeginLineRun();
            Coop_MasterSetState(F21_COOP_MASTER_OUTBOUND_MAIN);
        }
        break;

    case F21_COOP_MASTER_OUTBOUND_MAIN:
        if (Coop_LineRunUntilCross(s_detectStartPulse))
        {
            Coop_MasterSetState(F21_COOP_MASTER_OUTBOUND_CROSS_ADVANCE);
        }
        break;

    case F21_COOP_MASTER_OUTBOUND_CROSS_ADVANCE:
        if (Coop_DriveStraightDistance(s_crossPulse, Coop_CmToPulse(F21_CROSS_ADVANCE_CM),
                F21_CROSS_ADVANCE_SPEED_CMPS))
        {
            s_turnStartPulse = g_turnEncoderTotal;
            Coop_MasterSetState(F21_COOP_MASTER_OUTBOUND_TURN);
        }
        break;

    case F21_COOP_MASTER_OUTBOUND_TURN:
        if (F21_TURN_90_PULSE == 0U)
        {
            Coop_EnterFault(1U);
            break;
        }
        if (Coop_Turn90(s_outboundTurnDir))
        {
            Coop_SafeStop();
            s_stateStartPulse = g_forwardEncoderTotal;
            Coop_MasterSetState(F21_COOP_MASTER_ROOM_RUN);
        }
        break;

    case F21_COOP_MASTER_ROOM_RUN:
        if (Coop_LineRunDistance(s_stateStartPulse, s_roomRunPulse, F21_LINE_BASE_SPEED_CMPS))
        {
            Coop_SafeStop();
            Coop_MasterSetState(F21_COOP_MASTER_SEND_SLAVE_START);
        }
        break;

    case F21_COOP_MASTER_SEND_SLAVE_START:
        Coop_MasterSendSlaveStart10ms();
        break;

    case F21_COOP_MASTER_UNLOAD_WAIT:
        Coop_SafeStop();
        if (s_stateMs >= F21_UNLOAD_WAIT_MS && s_slaveAtWaitReceived != 0U)
        {
            s_turnStartPulse = g_turnEncoderTotal;
            DebugSerial_Printf("[coop,master,return_start]\r\n");
            Coop_MasterSetState(F21_COOP_MASTER_TURN_AROUND);
        }
        else if (s_stateMs >= F21_UNLOAD_WAIT_MS && s_waitSlaveLogged == 0U)
        {
            s_waitSlaveLogged = 1U;
            DebugSerial_Printf("[coop,master,wait_slave]\r\n");
        }
        break;

    case F21_COOP_MASTER_TURN_AROUND:
        if (F21_TURN_180_PULSE == 0U)
        {
            Coop_EnterFault(2U);
            break;
        }
        if (Coop_Turn180Right())
        {
            Coop_SafeStop();
            Coop_ApplyReturnRoute(s_targetRoom);
            s_crossMonitoring = 0U;
            s_crossConfirmCnt = 0U;
            s_stateStartPulse = g_forwardEncoderTotal;
            Coop_MasterSetState(F21_COOP_MASTER_RETURN_BRANCH);
        }
        break;

    case F21_COOP_MASTER_RETURN_BRANCH:
        if (Coop_LineRunUntilCross(s_returnDetectStartPulse))
        {
            Coop_MasterSetState(F21_COOP_MASTER_RETURN_CROSS_ADVANCE);
        }
        break;

    case F21_COOP_MASTER_RETURN_CROSS_ADVANCE:
        if (Coop_DriveStraightDistance(s_crossPulse, Coop_CmToPulse(F21_CROSS_ADVANCE_CM),
                F21_CROSS_ADVANCE_SPEED_CMPS))
        {
            s_turnStartPulse = g_turnEncoderTotal;
            Coop_MasterSetState(F21_COOP_MASTER_RETURN_TURN);
        }
        break;

    case F21_COOP_MASTER_RETURN_TURN:
        if (F21_TURN_90_PULSE == 0U)
        {
            Coop_EnterFault(3U);
            break;
        }
        if (Coop_Turn90(s_returnTurnDir))
        {
            Coop_SafeStop();
            s_stateStartPulse = g_forwardEncoderTotal;
            Coop_MasterSetState(F21_COOP_MASTER_CLEAR_JUNCTION);
        }
        break;

    case F21_COOP_MASTER_CLEAR_JUNCTION:
        if (Coop_LineRunDistance(s_stateStartPulse, Coop_CmToPulse(F21_COOP_RELEASE_CLEAR_CM),
                F21_LINE_BASE_SPEED_CMPS))
        {
            s_returnHomeRemainPulse = s_returnFinalRunPulse - Coop_CmToPulse(F21_COOP_RELEASE_CLEAR_CM);
            if (s_returnHomeRemainPulse < 0) s_returnHomeRemainPulse = 0;
            s_stateStartPulse = g_forwardEncoderTotal;
            Coop_MasterSetState(F21_COOP_MASTER_SEND_RELEASE);
        }
        break;

    case F21_COOP_MASTER_SEND_RELEASE:
        Coop_MasterSendRelease10ms();
        break;

    case F21_COOP_MASTER_RETURN_HOME:
        if (Coop_LineRunDistance(s_stateStartPulse, s_returnHomeRemainPulse, F21_LINE_BASE_SPEED_CMPS))
        {
            Coop_SafeStop();
            s_turnStartPulse = g_turnEncoderTotal;
            Coop_MasterSetState(F21_COOP_MASTER_RETURN_HOME_TURN_AROUND);
        }
        break;

    case F21_COOP_MASTER_RETURN_HOME_TURN_AROUND:
        if (F21_TURN_180_PULSE == 0U)
        {
            Coop_EnterFault(4U);
            break;
        }
        if (Coop_Turn180Right())
        {
            Coop_SafeStop();
            Coop_MasterSetState(F21_COOP_MASTER_FINISH);
        }
        break;

    case F21_COOP_MASTER_FINISH:
        Coop_MasterFinishTask10ms();
        break;

    case F21_COOP_MASTER_STOP:
    case F21_COOP_MASTER_FAULT:
        Coop_SafeStop();
        break;

    default:
        Coop_EnterFault(250U);
        break;
    }
}
#endif

#if CAR_ROLE_SLAVE
static uint8_t Coop_TakeStateEntry(void)
{
    if (s_stateEntry)
    {
        s_stateEntry = 0U;
        return 1U;
    }

    return 0U;
}

static uint8_t Coop_SlaveIsGoingToWaitRoom(void)
{
    return (s_slaveState == F21_COOP_SLAVE_GO_WAIT_MAIN ||
            s_slaveState == F21_COOP_SLAVE_GO_WAIT_CROSS_ADVANCE ||
            s_slaveState == F21_COOP_SLAVE_GO_WAIT_TURN ||
            s_slaveState == F21_COOP_SLAVE_GO_WAIT_ROOM) ? 1U : 0U;
}

static void Coop_SlaveSetState(F21CoopSlaveState_t state)
{
    if (s_slaveState == state) return;

    s_slaveState = state;
    s_stateMs = 0U;
    s_stateEntry = 1U;

    switch (state)
    {
    case F21_COOP_SLAVE_YELLOW_WAIT:
        s_atWaitSent = 0U;
        Coop_ResetRadioRetry();
        DebugSerial_Printf("[coop,slave,yellow_wait]\r\n");
        break;
    case F21_COOP_SLAVE_TARGET_ROOM_RUN:
        DebugSerial_Printf("[coop,slave,target_room]\r\n");
        break;
    case F21_COOP_SLAVE_FINISH:
        DebugSerial_Printf("[coop,finish]\r\n");
        break;
    default:
        break;
    }
}

static void Coop_SlaveIgnore(uint8_t cmd)
{
    DebugSerial_Printf("[coop,ignore,cmd=%u]\r\n", (unsigned int)cmd);
}

static void Coop_SlaveIgnoreRoom(uint8_t room)
{
    DebugSerial_Printf("[coop,ignore,room=%u]\r\n", (unsigned int)room);
}

static void Coop_SlaveAtWaitTask10ms(void)
{
    Coop_SafeStop();
    Coop_SetYellowWait(1U);

    if (s_atWaitSent == 0U && Coop_IsRadioRetryDue())
    {
        if (App_Radio_SendSlaveAtWait(s_targetRoom))
        {
            s_atWaitSent = 1U;
            DebugSerial_Printf("[coop,slave,at_wait_sent]\r\n");
        }
        else
        {
            Coop_LogRetryFailure("[coop,slave,at_wait_retry]");
        }
    }

    if (s_releasePending != 0U)
    {
        s_releasePending = 0U;
        Coop_SetYellowWait(0U);
        DebugSerial_Printf("[coop,slave,release]\r\n");
        Coop_SlaveSetState(F21_COOP_SLAVE_LEAVE_WAIT_TURN_AROUND);
    }
}

static void Coop_SlaveOnRadioCommand(uint8_t cmd, uint8_t room)
{
    switch (cmd)
    {
    case RADIO_CMD_TARGET_ROOM:
        if (!Coop_IsRoom34(room))
        {
            Coop_SlaveIgnoreRoom(room);
            return;
        }

        if (s_slaveState == F21_COOP_SLAVE_IDLE)
        {
            s_targetRoom = room;
            s_waitRoom = Coop_GetOppositeRoom(room);
            s_faultCode = 0U;
            s_releasePending = 0U;
            s_atWaitSent = 0U;
            Coop_SetYellowWait(0U);
            Coop_SlaveSetState(F21_COOP_SLAVE_WAIT_START);
            DebugSerial_Printf("[coop,slave,target=%u,wait=%u]\r\n",
                (unsigned int)s_targetRoom, (unsigned int)s_waitRoom);
        }
        else if (s_slaveState == F21_COOP_SLAVE_WAIT_START)
        {
            if (room == s_targetRoom)
            {
                return;
            }
            else
            {
                Coop_SlaveIgnoreRoom(room);
            }
        }
        else
        {
            Coop_SlaveIgnore(cmd);
        }
        break;

    case RADIO_CMD_SLAVE_START:
        if (room != s_targetRoom)
        {
            Coop_SlaveIgnoreRoom(room);
        }
        else if (s_slaveState == F21_COOP_SLAVE_WAIT_START)
        {
            DebugSerial_Printf("[coop,slave,start]\r\n");
            Coop_SlaveSetState(F21_COOP_SLAVE_GO_WAIT_MAIN);
        }
        else
        {
            Coop_SlaveIgnore(cmd);
        }
        break;

    case RADIO_CMD_SLAVE_RELEASE:
        if (room != s_targetRoom)
        {
            Coop_SlaveIgnoreRoom(room);
        }
        else if (s_slaveState == F21_COOP_SLAVE_YELLOW_WAIT)
        {
            s_releasePending = 0U;
            Coop_SetYellowWait(0U);
            DebugSerial_Printf("[coop,slave,release]\r\n");
            Coop_SlaveSetState(F21_COOP_SLAVE_LEAVE_WAIT_TURN_AROUND);
        }
        else if (Coop_SlaveIsGoingToWaitRoom())
        {
            if (s_releasePending == 0U)
            {
                s_releasePending = 1U;
                DebugSerial_Printf("[coop,slave,release_pending]\r\n");
            }
        }
        else
        {
            Coop_SlaveIgnore(cmd);
        }
        break;

    default:
        Coop_SlaveIgnore(cmd);
        break;
    }
}

static void Coop_SlaveProcessRadio10ms(void)
{
    AppRadioCommand_t cmd;

    while (App_Radio_PopCommand(&cmd))
    {
        Coop_SlaveOnRadioCommand(cmd.cmd, cmd.room_id);
    }
}

static void Coop_SlaveFinishTask10ms(void)
{
    Coop_SafeStop();
    Coop_SetYellowWait(0U);

    if (s_stateMs >= F21_COOP_FINISH_QUIET_MS)
    {
        Coop_SlaveSetState(F21_COOP_SLAVE_IDLE);
    }
}

static void Coop_SlaveTask10ms(void)
{
    Coop_SlaveProcessRadio10ms();

    switch (s_slaveState)
    {
    case F21_COOP_SLAVE_IDLE:
    case F21_COOP_SLAVE_WAIT_START:
        Coop_SafeStop();
        break;

    case F21_COOP_SLAVE_GO_WAIT_MAIN:
        if (Coop_TakeStateEntry())
        {
            Coop_ApplyOutboundRoute(s_waitRoom);
            Coop_BeginLineRun();
        }
        if (Coop_LineRunUntilCross(s_detectStartPulse))
        {
            Coop_SlaveSetState(F21_COOP_SLAVE_GO_WAIT_CROSS_ADVANCE);
        }
        break;

    case F21_COOP_SLAVE_GO_WAIT_CROSS_ADVANCE:
        if (Coop_DriveStraightDistance(s_crossPulse, Coop_CmToPulse(F21_CROSS_ADVANCE_CM),
                F21_CROSS_ADVANCE_SPEED_CMPS))
        {
            s_turnStartPulse = g_turnEncoderTotal;
            Coop_SlaveSetState(F21_COOP_SLAVE_GO_WAIT_TURN);
        }
        break;

    case F21_COOP_SLAVE_GO_WAIT_TURN:
        if (F21_TURN_90_PULSE == 0U)
        {
            Coop_EnterFault(11U);
            break;
        }
        if (Coop_Turn90(s_outboundTurnDir))
        {
            Coop_SafeStop();
            s_stateStartPulse = g_forwardEncoderTotal;
            Coop_SlaveSetState(F21_COOP_SLAVE_GO_WAIT_ROOM);
        }
        break;

    case F21_COOP_SLAVE_GO_WAIT_ROOM:
        if (Coop_LineRunDistance(s_stateStartPulse, s_roomRunPulse, F21_LINE_BASE_SPEED_CMPS))
        {
            Coop_SafeStop();
            Coop_SlaveSetState(F21_COOP_SLAVE_YELLOW_WAIT);
        }
        break;

    case F21_COOP_SLAVE_YELLOW_WAIT:
        Coop_SlaveAtWaitTask10ms();
        break;

    case F21_COOP_SLAVE_LEAVE_WAIT_TURN_AROUND:
        if (Coop_TakeStateEntry())
        {
            Coop_SetYellowWait(0U);
            s_turnStartPulse = g_turnEncoderTotal;
        }
        if (F21_TURN_180_PULSE == 0U)
        {
            Coop_EnterFault(12U);
            break;
        }
        if (Coop_Turn180Right())
        {
            Coop_SafeStop();
            Coop_ApplyReturnRoute(s_waitRoom);
            s_crossMonitoring = 0U;
            s_crossConfirmCnt = 0U;
            s_stateStartPulse = g_forwardEncoderTotal;
            Coop_SlaveSetState(F21_COOP_SLAVE_LEAVE_WAIT_BRANCH);
        }
        break;

    case F21_COOP_SLAVE_LEAVE_WAIT_BRANCH:
        if (Coop_LineRunUntilCross(s_returnDetectStartPulse))
        {
            Coop_SlaveSetState(F21_COOP_SLAVE_CROSS_TO_TARGET);
        }
        break;

    case F21_COOP_SLAVE_CROSS_TO_TARGET:
        if (Coop_DriveStraightDistance(s_crossPulse, Coop_CmToPulse(F21_COOP_CROSS_TO_TARGET_ADVANCE_CM),
                F21_CROSS_ADVANCE_SPEED_CMPS))
        {
            s_stateStartPulse = g_forwardEncoderTotal;
            s_roomRunPulse = Coop_CmToPulse(F21_COOP_ROOM_RUN_CM);
            Coop_SlaveSetState(F21_COOP_SLAVE_TARGET_ROOM_RUN);
        }
        break;

    case F21_COOP_SLAVE_TARGET_ROOM_RUN:
        if (Coop_LineRunDistance(s_stateStartPulse, s_roomRunPulse, F21_LINE_BASE_SPEED_CMPS))
        {
            Coop_SafeStop();
            Coop_SlaveSetState(F21_COOP_SLAVE_UNLOAD_WAIT);
        }
        break;

    case F21_COOP_SLAVE_UNLOAD_WAIT:
        Coop_SafeStop();
        if (s_stateMs >= F21_UNLOAD_WAIT_MS)
        {
            s_turnStartPulse = g_turnEncoderTotal;
            Coop_SlaveSetState(F21_COOP_SLAVE_TARGET_TURN_AROUND);
        }
        break;

    case F21_COOP_SLAVE_TARGET_TURN_AROUND:
        if (F21_TURN_180_PULSE == 0U)
        {
            Coop_EnterFault(13U);
            break;
        }
        if (Coop_Turn180Right())
        {
            Coop_SafeStop();
            Coop_ApplyReturnRoute(s_targetRoom);
            s_crossMonitoring = 0U;
            s_crossConfirmCnt = 0U;
            s_stateStartPulse = g_forwardEncoderTotal;
            Coop_SlaveSetState(F21_COOP_SLAVE_RETURN_BRANCH);
        }
        break;

    case F21_COOP_SLAVE_RETURN_BRANCH:
        if (Coop_LineRunUntilCross(s_returnDetectStartPulse))
        {
            Coop_SlaveSetState(F21_COOP_SLAVE_RETURN_CROSS_ADVANCE);
        }
        break;

    case F21_COOP_SLAVE_RETURN_CROSS_ADVANCE:
        if (Coop_DriveStraightDistance(s_crossPulse, Coop_CmToPulse(F21_CROSS_ADVANCE_CM),
                F21_CROSS_ADVANCE_SPEED_CMPS))
        {
            s_turnStartPulse = g_turnEncoderTotal;
            Coop_SlaveSetState(F21_COOP_SLAVE_RETURN_TURN);
        }
        break;

    case F21_COOP_SLAVE_RETURN_TURN:
        if (F21_TURN_90_PULSE == 0U)
        {
            Coop_EnterFault(14U);
            break;
        }
        if (Coop_Turn90(s_returnTurnDir))
        {
            Coop_SafeStop();
            s_stateStartPulse = g_forwardEncoderTotal;
            Coop_SlaveSetState(F21_COOP_SLAVE_RETURN_HOME);
        }
        break;

    case F21_COOP_SLAVE_RETURN_HOME:
        if (Coop_LineRunDistance(s_stateStartPulse, s_returnFinalRunPulse, F21_LINE_BASE_SPEED_CMPS))
        {
            Coop_SafeStop();
            s_turnStartPulse = g_turnEncoderTotal;
            Coop_SlaveSetState(F21_COOP_SLAVE_RETURN_HOME_TURN_AROUND);
        }
        break;

    case F21_COOP_SLAVE_RETURN_HOME_TURN_AROUND:
        if (F21_TURN_180_PULSE == 0U)
        {
            Coop_EnterFault(15U);
            break;
        }
        if (Coop_Turn180Right())
        {
            Coop_SafeStop();
            Coop_SlaveSetState(F21_COOP_SLAVE_FINISH);
        }
        break;

    case F21_COOP_SLAVE_FINISH:
        Coop_SlaveFinishTask10ms();
        break;

    case F21_COOP_SLAVE_STOP:
    case F21_COOP_SLAVE_FAULT:
        Coop_SafeStop();
        break;

    default:
        Coop_EnterFault(251U);
        break;
    }
}
#endif

void F21Coop_Init(void)
{
    F21Coop_ResetTask();
}

void F21Coop_Tick1ms(void)
{
}

void F21Coop_Task10ms(void)
{
    s_stateMs += CAR_CONTROL_PERIOD_MS;

#if CAR_ROLE_MASTER
    Coop_MasterTask10ms();
#elif CAR_ROLE_SLAVE
    Coop_SlaveTask10ms();
#else
    Coop_SafeStop();
#endif
}

void F21Coop_Task100ms(void)
{
}

void F21Coop_Task200ms(void)
{
}

void F21Coop_HandleKey(uint8_t key)
{
    if (key == 0U) return;

    switch (key)
    {
    case 1U:
#if CAR_ROLE_MASTER
        if (Coop_MasterCanStart())
        {
            s_targetRoom = (s_targetRoom == 3U) ? 4U : 3U;
            s_waitRoom = Coop_GetOppositeRoom(s_targetRoom);
            Coop_MasterSetState(F21_COOP_MASTER_WAIT_TARGET);
            DebugSerial_Printf("[coop,target=%u]\r\n", (unsigned int)s_targetRoom);
        }
#endif
        break;

    case 2U:
#if CAR_ROLE_MASTER
        Coop_MasterStartTask(s_targetRoom);
#endif
        break;

    case 3U:
#if CAR_ROLE_MASTER
        if (s_masterState != F21_COOP_MASTER_IDLE &&
            s_masterState != F21_COOP_MASTER_WAIT_TARGET &&
            s_masterState != F21_COOP_MASTER_FINISH &&
            s_masterState != F21_COOP_MASTER_FAULT)
        {
            Coop_SafeStop();
            Coop_SetYellowWait(0U);
            s_targetSent = 0U;
            s_slaveStartSent = 0U;
            s_slaveReleaseSent = 0U;
            s_slaveAtWaitReceived = 0U;
            s_waitSlaveLogged = 0U;
            Coop_MasterSetState(F21_COOP_MASTER_STOP);
        }
#elif CAR_ROLE_SLAVE
        if (s_slaveState != F21_COOP_SLAVE_IDLE &&
            s_slaveState != F21_COOP_SLAVE_FINISH &&
            s_slaveState != F21_COOP_SLAVE_FAULT)
        {
            Coop_SafeStop();
            Coop_SetYellowWait(0U);
            s_releasePending = 0U;
            s_atWaitSent = 0U;
            Coop_SlaveSetState(F21_COOP_SLAVE_STOP);
        }
#endif
        break;

    case 4U:
        F21Coop_ResetTask();
        break;

    default:
        break;
    }
}

void F21Coop_ResetTask(void)
{
    App_Radio_ClearPendingCommands();
    Coop_SetYellowWait(0U);
    Coop_ResetRunData();

    s_targetRoom = 3U;
    s_waitRoom = 4U;
    s_faultCode = 0U;

#if CAR_ROLE_MASTER
    s_targetSent = 0U;
    s_slaveStartSent = 0U;
    s_slaveReleaseSent = 0U;
    s_slaveAtWaitReceived = 0U;
    s_waitSlaveLogged = 0U;
    s_finishUnlockSent = 0U;
    s_returnHomeRemainPulse = 0;
#endif

#if CAR_ROLE_MASTER
    s_masterState = F21_COOP_MASTER_IDLE;
#if ENABLE_K230
    s_visionRxIdx = 0U;
#endif
#endif

#if CAR_ROLE_SLAVE
    s_atWaitSent = 0U;
    s_releasePending = 0U;
    s_slaveState = F21_COOP_SLAVE_IDLE;
#endif

    Coop_ResetRadioRetry();
}

uint8_t F21Coop_IsModeSwitchAllowed(void)
{
    uint8_t motorStopped = (g_carEnable == 0U && g_leftPwm == 0 && g_rightPwm == 0) ? 1U : 0U;

    if (!motorStopped) return 0U;

#if CAR_ROLE_MASTER
    return (s_masterState == F21_COOP_MASTER_IDLE ||
            s_masterState == F21_COOP_MASTER_WAIT_TARGET) ? 1U : 0U;
#elif CAR_ROLE_SLAVE
    return (s_slaveState == F21_COOP_SLAVE_IDLE) ? 1U : 0U;
#else
    return 0U;
#endif
}
