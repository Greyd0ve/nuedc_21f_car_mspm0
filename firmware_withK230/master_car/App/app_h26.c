#include "app_h26.h"
#include "app_h26_config.h"
#include "app_h26_led.h"
#include "app_h26_task2.h"
#include "app_car_state.h"
#include "app_control.h"
#include "app_line.h"
#include "DebugSerial.h"
#include "Encoder.h"
#include "Key.h"
#include "OLED.h"
#include "Timer.h"
#include "cmsis_compiler.h"
#include <stdint.h>

static volatile H26_SystemState_t s_systemState = H26_SYS_SELECT;
static volatile H26_TaskId_t s_selectedTask = H26_TASK_2;

static void H26_SafeStop(void)
{
    App_Control_ForcePWMZero();
}

static void H26_ClearEncoderAndControlData(void)
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
    if (primask == 0U)
    {
        __enable_irq();
    }

    App_Control_ResetPID();
    App_Line_ResetState();
}

static void H26_ClearRunData(void)
{
    H26_SafeStop();
    H26_ClearEncoderAndControlData();
    H26_Task2_Reset();
}

static uint8_t H26_IsTaskIdValid(H26_TaskId_t task)
{
    return (task >= H26_TASK_2 && task <= H26_TASK_6) ? 1U : 0U;
}

static H26_LedMode_t H26_GetStateLedRestoreMode(void)
{
    if (s_systemState == H26_SYS_STOPPED || s_systemState == H26_SYS_FAULT)
    {
        return H26_LED_FAULT;
    }
    if (s_systemState == H26_SYS_FINISHED)
    {
        return H26_LED_FINISHED;
    }
    return H26_LED_OFF;
}

static void H26_ShowSelectedTask(void)
{
    H26_LedShowTask((uint8_t)s_selectedTask, H26_GetStateLedRestoreMode());
}

static void H26_EnterFault(const char *reason)
{
    H26_SafeStop();
    H26_Task2_ForceFault();
    s_systemState = H26_SYS_FAULT;
    H26_LedSetMode(H26_LED_FAULT);
    DebugSerial_Printf("[h26,fault,%s]\r\n", reason);
}

static void H26_SelectNextTask(void)
{
    if (s_selectedTask >= H26_TASK_6)
    {
        s_selectedTask = H26_TASK_2;
    }
    else
    {
        s_selectedTask = (H26_TaskId_t)((uint8_t)s_selectedTask + 1U);
    }

    if (s_systemState == H26_SYS_FINISHED)
    {
        H26_ClearRunData();
        s_systemState = H26_SYS_SELECT;
    }
    else
    {
        H26_SafeStop();
    }

    H26_ShowSelectedTask();
    DebugSerial_Printf("[h26,select,task=%u]\r\n", (unsigned int)s_selectedTask);
}

static void H26_StartCurrentTask(void)
{
    uint32_t nowMs;

    if (H26_IsTaskIdValid(s_selectedTask) == 0U)
    {
        H26_EnterFault("task-id");
        return;
    }

    if (s_selectedTask != H26_TASK_2)
    {
        H26_ClearRunData();
        s_systemState = H26_SYS_SELECT;
        H26_LedSetMode(H26_LED_OFF);
        H26_ShowSelectedTask();
        DebugSerial_Printf("[h26,task=%u,not-implemented]\r\n",
                           (unsigned int)s_selectedTask);
        return;
    }

    H26_SafeStop();
    H26_ClearEncoderAndControlData();
    nowMs = Timer_GetMillis();
    H26_Task2_Start(nowMs);
    s_systemState = H26_SYS_PREPARE;
    H26_LedSetMode(H26_LED_RUNNING);
    DebugSerial_Printf("[h26,start,task=2,ms=%lu]\r\n", (unsigned long)nowMs);
}

void H26_Init(void)
{
    s_selectedTask = H26_TASK_2;
    s_systemState = H26_SYS_SELECT;
    H26_ClearRunData();
    H26_LedInit();
    H26_ShowSelectedTask();
    DebugSerial_Printf("[h26,init,task=2]\r\n");
}

void H26_Tick1ms(void)
{
    H26_LedTick1ms();
}

void H26_KeyProcess(void)
{
    uint8_t key = Key_GetNum();

    if (key == 0U)
    {
        return;
    }

    if (key == 3U)
    {
        H26_SafeStop();
        s_systemState = H26_SYS_STOPPED;
        H26_LedSetMode(H26_LED_FAULT);
        DebugSerial_Printf("[h26,manual-stop]\r\n");
        return;
    }

    if (key == 4U)
    {
        H26_ClearRunData();
        s_systemState = H26_SYS_SELECT;
        H26_LedSetMode(H26_LED_OFF);
        H26_ShowSelectedTask();
        DebugSerial_Printf("[h26,reset,task=%u]\r\n", (unsigned int)s_selectedTask);
        return;
    }

    if (key == 1U)
    {
        if (s_systemState == H26_SYS_SELECT ||
            s_systemState == H26_SYS_FINISHED ||
            s_systemState == H26_SYS_STOPPED)
        {
            H26_SelectNextTask();
        }
        return;
    }

    if (key == 2U &&
        (s_systemState == H26_SYS_SELECT || s_systemState == H26_SYS_FINISHED))
    {
        H26_StartCurrentTask();
    }
}

void H26_Task10ms(void)
{
    H26_Task2Result_t task2Result;
    uint32_t nowMs = Timer_GetMillis();

    switch (s_systemState)
    {
    case H26_SYS_SELECT:
    case H26_SYS_FINISHED:
    case H26_SYS_STOPPED:
    case H26_SYS_FAULT:
        H26_SafeStop();
        break;

    case H26_SYS_PREPARE:
        H26_SafeStop();
        if (s_selectedTask != H26_TASK_2 || H26_Task2_GetState() != H26_T2_LEAVE_A)
        {
            H26_EnterFault("prepare");
        }
        else
        {
            s_systemState = H26_SYS_RUNNING;
        }
        break;

    case H26_SYS_RUNNING:
        if (s_selectedTask != H26_TASK_2)
        {
            H26_EnterFault("run-task");
            break;
        }

        task2Result = H26_Task2_Task10ms(nowMs);
        if (task2Result == H26_T2_RESULT_FINISHED)
        {
            H26_SafeStop();
            s_systemState = H26_SYS_FINISHED;
            H26_LedSetMode(H26_LED_FINISHED);
            DebugSerial_Printf("[h26,finish,ms=%lu]\r\n",
                (unsigned long)H26_Task2_GetFinalElapsedMs());
        }
        else if (task2Result == H26_T2_RESULT_FAULT)
        {
            H26_EnterFault("task2");
        }
        break;

    default:
        H26_EnterFault("sys-state");
        break;
    }
}

void H26_Task100ms(void)
{
    static uint16_t debugMs = 0U;
    uint32_t elapsedMs;
    int32_t distanceCentiCm;
    int32_t leftSpeedCenti;
    int32_t rightSpeedCenti;
    int32_t commandSpeedCenti;

    debugMs = (debugMs > (uint16_t)(0xFFFFU - 100U)) ? 0xFFFFU :
        (uint16_t)(debugMs + 100U);
    if (debugMs < H26_DEBUG_PERIOD_MS)
    {
        return;
    }
    debugMs = 0U;

    elapsedMs = H26_Task2_GetElapsedMs(Timer_GetMillis());
    distanceCentiCm = (int32_t)(H26_Task2_GetDistanceCm() * 100.0f);
    leftSpeedCenti = (int32_t)(g_leftSpeed * 100.0f);
    rightSpeedCenti = (int32_t)(g_rightSpeed * 100.0f);
    commandSpeedCenti = (int32_t)(H26_Task2_GetCommandForwardSpeed() * 100.0f);

    DebugSerial_Printf(
        "[h26,sys=%u,task=%u,t2=%u,zone=%u,curve=%u,finish_slow=%u,cmd_v_c=%ld,"
        "mask=%02X,black=%u,hold=%u,fen=%u,flat=%u,dist_c=%ld,elapsed=%lu,"
        "vl_c=%ld,vr_c=%ld,fdet=%lu,fp=%ld,final=%lu]\r\n",
        (unsigned int)s_systemState,
        (unsigned int)s_selectedTask,
        (unsigned int)H26_Task2_GetState(),
        (unsigned int)H26_Task2_GetSpeedZone(),
        (unsigned int)H26_Task2_IsCurveMode(),
        (unsigned int)H26_Task2_IsFinishApproach(),
        (long)commandSpeedCenti,
        (unsigned int)g_lineMask,
        (unsigned int)g_lineBlackCount,
        (unsigned int)H26_Task2_GetBlackHoldMs(),
        (unsigned int)H26_Task2_IsFinishEnabled(),
        (unsigned int)H26_Task2_IsFinishLatched(),
        (long)distanceCentiCm,
        (unsigned long)elapsedMs,
        (long)leftSpeedCenti,
        (long)rightSpeedCenti,
        (unsigned long)H26_Task2_GetFinishDetectMs(),
        (long)H26_Task2_GetFinishDetectPulse(),
        (unsigned long)H26_Task2_GetFinalElapsedMs());
}

void H26_Task200ms(void)
{
#if CAR_OLED_ENABLE
    uint32_t elapsedMs = H26_Task2_GetElapsedMs(Timer_GetMillis());
    uint32_t seconds = elapsedMs / 1000U;
    uint32_t centiseconds = (elapsedMs % 1000U) / 10U;

    OLED_Clear();
    if (s_systemState == H26_SYS_FINISHED)
    {
        OLED_ShowString(0, 0, "TASK 2 FINISH", OLED_6X8);
        OLED_ShowString(0, 16, "STOPPED", OLED_6X8);
    }
    else
    {
        OLED_ShowString(0, 0, "TASK ", OLED_6X8);
        OLED_ShowNum(36, 0, (uint32_t)s_selectedTask, 1, OLED_6X8);
        OLED_ShowString(0, 16, "SYS:", OLED_6X8);
        OLED_ShowNum(24, 16, (uint32_t)s_systemState, 1, OLED_6X8);
        OLED_ShowString(48, 16, "T2:", OLED_6X8);
        OLED_ShowNum(66, 16, (uint32_t)H26_Task2_GetState(), 1, OLED_6X8);
    }
    OLED_ShowString(0, 32, "TIME:", OLED_6X8);
    OLED_ShowNum(36, 32, seconds, 2, OLED_6X8);
    OLED_ShowString(48, 32, ".", OLED_6X8);
    OLED_ShowNum(54, 32, centiseconds, 2, OLED_6X8);
    OLED_ShowString(72, 32, "s", OLED_6X8);
    OLED_ShowString(0, 48, "BLK:", OLED_6X8);
    OLED_ShowNum(30, 48, (uint32_t)g_lineBlackCount, 1, OLED_6X8);
    OLED_ShowString(48, 48, "DST:", OLED_6X8);
    OLED_ShowNum(72, 48, (uint32_t)H26_Task2_GetDistanceCm(), 3, OLED_6X8);
    OLED_Update();
#endif
}

H26_SystemState_t H26_GetSystemState(void)
{
    return s_systemState;
}

H26_TaskId_t H26_GetSelectedTask(void)
{
    return s_selectedTask;
}
