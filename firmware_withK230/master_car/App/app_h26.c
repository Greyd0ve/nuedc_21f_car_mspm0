#include "app_h26.h"
#include "app_h26_config.h"
#include "app_h26_led.h"
#include "app_h26_task2.h"
#include "app_h26_task3.h"
#include "app_h26_task4.h"
#include "app_h26_task5.h"
#include "app_ball_link.h"
#include "app_car_state.h"
#include "app_control.h"
#include "app_line.h"
#include "DebugSerial.h"
#include "Encoder.h"
#include "Key.h"
#include "OLED.h"
#include "RodEncoder.h"
#include "RodStepper.h"
#include "Serial.h"
#include "Timer.h"
#include "cmsis_compiler.h"
#include <stdint.h>

static volatile H26_SystemState_t s_systemState = H26_SYS_SELECT;
static volatile H26_TaskId_t s_selectedTask = H26_TASK_2;

static uint32_t H26_AbsInt32ToU32(int32_t value)
{
    return (value < 0) ? ((uint32_t)(-(value + 1)) + 1U) : (uint32_t)value;
}

static void H26_OledShowSignedCentiCm(uint8_t x, uint8_t y, int32_t centiCm)
{
    OLED_ShowString(x, y, (centiCm < 0) ? "-" : "+", OLED_6X8);
    OLED_ShowNum((uint8_t)(x + 6U), y, H26_AbsInt32ToU32(centiCm), 4, OLED_6X8);
}

static void H26_SafeCarStop(void)
{
    App_Control_ForcePWMZero();
}

static void H26_SafeStop(void)
{
    H26_SafeCarStop();
    RodStepper_Stop();
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
    H26_Task3_Reset();
    H26_Task4_Reset();
    H26_Task5_Reset();
    App_BallLink_Reset();
    RodEncoder_Reset();
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
    H26_Task3_ForceFault();
    H26_Task4_ForceFault();
    H26_Task5_ForceFault();
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

static void H26_StartCurrentTask(uint8_t task4DriveMode)
{
    uint32_t nowMs;

    if (H26_IsTaskIdValid(s_selectedTask) == 0U)
    {
        H26_EnterFault("task-id");
        return;
    }

    if (s_selectedTask != H26_TASK_2 && s_selectedTask != H26_TASK_3 &&
        s_selectedTask != H26_TASK_4 && s_selectedTask != H26_TASK_5)
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
    if (s_selectedTask == H26_TASK_2)
    {
        H26_Task2_Start(nowMs);
    }
    else if (s_selectedTask == H26_TASK_3)
    {
        H26_Task3_Start(nowMs);
    }
    else if (s_selectedTask == H26_TASK_4)
    {
        if (task4DriveMode != 0U)
        {
            H26_Task4_StartDrive(nowMs);
        }
        else
        {
            H26_Task4_Start(nowMs);
        }
    }
    else
    {
        H26_Task5_Start(nowMs);
    }
    s_systemState = H26_SYS_PREPARE;
    H26_LedSetMode(H26_LED_RUNNING);
    DebugSerial_Printf("[h26,start,task=%u,mode=%u,ms=%lu]\r\n",
        (unsigned int)s_selectedTask, (unsigned int)task4DriveMode,
        (unsigned long)nowMs);
}

void H26_Init(void)
{
    s_selectedTask = H26_TASK_2;
    s_systemState = H26_SYS_SELECT;
    H26_Task2_Init();
    H26_Task3_Init();
    H26_Task4_Init();
    H26_Task5_Init();
    App_BallLink_Init();
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
        if (s_selectedTask == H26_TASK_4 &&
            (s_systemState == H26_SYS_SELECT ||
             s_systemState == H26_SYS_FINISHED))
        {
            H26_StartCurrentTask(0U);
            return;
        }

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
        H26_StartCurrentTask((s_selectedTask == H26_TASK_4) ? 1U : 0U);
    }
}

void H26_Task10ms(void)
{
    H26_Task2Result_t task2Result;
    H26_Task3Result_t task3Result;
    H26_Task4Result_t task4Result;
    H26_Task5Result_t task5Result;
    uint32_t nowMs = Timer_GetMillis();

    switch (s_systemState)
    {
    case H26_SYS_SELECT:
    case H26_SYS_STOPPED:
    case H26_SYS_FAULT:
        H26_SafeStop();
        break;

    case H26_SYS_FINISHED:
        if (s_selectedTask == H26_TASK_3)
        {
            /* Task 3 has completed its final +5 cm PID settle. */
            H26_SafeCarStop();
            task3Result = H26_Task3_Task10ms(nowMs);
            if (task3Result == H26_T3_RESULT_FAULT)
            {
                H26_EnterFault("task3-hold");
            }
        }
        else if (s_selectedTask == H26_TASK_4)
        {
            H26_SafeCarStop();
            H26_Task4_HoldBall10ms(nowMs);
        }
        else if (s_selectedTask == H26_TASK_5)
        {
            /* Lap time is frozen; retain active O-point ball stabilization. */
            H26_SafeCarStop();
            H26_Task5_HoldBall10ms(nowMs);
        }
        else
        {
            H26_SafeStop();
        }
        break;

    case H26_SYS_PREPARE:
        H26_SafeStop();
        if ((s_selectedTask == H26_TASK_2 &&
             H26_Task2_GetState() == H26_T2_LEAVE_A) ||
            (s_selectedTask == H26_TASK_3 &&
             H26_Task3_GetState() == H26_T3_READY) ||
             (s_selectedTask == H26_TASK_4 &&
               (H26_Task4_GetState() == H26_T4_MANUAL_MOVE_HOLD_O ||
                H26_Task4_GetState() == H26_T4_DRIVE_ACQUIRE_O)) ||
            (s_selectedTask == H26_TASK_5 &&
             H26_Task5_GetState() == H26_T5_ACQUIRE_O))
        {
            s_systemState = H26_SYS_RUNNING;
        }
        else
        {
            H26_EnterFault("prepare");
        }
        break;

    case H26_SYS_RUNNING:
        if (s_selectedTask == H26_TASK_2)
        {
            task2Result = H26_Task2_Task10ms(nowMs);
            if (task2Result == H26_T2_RESULT_FINISHED)
            {
                H26_SafeStop();
                s_systemState = H26_SYS_FINISHED;
                H26_LedSetMode(H26_LED_FINISHED);
                DebugSerial_Printf("[h26,finish,task=2,ms=%lu]\r\n",
                    (unsigned long)H26_Task2_GetFinalElapsedMs());
            }
            else if (task2Result == H26_T2_RESULT_FAULT)
            {
                H26_EnterFault("task2");
            }
        }
        else if (s_selectedTask == H26_TASK_3)
        {
            /* Task 3 owns only the rod stepper; vehicle traction stays forced off. */
            H26_SafeCarStop();
            task3Result = H26_Task3_Task10ms(nowMs);
            if (task3Result == H26_T3_RESULT_FINISHED)
            {
                H26_SafeCarStop();
                s_systemState = H26_SYS_FINISHED;
                H26_LedSetMode(H26_LED_FINISHED);
                DebugSerial_Printf("[t3f,%lu]\r\n",
                    (unsigned long)H26_Task3_GetFinalElapsedMs());
            }
            else if (task3Result == H26_T3_RESULT_FAULT)
            {
                DebugSerial_Printf("[t3e,%u]\r\n",
                    (unsigned int)H26_Task3_GetFault());
                H26_EnterFault("task3");
            }
        }
        else if (s_selectedTask == H26_TASK_4)
        {
            task4Result = H26_Task4_Task10ms(nowMs);
            if (task4Result == H26_T4_RESULT_FINISHED)
            {
                H26_SafeStop();
                s_systemState = H26_SYS_FINISHED;
                H26_LedSetMode(H26_LED_FINISHED);
                DebugSerial_Printf("[h26,finish,task=4]\r\n");
            }
            else if (task4Result == H26_T4_RESULT_FAULT)
            {
                DebugSerial_Printf("[h26,task4-fault,code=%u]\r\n",
                    (unsigned int)H26_Task4_GetFault());
                H26_EnterFault("task4");
            }
        }
        else if (s_selectedTask == H26_TASK_5)
        {
            task5Result = H26_Task5_Task10ms(nowMs);
            if (task5Result == H26_T5_RESULT_FINISHED)
            {
                H26_SafeStop();
                s_systemState = H26_SYS_FINISHED;
                H26_LedSetMode(H26_LED_FINISHED);
                DebugSerial_Printf("[h26,finish,task=5,ms=%lu,peak_c=%ld]\r\n",
                    (unsigned long)H26_Task5_GetFinalElapsedMs(),
                    (long)(H26_Task5_GetBallPeakErrorCm() * 100.0f));
            }
            else if (task5Result == H26_T5_RESULT_FAULT)
            {
                DebugSerial_Printf("[h26,task5-fault,code=%u]\r\n",
                    (unsigned int)H26_Task5_GetFault());
                H26_EnterFault("task5");
            }
        }
        else
        {
            H26_EnterFault("run-task");
        }
        break;

    default:
        H26_EnterFault("sys-state");
        break;
    }
}

void H26_Task100ms(void)
{
#if !H26_VERBOSE_TELEMETRY_ENABLE
    static uint16_t debugMs = 0U;

    debugMs = (debugMs > (uint16_t)(0xFFFFU - 100U)) ? 0xFFFFU :
        (uint16_t)(debugMs + 100U);
    if (debugMs < H26_DEBUG_PERIOD_MS)
    {
        return;
    }
    debugMs = 0U;

    if (s_selectedTask == H26_TASK_3)
    {
        DebugSerial_Printf(
            "[t3,%u,%u,%lu,%lu,%u]\r\n",
            (unsigned int)s_systemState,
            (unsigned int)H26_Task3_GetState(),
            (unsigned long)H26_Task3_GetElapsedMs(Timer_GetMillis()),
            (unsigned long)H26_Task3_GetFinalElapsedMs(),
            (unsigned int)H26_Task3_GetFault());
        return;
    }

    if (s_selectedTask == H26_TASK_4)
    {
        DebugSerial_Printf(
            "[h26,t4=%u,p=%ld,e=%ld,ff=%ld,t=%lu,f=%u]\r\n",
            (unsigned int)H26_Task4_GetState(),
            (long)(H26_Task4_GetBallPositionCm() * 100.0f),
            (long)(H26_Task4_GetBallErrorCm() * 100.0f),
            (long)(H26_Task4_GetFeedForwardTiltMm() * 1000.0f),
            (unsigned long)H26_Task4_GetElapsedMs(Timer_GetMillis()),
            (unsigned int)H26_Task4_GetFault());
        return;
    }

    if (s_selectedTask == H26_TASK_5)
    {
        DebugSerial_Printf(
            "[h26,sys=%u,task=5,t5=%u,dist_c=%ld,pos_c=%ld,peak_c=%ld,elapsed=%lu,final=%lu,fault=%u]\r\n",
            (unsigned int)s_systemState,
            (unsigned int)H26_Task5_GetState(),
            (long)(H26_Task5_GetDistanceCm() * 100.0f),
            (long)(H26_Task5_GetBallPositionCm() * 100.0f),
            (long)(H26_Task5_GetBallPeakErrorCm() * 100.0f),
            (unsigned long)H26_Task5_GetElapsedMs(Timer_GetMillis()),
            (unsigned long)H26_Task5_GetFinalElapsedMs(),
            (unsigned int)H26_Task5_GetFault());
        return;
    }

    DebugSerial_Printf(
        "[h26,sys=%u,task=2,t2=%u,dist_c=%ld,cmd_v_c=%ld,mask=%02X,black=%u,elapsed=%lu,final=%lu]\r\n",
        (unsigned int)s_systemState,
        (unsigned int)H26_Task2_GetState(),
        (long)(H26_Task2_GetDistanceCm() * 100.0f),
        (long)(H26_Task2_GetCommandForwardSpeed() * 100.0f),
        (unsigned int)g_lineMask,
        (unsigned int)g_lineBlackCount,
        (unsigned long)H26_Task2_GetElapsedMs(Timer_GetMillis()),
        (unsigned long)H26_Task2_GetFinalElapsedMs());
#else
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

    if (s_selectedTask == H26_TASK_3)
    {
        DebugSerial_Printf(
            "[t3,%u,%u,%lu,%lu,%u]\r\n",
            (unsigned int)s_systemState,
            (unsigned int)H26_Task3_GetState(),
            (unsigned long)H26_Task3_GetElapsedMs(Timer_GetMillis()),
            (unsigned long)H26_Task3_GetFinalElapsedMs(),
            (unsigned int)H26_Task3_GetFault());
        return;
    }

    if (s_selectedTask == H26_TASK_4)
    {
        elapsedMs = H26_Task4_GetElapsedMs(Timer_GetMillis());

        DebugSerial_Printf(
            "[h26,sys=%u,task=4,t4=%u,dist_c=%ld,cmd_v_c=%ld,turn_c=%ld,pos_c=%ld,err_c=%ld,vel_c=%ld,car_v_c=%ld,car_a_c=%ld,pid_um=%ld,ff_um=%ld,tilt_um=%ld,"
            "rod=%ld,rod_t=%ld,vis=%u,org=%u,conf=%u,elapsed=%lu,fault=%u]\r\n",
            (unsigned int)s_systemState,
            (unsigned int)H26_Task4_GetState(),
            (long)(H26_Task4_GetDistanceCm() * 100.0f),
            (long)(H26_Task4_GetCommandForwardSpeedCmps() * 100.0f),
            (long)(g_targetTurnSpeed * 100.0f),
            (long)(H26_Task4_GetBallPositionCm() * 100.0f),
            (long)(H26_Task4_GetBallErrorCm() * 100.0f),
            (long)(H26_Task4_GetBallSpeedCmps() * 100.0f),
            (long)(H26_Task4_GetForwardSpeedCmps() * 100.0f),
            (long)(H26_Task4_GetForwardAccelerationCmps2() * 100.0f),
            (long)(H26_Task4_GetPidTiltCommandMm() * 1000.0f),
            (long)(H26_Task4_GetFeedForwardTiltMm() * 1000.0f),
            (long)(H26_Task4_GetTiltCommandMm() * 1000.0f),
            (long)H26_Task4_GetRodEncoderCount(),
            (long)H26_Task4_GetRodTargetCount(),
            (unsigned int)H26_Task4_IsVisionValid(),
            (unsigned int)H26_Task4_IsOriginCalibrated(),
            (unsigned int)H26_Task4_GetVisionConfidence(),
            (unsigned long)elapsedMs,
            (unsigned int)H26_Task4_GetFault());
        return;
    }

    elapsedMs = H26_Task2_GetElapsedMs(Timer_GetMillis());
    distanceCentiCm = (int32_t)(H26_Task2_GetDistanceCm() * 100.0f);
    leftSpeedCenti = (int32_t)(g_leftSpeed * 100.0f);
    rightSpeedCenti = (int32_t)(g_rightSpeed * 100.0f);
    commandSpeedCenti = (int32_t)(H26_Task2_GetCommandForwardSpeed() * 100.0f);

    DebugSerial_Printf(
        "[h26,s=%u,task=%u,t2=%u,curve=%u,v_c=%ld,mask=%02X,black=%u,"
        "exit=%u,finish=%u,dist_c=%ld,t=%lu,vl_c=%ld,vr_c=%ld]\r\n",
        (unsigned int)s_systemState,
        (unsigned int)s_selectedTask,
        (unsigned int)H26_Task2_GetState(),
        (unsigned int)H26_Task2_IsCurveMode(),
        (long)commandSpeedCenti,
        (unsigned int)g_lineMask,
        (unsigned int)g_lineBlackCount,
        (unsigned int)H26_Task2_IsFinishEnabled(),
        (unsigned int)H26_Task2_IsFinishLatched(),
        (long)distanceCentiCm,
        (unsigned long)elapsedMs,
        (long)leftSpeedCenti,
        (long)rightSpeedCenti);
#endif
}

void H26_Task200ms(void)
{
#if CAR_OLED_ENABLE
    uint32_t elapsedMs;
    uint32_t seconds;
    uint32_t centiseconds;

    if (s_selectedTask == H26_TASK_3)
    {
        elapsedMs = H26_Task3_GetElapsedMs(Timer_GetMillis());
    }
    else if (s_selectedTask == H26_TASK_4)
    {
        elapsedMs = H26_Task4_GetElapsedMs(Timer_GetMillis());
    }
    else if (s_selectedTask == H26_TASK_5)
    {
        elapsedMs = H26_Task5_GetElapsedMs(Timer_GetMillis());
    }
    else
    {
        elapsedMs = H26_Task2_GetElapsedMs(Timer_GetMillis());
    }
    seconds = elapsedMs / 1000U;
    centiseconds = (elapsedMs % 1000U) / 10U;

    OLED_Clear();
    if (s_systemState == H26_SYS_FINISHED)
    {
        OLED_ShowString(0, 0, (s_selectedTask == H26_TASK_3) ?
                        "TASK 3 FINISH" :
                        ((s_selectedTask == H26_TASK_4) ? "TASK 4 FINISH" :
                         ((s_selectedTask == H26_TASK_5) ? "TASK 5 FINISH" :
                          "TASK 2 FINISH")), OLED_6X8);
        OLED_ShowString(0, 16, "STOPPED", OLED_6X8);
    }
    else
    {
        OLED_ShowString(0, 0, "TASK ", OLED_6X8);
        OLED_ShowNum(36, 0, (uint32_t)s_selectedTask, 1, OLED_6X8);
        OLED_ShowString(0, 16, "SYS:", OLED_6X8);
        OLED_ShowNum(24, 16, (uint32_t)s_systemState, 1, OLED_6X8);
        OLED_ShowString(48, 16, (s_selectedTask == H26_TASK_3) ? "T3:" :
                        ((s_selectedTask == H26_TASK_4) ? "T4:" :
                         ((s_selectedTask == H26_TASK_5) ? "T5:" : "T2:")), OLED_6X8);
        OLED_ShowNum(66, 16,
            (s_selectedTask == H26_TASK_3) ? (uint32_t)H26_Task3_GetState() :
            ((s_selectedTask == H26_TASK_4) ? (uint32_t)H26_Task4_GetState() :
             ((s_selectedTask == H26_TASK_5) ? (uint32_t)H26_Task5_GetState() :
              (uint32_t)H26_Task2_GetState())), 1, OLED_6X8);
    }
    OLED_ShowString(0, 32, "TIME:", OLED_6X8);
    OLED_ShowNum(36, 32, seconds, 2, OLED_6X8);
    OLED_ShowString(48, 32, ".", OLED_6X8);
    OLED_ShowNum(54, 32, centiseconds, 2, OLED_6X8);
    OLED_ShowString(72, 32, "s", OLED_6X8);
    if (s_selectedTask == H26_TASK_3)
    {
        OLED_ShowString(0, 48, "FINAL PID +5", OLED_6X8);
    }
    else if (s_selectedTask == H26_TASK_4)
    {
        OLED_ShowString(0, 48, "P:", OLED_6X8);
        H26_OledShowSignedCentiCm(12, 48,
            (int32_t)(H26_Task4_GetBallPositionCm() * 100.0f));
        OLED_ShowString(48, 48, "E:", OLED_6X8);
        H26_OledShowSignedCentiCm(60, 48,
            (int32_t)(H26_Task4_GetBallErrorCm() * 100.0f));
    }
    else if (s_selectedTask == H26_TASK_5)
    {
        OLED_ShowString(0, 48, "P:", OLED_6X8);
        H26_OledShowSignedCentiCm(12, 48,
            (int32_t)(H26_Task5_GetBallPositionCm() * 100.0f));
        OLED_ShowString(42, 48, "DST:", OLED_6X8);
        OLED_ShowNum(66, 48, (uint32_t)H26_Task5_GetDistanceCm(), 3, OLED_6X8);
    }
    else
    {
        OLED_ShowString(0, 48, "BLK:", OLED_6X8);
        OLED_ShowNum(30, 48, (uint32_t)g_lineBlackCount, 1, OLED_6X8);
        OLED_ShowString(48, 48, "DST:", OLED_6X8);
        OLED_ShowNum(72, 48, (uint32_t)H26_Task2_GetDistanceCm(), 3, OLED_6X8);
    }
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
