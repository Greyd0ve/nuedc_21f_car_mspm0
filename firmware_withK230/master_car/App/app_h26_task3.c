#include "app_h26_task3.h"
#include "app_h26_ball_control.h"
#include "app_h26_config.h"
#include "RodStepper.h"
#include <stdint.h>

static volatile H26_Task3State_t s_state = H26_T3_IDLE;
static volatile H26_Task3Fault_t s_fault = H26_T3_FAULT_NONE;
static volatile uint32_t s_startMs = 0U;
static volatile uint32_t s_finalElapsedMs = 0U;
static volatile uint32_t s_phaseStartMs = 0U;
static volatile uint16_t s_stableMs = 0U;

static float H26_T3_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint16_t H26_T3_AddMs(uint16_t value, uint16_t deltaMs)
{
    return (value > (uint16_t)(0xFFFFU - deltaMs)) ? 0xFFFFU :
        (uint16_t)(value + deltaMs);
}

static RodStepperDirection_t H26_T3_GetExtendDirection(void)
{
    return (H26_T3_CHAIN_EXTEND_DIR_POSITIVE != 0U) ?
        ROD_STEPPER_DIR_POSITIVE : ROD_STEPPER_DIR_NEGATIVE;
}

static RodStepperDirection_t H26_T3_GetRetractDirection(void)
{
    return (H26_T3_CHAIN_EXTEND_DIR_POSITIVE != 0U) ?
        ROD_STEPPER_DIR_NEGATIVE : ROD_STEPPER_DIR_POSITIVE;
}

static void H26_T3_EnterFault(H26_Task3Fault_t fault)
{
    H26_BallControl_Stop();
    s_fault = fault;
    s_state = H26_T3_FAULT;
}

void H26_Task3_Init(void)
{
    H26_Task3_Reset();
}

void H26_Task3_Reset(void)
{
    H26_BallControl_Reset();
    s_state = H26_T3_IDLE;
    s_fault = H26_T3_FAULT_NONE;
    s_startMs = 0U;
    s_finalElapsedMs = 0U;
    s_phaseStartMs = 0U;
    s_stableMs = 0U;
}

void H26_Task3_Start(uint32_t startMs)
{
    H26_Task3_Reset();
    H26_BallControl_Start();
    s_startMs = startMs;
    s_state = H26_T3_READY;
}

void H26_Task3_ForceFault(void)
{
    H26_T3_EnterFault((s_fault == H26_T3_FAULT_NONE) ?
        H26_T3_FAULT_ILLEGAL_STATE : s_fault);
}

static void H26_T3_RunPid(uint32_t nowMs,
                           float targetCm,
                           float toleranceCm,
                           float kd,
                           uint16_t stableMsTarget,
                           H26_Task3State_t nextState)
{
    H26_BallControlSample_t sample;

    sample = H26_BallControl_Task10msWithPidFeedForward(nowMs,
        targetCm,
        H26_T3_FINAL_PID_KP_MM_PER_CM,
        H26_T3_FINAL_PID_KI_MM_PER_CM_S,
        kd,
        H26_T3_FINAL_PID_INTEGRAL_LIMIT_CM_S,
        H26_T3_FINAL_PID_TILT_LIMIT_MM,
        0.0f,
        H26_T3_TILT_DEADBAND_CM);

    if (sample == H26_BALL_SAMPLE_NEW &&
        H26_T3_AbsFloat(H26_BallControl_GetErrorCm()) <= toleranceCm)
    {
        s_stableMs = H26_T3_AddMs(s_stableMs,
            H26_BallControl_GetStableSampleMs());
        if (s_stableMs >= stableMsTarget)
        {
            s_state = nextState;
        }
    }
    else if (sample == H26_BALL_SAMPLE_NEW)
    {
        s_stableMs = 0U;
    }
}

static void H26_T3_RunPidForever(uint32_t nowMs, float targetCm)
{
    (void)H26_BallControl_Task10msWithPidFeedForward(nowMs,
        targetCm,
        H26_T3_FINAL_PID_KP_MM_PER_CM,
        H26_T3_FINAL_PID_KI_MM_PER_CM_S,
        H26_T3_FINAL_PID_KD_MM_PER_CMPS,
        H26_T3_FINAL_PID_INTEGRAL_LIMIT_CM_S,
        H26_T3_FINAL_PID_TILT_LIMIT_MM,
        0.0f,
        H26_T3_TILT_DEADBAND_CM);
}

static void H26_T3_StartMove(H26_Task3State_t moveState,
                              RodStepperDirection_t dir,
                              uint32_t pulses,
                              uint32_t freqHz)
{
    if (RodStepper_MovePulses(dir, pulses, freqHz) != 0U)
    {
        s_state = moveState;
    }
    else
    {
        H26_T3_EnterFault(H26_T3_FAULT_STEPPER_OUTPUT);
    }
}

H26_Task3Result_t H26_Task3_Task10ms(uint32_t nowMs)
{
    if (s_state != H26_T3_FINAL_PID_STAGE1 &&
        s_state != H26_T3_FINAL_PID_STAGE2 &&
        s_state != H26_T3_FINAL_PID_STAGE3)
    {
        (void)H26_BallControl_Observe10ms(nowMs);
    }

    switch (s_state)
    {
    case H26_T3_READY:
        H26_T3_StartMove(H26_T3_EXTEND_10MM,
            H26_T3_GetExtendDirection(),
            H26_T3_EXTEND_10MM_PULSES,
            H26_T3_EXTEND_10MM_STEP_HZ);
        break;

    case H26_T3_EXTEND_10MM:
        if (RodStepper_TakeCompletionEvent() != 0U)
        {
            RodStepper_Stop();
            s_phaseStartMs = nowMs;
            s_state = H26_T3_HOLD_TO_STAGE1;
        }
        break;

    case H26_T3_HOLD_TO_STAGE1:
        if ((nowMs - s_phaseStartMs) >= H26_T3_HOLD_TO_STAGE1_MS)
        {
            s_phaseStartMs = nowMs;
            s_stableMs = 0U;
            s_state = H26_T3_FINAL_PID_STAGE1;
        }
        break;

    case H26_T3_FINAL_PID_STAGE1:
        H26_T3_RunPid(nowMs,
            H26_T3_PID_STAGE1_TARGET_CM,
            H26_T3_PID_STAGE1_TOLERANCE_CM,
            H26_T3_FINAL_PID_KD_MM_PER_CMPS,
            H26_T3_PID_STAGE1_STABLE_MS,
            H26_T3_HOLD_TO_RETRACT);
        if (s_state == H26_T3_HOLD_TO_RETRACT)
        {
            s_phaseStartMs = nowMs;
        }
        break;

    case H26_T3_HOLD_TO_RETRACT:
        if ((nowMs - s_phaseStartMs) >= H26_T3_HOLD_TO_RETRACT_MS)
        {
            H26_T3_StartMove(H26_T3_RETRACT_10MM,
                H26_T3_GetRetractDirection(),
                H26_T3_RETRACT_10MM_PULSES,
                H26_T3_RETRACT_10MM_STEP_HZ);
        }
        break;

    case H26_T3_RETRACT_10MM:
        if (RodStepper_TakeCompletionEvent() != 0U)
        {
            RodStepper_Stop();
            s_phaseStartMs = nowMs;
            s_state = H26_T3_HOLD_TO_STAGE2;
        }
        break;

    case H26_T3_HOLD_TO_STAGE2:
        if ((nowMs - s_phaseStartMs) >= H26_T3_HOLD_TO_STAGE2_MS)
        {
            s_phaseStartMs = nowMs;
            s_stableMs = 0U;
            s_state = H26_T3_FINAL_PID_STAGE2;
        }
        break;

    case H26_T3_FINAL_PID_STAGE2:
        H26_T3_RunPid(nowMs,
            H26_T3_PID_STAGE2_TARGET_CM,
            H26_T3_PID_STAGE2_TOLERANCE_CM,
            H26_T3_FINAL_PID_KD_MM_PER_CMPS,
            H26_T3_PID_STAGE2_STABLE_MS,
            H26_T3_DONE);
        if (s_state == H26_T3_DONE)
        {
            s_finalElapsedMs = nowMs - s_startMs;
            H26_BallControl_Stop();
            s_phaseStartMs = nowMs;
            s_state = H26_T3_FINAL_PID_STAGE3;
        }
        break;

    case H26_T3_DONE:
        s_finalElapsedMs = nowMs - s_startMs;
        H26_BallControl_Stop();
        s_phaseStartMs = nowMs;
        s_state = H26_T3_FINAL_PID_STAGE3;
        break;

    case H26_T3_FINAL_PID_STAGE3:
        H26_T3_RunPidForever(nowMs, H26_T3_PID_STAGE2_TARGET_CM);
        return H26_T3_RESULT_RUNNING;

    case H26_T3_FAULT:
        H26_BallControl_Stop();
        return H26_T3_RESULT_FAULT;

    case H26_T3_IDLE:
    default:
        H26_T3_EnterFault(H26_T3_FAULT_ILLEGAL_STATE);
        return H26_T3_RESULT_FAULT;
    }

    return (s_state == H26_T3_FAULT) ? H26_T3_RESULT_FAULT :
        H26_T3_RESULT_RUNNING;
}

H26_Task3State_t H26_Task3_GetState(void) { return s_state; }

uint32_t H26_Task3_GetElapsedMs(uint32_t nowMs)
{
    if (s_state == H26_T3_IDLE)
    {
        return 0U;
    }
    if (s_state == H26_T3_DONE || s_state == H26_T3_FINAL_PID_STAGE3)
    {
        return s_finalElapsedMs;
    }
    return nowMs - s_startMs;
}

uint32_t H26_Task3_GetFinalElapsedMs(void) { return s_finalElapsedMs; }
H26_Task3Fault_t H26_Task3_GetFault(void) { return s_fault; }
