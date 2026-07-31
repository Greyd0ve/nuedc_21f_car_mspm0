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
static volatile uint32_t s_moveTimeoutMs = 0U;
static volatile uint16_t s_finalPidStableMs = 0U;

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

static uint8_t H26_T3_StartMove(uint32_t nowMs,
                                 RodStepperDirection_t direction,
                                 uint32_t pulses,
                                 uint32_t frequencyHz,
                                 H26_Task3State_t moveState)
{
    uint32_t travelMs;

    if (RodStepper_MovePulses(direction, pulses, frequencyHz) == 0U)
    {
        H26_T3_EnterFault(H26_T3_FAULT_STEPPER_OUTPUT);
        return 0U;
    }

    /* The timeout is a motor/output safety guard; normal transitions still
     * use the exact STEP completion event rather than elapsed time. */
    travelMs = ((pulses * 1000U) + frequencyHz - 1U) / frequencyHz;
    s_phaseStartMs = nowMs;
    s_moveTimeoutMs = travelMs + H26_T3_MOVE_TIMEOUT_MARGIN_MS;
    s_state = moveState;
    return 1U;
}

static uint8_t H26_T3_IsMoveTimedOut(uint32_t nowMs)
{
    return ((nowMs - s_phaseStartMs) > s_moveTimeoutMs) ? 1U : 0U;
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
    s_moveTimeoutMs = 0U;
    s_finalPidStableMs = 0U;
}

void H26_Task3_Start(uint32_t startMs)
{
    H26_Task3_Reset();
    /* Capture the O origin while the fixed strokes run, without allowing
     * the ball controller to alter any of those prescribed motions. */
    H26_BallControl_Start();
    s_startMs = startMs;
    /* H26_SYS_PREPARE keeps all outputs stopped.  The first pulse is issued
     * on the following RUNNING tick. */
    s_state = H26_T3_READY;
}

void H26_Task3_ForceFault(void)
{
    H26_T3_EnterFault((s_fault == H26_T3_FAULT_NONE) ?
        H26_T3_FAULT_ILLEGAL_STATE : s_fault);
}

H26_Task3Result_t H26_Task3_Task10ms(uint32_t nowMs)
{
    H26_BallControlSample_t sample;

    if (s_state != H26_T3_FINAL_PID)
    {
        (void)H26_BallControl_Observe10ms(nowMs);
    }

    switch (s_state)
    {
    case H26_T3_READY:
        (void)H26_T3_StartMove(nowMs, H26_T3_GetExtendDirection(),
            H26_T3_EXTEND_9MM_PULSES, H26_T3_CHAIN_STEP_HZ,
            H26_T3_EXTEND_9MM);
        break;

    case H26_T3_EXTEND_9MM:
        if (RodStepper_TakeCompletionEvent() != 0U)
        {
            (void)H26_T3_StartMove(nowMs, H26_T3_GetRetractDirection(),
                H26_T3_RETRACT_18MM_PULSES, H26_T3_RETRACT_18MM_STEP_HZ,
                H26_T3_RETRACT_18MM);
        }
        else if (H26_T3_IsMoveTimedOut(nowMs) != 0U)
        {
            H26_T3_EnterFault(H26_T3_FAULT_MOVE_TIMEOUT);
        }
        break;

    case H26_T3_RETRACT_18MM:
        if (RodStepper_TakeCompletionEvent() != 0U)
        {
            RodStepper_Stop();
            s_phaseStartMs = nowMs;
            s_moveTimeoutMs = 0U;
            s_state = H26_T3_HOLD_RETRACT_18MM;
        }
        else if (H26_T3_IsMoveTimedOut(nowMs) != 0U)
        {
            H26_T3_EnterFault(H26_T3_FAULT_MOVE_TIMEOUT);
        }
        break;

    case H26_T3_HOLD_RETRACT_18MM:
        if ((nowMs - s_phaseStartMs) >= H26_T3_HOLD_RETRACT_18MM_MS)
        {
            (void)H26_T3_StartMove(nowMs, H26_T3_GetExtendDirection(),
                H26_T3_EXTEND_16MM_PULSES, H26_T3_CHAIN_STEP_HZ,
                H26_T3_EXTEND_16MM);
        }
        break;

    case H26_T3_EXTEND_16MM:
        if (RodStepper_TakeCompletionEvent() != 0U)
        {
            (void)H26_T3_StartMove(nowMs, H26_T3_GetRetractDirection(),
                H26_T3_RETRACT_7MM_PULSES, H26_T3_RETRACT_7MM_STEP_HZ,
                H26_T3_RETRACT_7MM);
        }
        else if (H26_T3_IsMoveTimedOut(nowMs) != 0U)
        {
            H26_T3_EnterFault(H26_T3_FAULT_MOVE_TIMEOUT);
        }
        break;

    case H26_T3_RETRACT_7MM:
        if (RodStepper_TakeCompletionEvent() != 0U)
        {
            RodStepper_Stop();
            s_phaseStartMs = nowMs;
            s_moveTimeoutMs = 0U;
            s_finalPidStableMs = 0U;
            s_state = H26_T3_FINAL_PID;
        }
        else if (H26_T3_IsMoveTimedOut(nowMs) != 0U)
        {
            H26_T3_EnterFault(H26_T3_FAULT_MOVE_TIMEOUT);
        }
        break;

    case H26_T3_FINAL_PID:
        sample = H26_BallControl_Task10msWithPidFeedForward(nowMs,
            H26_T3_FINAL_PID_TARGET_CM,
            H26_T3_FINAL_PID_KP_MM_PER_CM,
            H26_T3_FINAL_PID_KI_MM_PER_CM_S,
            H26_T3_FINAL_PID_KD_MM_PER_CMPS,
            H26_T3_FINAL_PID_INTEGRAL_LIMIT_CM_S,
            H26_T3_FINAL_PID_TILT_LIMIT_MM,
            0.0f);
        if (sample == H26_BALL_SAMPLE_NEW &&
            H26_T3_AbsFloat(H26_BallControl_GetErrorCm()) <=
                H26_T3_FINAL_PID_TOLERANCE_CM &&
            H26_T3_AbsFloat(H26_BallControl_GetBallSpeedCmps()) <=
                H26_T3_FINAL_PID_STABLE_SPEED_CMPS)
        {
            s_finalPidStableMs = H26_T3_AddMs(s_finalPidStableMs,
                H26_BallControl_GetStableSampleMs());
            if (s_finalPidStableMs >= H26_T3_FINAL_PID_STABLE_MS)
            {
                s_finalElapsedMs = nowMs - s_startMs;
                H26_BallControl_Stop();
                s_state = H26_T3_DONE;
                return H26_T3_RESULT_FINISHED;
            }
        }
        else if (sample == H26_BALL_SAMPLE_NEW)
        {
            s_finalPidStableMs = 0U;
        }

        if ((nowMs - s_phaseStartMs) >= H26_T3_FINAL_PID_TIMEOUT_MS)
        {
            H26_T3_EnterFault(H26_T3_FAULT_FINAL_PID_TIMEOUT);
        }
        break;

    case H26_T3_DONE:
        H26_BallControl_Stop();
        return H26_T3_RESULT_FINISHED;

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
    if (s_state == H26_T3_DONE)
    {
        return s_finalElapsedMs;
    }
    return nowMs - s_startMs;
}

uint32_t H26_Task3_GetFinalElapsedMs(void) { return s_finalElapsedMs; }
H26_Task3Fault_t H26_Task3_GetFault(void) { return s_fault; }
