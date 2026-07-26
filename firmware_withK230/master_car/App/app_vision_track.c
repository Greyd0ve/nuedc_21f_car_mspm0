#include "app_vision_track.h"
#include "app_vision_link.h"
#include "app_config.h"
#include "app_car_state.h"
#include "app_control.h"
#if VISION_TRACK_DEBUG_ENABLE
#include "DebugSerial.h"
#endif
#include "Motor.h"
#include <stdint.h>

static VisionTrackState_t s_state = VISION_TRACK_IDLE;

static int16_t  s_lastEy = 0;
static uint8_t  s_hasLastEy = 0U;

static uint8_t  s_acquireCount = 0U;
static uint16_t s_lastAcquireSeq = 0xFFFFU;

static uint8_t  s_lostRecoverCount = 0U;
static uint16_t s_lastRecoverSeq = 0xFFFFU;

static uint8_t  s_invalidFrameCount = 0U;
static uint32_t s_lastOverflowCount = 0U;

static VisionTrackFrame_t s_curFrame;

static float s_lastDEy = 0.0f;
static float s_severity = 0.0f;
static float s_forwardTarget = 0.0f;
static float s_forwardCmdFiltered = 0.0f;
static float s_turnTarget = 0.0f;
static float s_turnCmd = 0.0f;
static float s_leftTarget = 0.0f;
static float s_rightTarget = 0.0f;

static float s_filteredEyControl = 0.0f;
static float s_filteredEaControl = 0.0f;
static float s_previewCompDeciMm = 0.0f;
static float s_bodyEyControl = 0.0f;
static float s_headingScale = 0.0f;
static uint8_t s_hasFilteredErrors = 0U;

static float s_turnEy = 0.0f;
static float s_turnEa = 0.0f;
static float s_turnD = 0.0f;
static uint8_t s_curveHoldFrames = 0U;
static uint8_t s_degradedFrameStreak = 0U;
static uint8_t s_turnReversalFrames = 0U;
static int8_t s_pendingTurnDirection = 0;
static uint32_t s_turnDirectionChangeCount = 0U;
static int8_t s_curveDirectionLock = 0;
static int8_t s_curveLockCandidateDirection = 0;
static uint8_t s_curveLockCandidateFrames = 0U;
static uint8_t s_curveReverseFrames = 0U;
static uint8_t s_curveReverseAllowed = 0U;

#if VISION_TRACK_DEBUG_ENABLE
static char     s_debugTxBuffer[VISION_TRACK_DEBUG_BUFFER_SIZE];
static uint16_t s_debugTxLength = 0U;
static uint16_t s_debugElapsedMs = 0U;
#endif

static int16_t VisionTrack_CentiDegToDeciDeg(int16_t value)
{
    if (value >= 0)
        return (int16_t)((value + 5) / 10);
    return (int16_t)((value - 5) / 10);
}

static float VisionTrack_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float VisionTrack_LimitFloat(float value, float minValue, float maxValue)
{
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static float VisionTrack_MaxFloat(float a, float b)
{
    return (a >= b) ? a : b;
}

static float VisionTrack_CalcTurnLimit(float forwardSpeed)
{
    float innerSpeed;

    forwardSpeed = VisionTrack_LimitFloat(forwardSpeed,
                                          0.0f,
                                          VISION_TRACK_TURN_LIMIT_CMPS);
    innerSpeed = VisionTrack_LimitFloat(
        VISION_TRACK_MIN_INNER_WHEEL_SPEED_CMPS,
        0.0f,
        forwardSpeed);

    return forwardSpeed - innerSpeed;
}

static float VisionTrack_MoveToward(float current,
                                     float target,
                                     float upStep,
                                     float downStep)
{
    if (target > current)
    {
        if ((target - current) > upStep)
        {
            return current + upStep;
        }
    }
    else if (target < current)
    {
        if ((current - target) > downStep)
        {
            return current - downStep;
        }
    }

    return target;
}

static int8_t VisionTrack_GetTurnDirection(float value)
{
    if (value > VISION_TRACK_TURN_REVERSAL_DEADBAND_CMPS)
    {
        return 1;
    }
    if (value < -VISION_TRACK_TURN_REVERSAL_DEADBAND_CMPS)
    {
        return -1;
    }
    return 0;
}

static void VisionTrack_UpdateFilteredErrors(int16_t eyControl,
                                             int16_t eaControl,
                                             uint8_t degraded)
{
    float eyNew = (float)eyControl;
    float eaNew = (float)eaControl;
    float eyAlpha;
    float eaAlpha;

    if (!s_hasFilteredErrors)
    {
        s_filteredEyControl = eyNew;
        s_filteredEaControl = degraded ? 0.0f : eaNew;
        s_hasFilteredErrors = 1U;
        return;
    }

    if (degraded)
    {
        /* A one-boundary ey estimate can jump; ea=0 means unavailable, not straight. */
        if (VisionTrack_AbsFloat(eyNew - s_filteredEyControl) <=
            VISION_TRACK_EY_JUMP_REJECT_DECI_MM)
        {
            s_filteredEyControl += VISION_TRACK_DEGRADED_EY_FILTER_ALPHA *
                (eyNew - s_filteredEyControl);
        }
        s_filteredEaControl *= VISION_TRACK_DEGRADED_EA_DECAY;
    }
    else
    {
        if (s_curveHoldFrames > 0U)
        {
            eyAlpha = VISION_TRACK_CURVE_EY_FILTER_ALPHA;
            eaAlpha = VISION_TRACK_CURVE_EA_FILTER_ALPHA;
        }
        else
        {
            eyAlpha = VISION_TRACK_EY_FILTER_ALPHA;
            eaAlpha = VISION_TRACK_EA_FILTER_ALPHA;
        }

        s_filteredEyControl += eyAlpha *
            (eyNew - s_filteredEyControl);
        s_filteredEaControl += eaAlpha *
            (eaNew - s_filteredEaControl);
    }
}

static void VisionTrack_UpdateBodyError(const VisionTrackFrame_t *frame,
                                        uint8_t degraded)
{
    float previewHeading;
    float previewCompTarget;
    float previewDistance;

    /*
     * Heading is unavailable in a one-boundary/degraded frame.  Keep the
     * previous geometric compensation instead of snapping the virtual point
     * back to the camera centre for one frame.
     */
    previewCompTarget = s_previewCompDeciMm;

    if ((!degraded) &&
        (frame->confidence >= VISION_TRACK_TRUSTED_CONFIDENCE))
    {
        previewHeading = VisionTrack_LimitFloat(
            s_filteredEaControl,
            -VISION_TRACK_PREVIEW_EA_LIMIT_DEG /
                VISION_TRACK_EA_DECI_DEG_TO_DEG,
            VISION_TRACK_PREVIEW_EA_LIMIT_DEG /
                VISION_TRACK_EA_DECI_DEG_TO_DEG);
        previewDistance = VISION_TRACK_CAMERA_LOOKAHEAD_MM -
            VISION_TRACK_VIRTUAL_LOOKAHEAD_MM;
        previewCompTarget = VISION_TRACK_PREVIEW_HEADING_SIGN *
            previewDistance * previewHeading *
            VISION_TRACK_PREVIEW_RAD_PER_DECI_DEG /
            VISION_TRACK_EY_DECI_MM_TO_MM;
        previewCompTarget = VisionTrack_LimitFloat(
            previewCompTarget,
            -VISION_TRACK_PREVIEW_MAX_COMP_MM /
                VISION_TRACK_EY_DECI_MM_TO_MM,
            VISION_TRACK_PREVIEW_MAX_COMP_MM /
                VISION_TRACK_EY_DECI_MM_TO_MM);
    }

    /* Do not let a one-frame heading change move the virtual reference. */
    s_previewCompDeciMm = VisionTrack_MoveToward(
        s_previewCompDeciMm,
        previewCompTarget,
        VISION_TRACK_PREVIEW_STEP_DECI_MM,
        VISION_TRACK_PREVIEW_STEP_DECI_MM);
    s_bodyEyControl = s_filteredEyControl - s_previewCompDeciMm;
}

static float VisionTrack_CalcLargeErrorTurnScale(float bodyEyControl)
{
    float ratio;
    float smoothRatio;
    float absEy = VisionTrack_AbsFloat(bodyEyControl);

    if (absEy <= VISION_TRACK_TURN_ATTENUATE_START_DECI_MM)
    {
        return 1.0f;
    }

    if (absEy >= VISION_TRACK_TURN_ATTENUATE_FULL_DECI_MM)
    {
        return VISION_TRACK_LARGE_ERROR_MIN_TURN_SCALE;
    }

    ratio = (absEy - VISION_TRACK_TURN_ATTENUATE_START_DECI_MM) /
        (VISION_TRACK_TURN_ATTENUATE_FULL_DECI_MM -
         VISION_TRACK_TURN_ATTENUATE_START_DECI_MM);
    /* Smoothstep avoids a gain corner when the vehicle enters a deep bend. */
    smoothRatio = ratio * ratio * (3.0f - 2.0f * ratio);

    return 1.0f - (1.0f - VISION_TRACK_LARGE_ERROR_MIN_TURN_SCALE) *
        smoothRatio;
}

static float VisionTrack_CalcTurnCandidate(float dEy, uint8_t confidence)
{
    float maxOpposingEa;
    float headingControl;
    float eyRatio;
    float largeErrorTurnScale;
    float turnCandidate;

    s_turnEy = VISION_TRACK_TURN_SIGN * VISION_TRACK_KY *
        s_bodyEyControl;

    headingControl = VisionTrack_LimitFloat(
        s_filteredEaControl,
        -VISION_TRACK_HEADING_CONTROL_LIMIT_DEG /
            VISION_TRACK_EA_DECI_DEG_TO_DEG,
        VISION_TRACK_HEADING_CONTROL_LIMIT_DEG /
            VISION_TRACK_EA_DECI_DEG_TO_DEG);
    eyRatio = VisionTrack_LimitFloat(
        VisionTrack_AbsFloat(s_bodyEyControl) /
            VISION_TRACK_HEADING_FULL_EY_DECI_MM,
        0.0f,
        1.0f);
    s_headingScale = VISION_TRACK_HEADING_MIN_SCALE +
        (1.0f - VISION_TRACK_HEADING_MIN_SCALE) * eyRatio;
    if (confidence < VISION_TRACK_TRUSTED_CONFIDENCE)
    {
        s_headingScale *= VISION_TRACK_HEADING_LOW_CONF_SCALE;
    }

    s_turnEa = VISION_TRACK_HEADING_SIGN * VISION_TRACK_KA *
        headingControl * s_headingScale;
    s_turnEa = VisionTrack_LimitFloat(s_turnEa,
                                      -VISION_TRACK_HEADING_TERM_LIMIT_CMPS,
                                      VISION_TRACK_HEADING_TERM_LIMIT_CMPS);
    s_turnD = VISION_TRACK_TURN_SIGN * VISION_TRACK_KD * dEy;
    s_turnD = VisionTrack_LimitFloat(s_turnD,
                                     -VISION_TRACK_D_TERM_LIMIT_CMPS,
                                     VISION_TRACK_D_TERM_LIMIT_CMPS);

    /* A large cross-track error must not be reversed by an opposing ea term. */
    if ((VisionTrack_AbsFloat(s_bodyEyControl) >=
         VISION_TRACK_EY_HEADING_GUARD_DECI_MM) &&
        ((s_turnEy * s_turnEa) < 0.0f))
    {
        maxOpposingEa = VisionTrack_AbsFloat(s_turnEy) *
            VISION_TRACK_OPPOSING_EA_RATIO;
        s_turnEa = VisionTrack_LimitFloat(s_turnEa,
                                           -maxOpposingEa,
                                           maxOpposingEa);
    }

    turnCandidate = s_turnEy + s_turnEa + s_turnD;
    largeErrorTurnScale = VisionTrack_CalcLargeErrorTurnScale(
        s_bodyEyControl);
    turnCandidate *= largeErrorTurnScale;
    return VisionTrack_LimitFloat(turnCandidate,
                                  -VISION_TRACK_TURN_LIMIT_CMPS,
                                  VISION_TRACK_TURN_LIMIT_CMPS);
}

static void VisionTrack_UpdateCurveHold(const VisionTrackFrame_t *frame,
                                        float severity,
                                        uint8_t degraded)
{
    float absEyMm = VisionTrack_AbsFloat(s_bodyEyControl) *
                    VISION_TRACK_EY_DECI_MM_TO_MM;
    float absEaDeg = VisionTrack_AbsFloat(s_filteredEaControl) *
                     VISION_TRACK_EA_DECI_DEG_TO_DEG;

    if (degraded ||
        (severity >= VISION_TRACK_CURVE_TRIGGER_SEVERITY) ||
        (absEyMm >= VISION_TRACK_EY_FULL_SLOW_MM) ||
        (absEaDeg >= VISION_TRACK_EA_FULL_SLOW_DEG))
    {
        s_curveHoldFrames = VISION_TRACK_CURVE_HOLD_FRAMES;
    }
    else if ((!degraded) &&
             (frame->confidence >= VISION_TRACK_TRUSTED_CONFIDENCE) &&
             (severity <= VISION_TRACK_CURVE_RELEASE_SEVERITY) &&
             (s_curveHoldFrames > 0U))
    {
        s_curveHoldFrames--;
    }
    else if (s_curveHoldFrames > 0U)
    {
        /* Release only after consecutive trusted, low-severity full frames. */
        s_curveHoldFrames = VISION_TRACK_CURVE_HOLD_FRAMES;
    }
}

static float VisionTrack_ApplyCurveDirectionLock(float requestedTurn,
                                                  uint8_t curveActive,
                                                  uint8_t reverseAllowed)
{
    int8_t requestedDirection;

    if (!curveActive)
    {
        s_curveDirectionLock = 0;
        s_curveLockCandidateDirection = 0;
        s_curveLockCandidateFrames = 0U;
        s_curveReverseFrames = 0U;
        return requestedTurn;
    }

    requestedDirection = VisionTrack_GetTurnDirection(requestedTurn);
    if (requestedDirection == 0)
    {
        return requestedTurn;
    }

    if (s_curveDirectionLock == 0)
    {
        if (requestedDirection == s_curveLockCandidateDirection)
        {
            if (s_curveLockCandidateFrames < 0xFFU)
            {
                s_curveLockCandidateFrames++;
            }
        }
        else
        {
            s_curveLockCandidateDirection = requestedDirection;
            s_curveLockCandidateFrames = 1U;
        }

        if (s_curveLockCandidateFrames >=
            VISION_TRACK_CURVE_DIRECTION_LOCK_FRAMES)
        {
            s_curveDirectionLock = requestedDirection;
            s_curveReverseFrames = 0U;
        }
        return requestedTurn;
    }

    if (requestedDirection == s_curveDirectionLock)
    {
        s_curveReverseFrames = 0U;
        return requestedTurn;
    }

    /*
     * The virtual preview point can move eyBody across zero on a sharp bend.
     * Only a trusted two-boundary raw ey displacement may reverse the curve.
     */
    if ((!reverseAllowed) ||
        (VisionTrack_AbsFloat(s_filteredEyControl) <
            VISION_TRACK_CURVE_REVERSE_EY_DECI_MM))
    {
        s_curveReverseFrames = 0U;
        return (float)s_curveDirectionLock *
            VISION_TRACK_CURVE_LOCK_HOLD_TURN_CMPS;
    }

    if (s_curveReverseFrames < 0xFFU)
    {
        s_curveReverseFrames++;
    }

    if (s_curveReverseFrames <
        VISION_TRACK_CURVE_REVERSE_CONFIRM_FRAMES)
    {
        return (float)s_curveDirectionLock *
            VISION_TRACK_CURVE_LOCK_HOLD_TURN_CMPS;
    }

    s_curveDirectionLock = requestedDirection;
    s_curveReverseFrames = 0U;
    return requestedTurn;
}

static void VisionTrack_UpdateTurnCommand(float requestedTurn,
                                          uint8_t degraded,
                                          uint8_t isNewFrame,
                                          float forwardLimit,
                                          uint8_t curveActive,
                                          uint8_t reverseAllowed)
{
    int8_t currentDirection;
    int8_t requestedDirection;

    if (isNewFrame)
    {
        requestedTurn = VisionTrack_LimitFloat(
            requestedTurn,
            -VISION_TRACK_TURN_LIMIT_CMPS,
            VISION_TRACK_TURN_LIMIT_CMPS);
        forwardLimit = VisionTrack_CalcTurnLimit(forwardLimit);
        requestedTurn = VisionTrack_LimitFloat(requestedTurn,
                                                -forwardLimit,
                                                forwardLimit);

        if (degraded)
        {
            /* Limit a one-boundary estimate once per K230 frame, not per 10 ms. */
            requestedTurn = VisionTrack_MoveToward(
                s_turnTarget,
                requestedTurn,
                VISION_TRACK_DEGRADED_TURN_STEP_CMPS,
                VISION_TRACK_DEGRADED_TURN_STEP_CMPS);
        }

        requestedTurn = VisionTrack_ApplyCurveDirectionLock(requestedTurn,
                                                              curveActive,
                                                              reverseAllowed);

        currentDirection = VisionTrack_GetTurnDirection(s_turnCmd);
        requestedDirection = VisionTrack_GetTurnDirection(requestedTurn);

        if ((currentDirection != 0) &&
            (requestedDirection != 0) &&
            (currentDirection != requestedDirection))
        {
            if (requestedDirection == s_pendingTurnDirection)
            {
                if (s_turnReversalFrames < 0xFFU)
                {
                    s_turnReversalFrames++;
                }
            }
            else
            {
                s_pendingTurnDirection = requestedDirection;
                s_turnReversalFrames = 1U;
            }

            if (s_turnReversalFrames <
                VISION_TRACK_TURN_REVERSAL_CONFIRM_FRAMES)
            {
                /* Decay to zero before a newly observed opposite command is trusted. */
                s_turnTarget = 0.0f;
            }
            else
            {
                s_turnTarget = requestedTurn;
                s_turnReversalFrames = 0U;
                s_pendingTurnDirection = 0;
                s_turnDirectionChangeCount++;
            }
        }
        else
        {
            s_turnTarget = requestedTurn;
            s_turnReversalFrames = 0U;
            s_pendingTurnDirection = 0;
        }
    }

    s_turnCmd = VisionTrack_MoveToward(s_turnCmd,
                                        s_turnTarget,
                                        VISION_TRACK_TURN_STEP_CMPS,
                                        VISION_TRACK_TURN_STEP_CMPS);
}

static float VisionTrack_CalcSeverity(float eyControl,
                                       float eaControl,
                                       float turnCmd)
{
    float absEyMm;
    float absEaDeg;
    float eySeverity;
    float eaSeverity;
    float turnSeverity;

    absEyMm = VisionTrack_AbsFloat(eyControl) *
               VISION_TRACK_EY_DECI_MM_TO_MM;
    absEaDeg = VisionTrack_AbsFloat(eaControl) *
               VISION_TRACK_EA_DECI_DEG_TO_DEG;

    eySeverity = VisionTrack_LimitFloat(
        absEyMm / VISION_TRACK_EY_FULL_SLOW_MM, 0.0f, 1.0f);
    eaSeverity = VisionTrack_LimitFloat(
        absEaDeg / VISION_TRACK_EA_FULL_SLOW_DEG, 0.0f, 1.0f);
    turnSeverity = VisionTrack_LimitFloat(
        VisionTrack_AbsFloat(turnCmd) / VISION_TRACK_TURN_LIMIT_CMPS,
        0.0f,
        1.0f);

    return VisionTrack_MaxFloat(
        eySeverity, VisionTrack_MaxFloat(eaSeverity, turnSeverity));
}

static float VisionTrack_CalcForwardTarget(float severity,
                                           uint8_t degraded,
                                           uint8_t curveHoldFrames)
{
    float target;

    if (degraded)
    {
        target = VISION_TRACK_DEGRADED_SPEED_CMPS;
    }
    else
    {
        target = VISION_TRACK_MAX_SPEED_CMPS -
            severity *
            (VISION_TRACK_MAX_SPEED_CMPS - VISION_TRACK_MIN_SPEED_CMPS);
    }

    target = VisionTrack_LimitFloat(target,
                                    VISION_TRACK_MIN_SPEED_CMPS,
                                    VISION_TRACK_MAX_SPEED_CMPS);

    if (curveHoldFrames > 0U)
    {
        target = VisionTrack_LimitFloat(target,
                                        VISION_TRACK_MIN_SPEED_CMPS,
                                        VISION_TRACK_CURVE_HOLD_SPEED_CMPS);
    }

    return target;
}

static void VisionTrack_ResetControlHistory(void)
{
    s_lastEy = 0;
    s_hasLastEy = 0U;
    s_lastDEy = 0.0f;
    s_severity = 0.0f;
    s_forwardTarget = 0.0f;
    s_forwardCmdFiltered = 0.0f;
    s_turnTarget = 0.0f;
    s_turnCmd = 0.0f;
    s_leftTarget = 0.0f;
    s_rightTarget = 0.0f;

    s_filteredEyControl = 0.0f;
    s_filteredEaControl = 0.0f;
    s_previewCompDeciMm = 0.0f;
    s_bodyEyControl = 0.0f;
    s_headingScale = 0.0f;
    s_hasFilteredErrors = 0U;
    s_turnEy = 0.0f;
    s_turnEa = 0.0f;
    s_turnD = 0.0f;
    s_curveHoldFrames = 0U;
    s_degradedFrameStreak = 0U;
    s_turnReversalFrames = 0U;
    s_pendingTurnDirection = 0;
    s_turnDirectionChangeCount = 0U;
    s_curveDirectionLock = 0;
    s_curveLockCandidateDirection = 0;
    s_curveLockCandidateFrames = 0U;
    s_curveReverseFrames = 0U;
    s_curveReverseAllowed = 0U;
}

static void VisionTrack_PrepareRun(void)
{
    VisionTrack_ResetControlHistory();
    s_forwardTarget = VISION_TRACK_MIN_SPEED_CMPS;
    s_forwardCmdFiltered = VISION_TRACK_MIN_SPEED_CMPS;
    s_leftTarget = VISION_TRACK_MIN_SPEED_CMPS;
    s_rightTarget = VISION_TRACK_MIN_SPEED_CMPS;
}

static uint8_t VisionTrack_IsTransportOk(const VisionTrackFrame_t *frame)
{
    if (!frame->transportValid) return 0U;
    if (App_VisionLink_GetFrameAgeMs() > VISION_TRACK_FRESH_LIMIT_MS)
        return 0U;
    return 1U;
}

static uint8_t VisionTrack_IsFrameUsable(const VisionTrackFrame_t *frame)
{
    if (!VisionTrack_IsTransportOk(frame)) return 0U;
    if (!frame->visionValid) return 0U;
    if (frame->mode != VISION_MODE_TRACK) return 0U;
    if (frame->confidence < VISION_TRACK_MIN_CONFIDENCE) return 0U;
    return 1U;
}

static uint8_t VisionTrack_IsDegradedFrame(const VisionTrackFrame_t *frame)
{
    if (frame->degraded)
    {
        return 1U;
    }

    return ((!frame->leftBoundaryValid) ||
            (!frame->rightBoundaryValid)) ? 1U : 0U;
}

static void VisionTrack_SafeStop(void)
{
    App_Control_SetLowSpeedTrackSafety(0U);
    App_Control_ForcePWMZero();
    Motor_StopAll();
    g_carEnable = 0U;
}

static void VisionTrack_EnterState(VisionTrackState_t next)
{
    if (s_state == next) return;

    s_state = next;

    if (next == VISION_TRACK_RUN)
    {
        VisionTrack_PrepareRun();
    }
    else
    {
        VisionTrack_ResetControlHistory();
    }
}

static void VisionTrack_ApplyMotorCommand(void)
{
    s_forwardCmdFiltered = VisionTrack_LimitFloat(
        s_forwardCmdFiltered,
        VISION_TRACK_MIN_SPEED_CMPS,
        VISION_TRACK_MAX_SPEED_CMPS);

    s_turnCmd = VisionTrack_LimitFloat(
        s_turnCmd,
        -VISION_TRACK_TURN_LIMIT_CMPS,
        VISION_TRACK_TURN_LIMIT_CMPS);

    /*
     * Normal RUN retains a rolling inside wheel.  This is stricter than the
     * old no-reverse-only limit and prevents a broad curve from degrading
     * into an abrupt one-wheel pivot.
     */
    s_turnCmd = VisionTrack_LimitFloat(
        s_turnCmd,
        -VisionTrack_CalcTurnLimit(s_forwardCmdFiltered),
        VisionTrack_CalcTurnLimit(s_forwardCmdFiltered));

    s_leftTarget = s_forwardCmdFiltered - s_turnCmd;
    s_rightTarget = s_forwardCmdFiltered + s_turnCmd;

    g_targetForwardSpeed = s_forwardCmdFiltered;
    g_targetTurnSpeed = s_turnCmd;
    g_carEnable = 1U;
    App_Control_SetLowSpeedTrackSafety(1U);
    App_Control_ApplyMotorOutput();
}

static void VisionTrack_ClearCurrentFrame(void)
{
    s_curFrame.sequence = 0U;
    s_curFrame.mode = VISION_MODE_IDLE;
    s_curFrame.statusFlags = 0U;
    s_curFrame.lateralErrorDeciMm = 0;
    s_curFrame.headingErrorCentiDeg = 0;
    s_curFrame.confidence = 0U;
    s_curFrame.transportValid = 0U;
    s_curFrame.visionValid = 0U;
    s_curFrame.degraded = 0U;
    s_curFrame.leftBoundaryValid = 0U;
    s_curFrame.rightBoundaryValid = 0U;
}

void App_VisionTrack_Init(void)
{
    s_state = VISION_TRACK_IDLE;
    s_acquireCount = 0U;
    s_lastAcquireSeq = 0xFFFFU;
    s_lostRecoverCount = 0U;
    s_lastRecoverSeq = 0xFFFFU;
    s_invalidFrameCount = 0U;
    s_lastOverflowCount = 0U;

    VisionTrack_ClearCurrentFrame();
    VisionTrack_ResetControlHistory();

#if VISION_TRACK_DEBUG_ENABLE
    s_debugTxLength = 0U;
    s_debugElapsedMs = 0U;
#endif

    VisionTrack_SafeStop();
}

#if VISION_TRACK_DEBUG_ENABLE
static void VisionTrack_DebugAppendChar(char value)
{
    if (s_debugTxLength < (VISION_TRACK_DEBUG_BUFFER_SIZE - 1U))
    {
        s_debugTxBuffer[s_debugTxLength++] = value;
    }
}

static void VisionTrack_DebugAppendString(const char *text)
{
    while (*text != '\0')
    {
        VisionTrack_DebugAppendChar(*text);
        text++;
    }
}

static void VisionTrack_DebugAppendU32(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do
    {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value > 0U) && (count < sizeof(digits)));

    while (count > 0U)
    {
        VisionTrack_DebugAppendChar(digits[--count]);
    }
}

static void VisionTrack_DebugAppendI32(int32_t value)
{
    uint32_t magnitude;

    if (value < 0)
    {
        VisionTrack_DebugAppendChar('-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)value;
    }

    VisionTrack_DebugAppendU32(magnitude);
}

static void VisionTrack_DebugAppendFloat2(float value)
{
    int32_t scaled;
    uint32_t magnitude;
    uint32_t fraction;

    if (value >= 0.0f)
    {
        scaled = (int32_t)(value * 100.0f + 0.5f);
    }
    else
    {
        scaled = (int32_t)(value * 100.0f - 0.5f);
    }

    if (scaled < 0)
    {
        VisionTrack_DebugAppendChar('-');
        magnitude = (uint32_t)(-(scaled + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)scaled;
    }

    fraction = magnitude % 100U;
    VisionTrack_DebugAppendU32(magnitude / 100U);
    VisionTrack_DebugAppendChar('.');
    VisionTrack_DebugAppendChar((char)('0' + (fraction / 10U)));
    VisionTrack_DebugAppendChar((char)('0' + (fraction % 10U)));
}

static void VisionTrack_DebugBuildRecord(void)
{
    s_debugTxLength = 0U;

    VisionTrack_DebugAppendString("[vision-track,state=");
    VisionTrack_DebugAppendU32((uint32_t)s_state);
    VisionTrack_DebugAppendString(",sequence=");
    VisionTrack_DebugAppendU32((uint32_t)s_curFrame.sequence);
    VisionTrack_DebugAppendString(",frameAge=");
    VisionTrack_DebugAppendU32(App_VisionLink_GetFrameAgeMs());
    VisionTrack_DebugAppendString(",visionValid=");
    VisionTrack_DebugAppendU32((uint32_t)s_curFrame.visionValid);
    VisionTrack_DebugAppendString(",degraded=");
    VisionTrack_DebugAppendU32((uint32_t)s_curFrame.degraded);
    VisionTrack_DebugAppendString(",confidence=");
    VisionTrack_DebugAppendU32((uint32_t)s_curFrame.confidence);
    VisionTrack_DebugAppendString(",ey=");
    VisionTrack_DebugAppendI32((int32_t)s_curFrame.lateralErrorDeciMm);
    VisionTrack_DebugAppendString(",ea=");
    VisionTrack_DebugAppendI32((int32_t)s_curFrame.headingErrorCentiDeg);
    VisionTrack_DebugAppendString(",dEy=");
    VisionTrack_DebugAppendI32((int32_t)s_lastDEy);
    VisionTrack_DebugAppendString(",eyFilt=");
    VisionTrack_DebugAppendFloat2(s_filteredEyControl);
    VisionTrack_DebugAppendString(",eaFilt=");
    VisionTrack_DebugAppendFloat2(s_filteredEaControl);
    VisionTrack_DebugAppendString(",pComp=");
    VisionTrack_DebugAppendFloat2(s_previewCompDeciMm);
    VisionTrack_DebugAppendString(",eyBody=");
    VisionTrack_DebugAppendFloat2(s_bodyEyControl);
    VisionTrack_DebugAppendString(",turnEy=");
    VisionTrack_DebugAppendFloat2(s_turnEy);
    VisionTrack_DebugAppendString(",turnEa=");
    VisionTrack_DebugAppendFloat2(s_turnEa);
    VisionTrack_DebugAppendString(",hScale=");
    VisionTrack_DebugAppendFloat2(s_headingScale);
    VisionTrack_DebugAppendString(",turnD=");
    VisionTrack_DebugAppendFloat2(s_turnD);
    VisionTrack_DebugAppendString(",turnTarget=");
    VisionTrack_DebugAppendFloat2(s_turnTarget);
    VisionTrack_DebugAppendString(",severity=");
    VisionTrack_DebugAppendFloat2(s_severity);
    VisionTrack_DebugAppendString(",forwardTarget=");
    VisionTrack_DebugAppendFloat2(s_forwardTarget);
    VisionTrack_DebugAppendString(",forwardFiltered=");
    VisionTrack_DebugAppendFloat2(s_forwardCmdFiltered);
    VisionTrack_DebugAppendString(",turnCmd=");
    VisionTrack_DebugAppendFloat2(s_turnCmd);
    VisionTrack_DebugAppendString(",leftTarget=");
    VisionTrack_DebugAppendFloat2(s_leftTarget);
    VisionTrack_DebugAppendString(",rightTarget=");
    VisionTrack_DebugAppendFloat2(s_rightTarget);
    VisionTrack_DebugAppendString(",leftSpeed=");
    VisionTrack_DebugAppendFloat2(g_leftSpeed);
    VisionTrack_DebugAppendString(",rightSpeed=");
    VisionTrack_DebugAppendFloat2(g_rightSpeed);
    VisionTrack_DebugAppendString(",leftPWM=");
    VisionTrack_DebugAppendI32((int32_t)g_leftPwm);
    VisionTrack_DebugAppendString(",rightPWM=");
    VisionTrack_DebugAppendI32((int32_t)g_rightPwm);
    VisionTrack_DebugAppendString(",invalidFrameCount=");
    VisionTrack_DebugAppendU32((uint32_t)s_invalidFrameCount);
    VisionTrack_DebugAppendString(",curveHold=");
    VisionTrack_DebugAppendU32((uint32_t)s_curveHoldFrames);
    VisionTrack_DebugAppendString(",curveDir=");
    VisionTrack_DebugAppendI32((int32_t)s_curveDirectionLock);
    VisionTrack_DebugAppendString(",curveRevOk=");
    VisionTrack_DebugAppendU32((uint32_t)s_curveReverseAllowed);
    VisionTrack_DebugAppendString(",degradedFrames=");
    VisionTrack_DebugAppendU32((uint32_t)s_degradedFrameStreak);
    VisionTrack_DebugAppendString(",turnFlips=");
    VisionTrack_DebugAppendU32(s_turnDirectionChangeCount);
    VisionTrack_DebugAppendString(",rxOverflowCount=");
    VisionTrack_DebugAppendU32(App_VisionLink_GetRxOverflowCount());
    VisionTrack_DebugAppendString("]\r\n");

    s_debugTxBuffer[s_debugTxLength] = '\0';
}
#endif

void App_VisionTrack_Task10ms(void)
{
    uint8_t isNewSeq;
    uint8_t transportOk;
    uint8_t frameUsable;
    uint8_t controlDegraded;
    uint8_t curveActive;
    uint8_t curveReverseAllowed;
    int16_t eyControl;
    int16_t eaControl;
    float turnCandidate;
    float dEy;
    uint32_t curOverflow;

    (void)App_VisionLink_GetLatest(&s_curFrame);
    isNewSeq = App_VisionLink_HasNewFrame();
    transportOk = VisionTrack_IsTransportOk(&s_curFrame);
    frameUsable = VisionTrack_IsFrameUsable(&s_curFrame);
    controlDegraded = VisionTrack_IsDegradedFrame(&s_curFrame);
    curOverflow = App_VisionLink_GetRxOverflowCount();

    switch (s_state)
    {
    case VISION_TRACK_IDLE:
        VisionTrack_SafeStop();
        break;

    case VISION_TRACK_ACQUIRE:
        VisionTrack_SafeStop();
        if (!frameUsable)
        {
            s_acquireCount = 0U;
            s_lastAcquireSeq = 0xFFFFU;
        }
        else if (isNewSeq && s_curFrame.sequence != s_lastAcquireSeq)
        {
            s_lastAcquireSeq = s_curFrame.sequence;
            s_acquireCount++;
            if (s_acquireCount >= VISION_TRACK_ACQUIRE_FRAMES)
            {
                s_invalidFrameCount = 0U;
                s_lastOverflowCount = curOverflow;
                VisionTrack_EnterState(VISION_TRACK_RUN);
            }
        }
        break;

    case VISION_TRACK_RUN:
        if (curOverflow != s_lastOverflowCount)
        {
            VisionTrack_SafeStop();
            s_lostRecoverCount = 0U;
            s_lastRecoverSeq = 0xFFFFU;
            s_invalidFrameCount = 0U;
            App_VisionLink_Reset();
            VisionTrack_EnterState(VISION_TRACK_LOST);
            break;
        }

        if (!transportOk)
        {
            VisionTrack_SafeStop();
            s_lostRecoverCount = 0U;
            s_lastRecoverSeq = 0xFFFFU;
            s_invalidFrameCount = 0U;
            VisionTrack_EnterState(VISION_TRACK_LOST);
            break;
        }

        if (isNewSeq)
        {
            if (frameUsable)
            {
                s_invalidFrameCount = 0U;
            }
            else if (s_invalidFrameCount < 0xFFU)
            {
                s_invalidFrameCount++;
            }
        }

        if (s_invalidFrameCount >= VISION_TRACK_INVALID_CONFIRM_FRAMES)
        {
            VisionTrack_SafeStop();
            s_lostRecoverCount = 0U;
            s_lastRecoverSeq = 0xFFFFU;
            VisionTrack_EnterState(VISION_TRACK_LOST);
            break;
        }

        if (!frameUsable)
        {
            /*
             * Confirm short invalid bursts without using sentinel errors.
             * Slow down, keep the curve-speed lock and gently remove steering
             * until a usable frame
             * returns or the invalid-frame threshold enters LOST.
             */
            s_lastDEy = 0.0f;
            s_severity = 0.0f;
            s_curveHoldFrames = VISION_TRACK_CURVE_HOLD_FRAMES;
            s_forwardTarget = VISION_TRACK_DEGRADED_SPEED_CMPS;
            s_forwardCmdFiltered = VisionTrack_MoveToward(
                s_forwardCmdFiltered,
                s_forwardTarget,
                VISION_TRACK_ACCEL_STEP_CMPS,
                VISION_TRACK_DECEL_STEP_CMPS);
            s_turnTarget = 0.0f;
            s_turnCmd = VisionTrack_MoveToward(
                s_turnCmd,
                0.0f,
                VISION_TRACK_TURN_DECAY_STEP_CMPS,
                VISION_TRACK_TURN_DECAY_STEP_CMPS);
            VisionTrack_ApplyMotorCommand();
            break;
        }

        eyControl = s_curFrame.lateralErrorDeciMm;
        eaControl = VisionTrack_CentiDegToDeciDeg(
            s_curFrame.headingErrorCentiDeg);

        if (isNewSeq && !controlDegraded)
        {
            if (s_hasLastEy)
            {
                dEy = (float)(eyControl - s_lastEy);
            }
            else
            {
                dEy = 0.0f;
            }
            s_lastEy = eyControl;
            s_hasLastEy = 1U;
        }
        else
        {
            dEy = 0.0f;
        }
        s_lastDEy = dEy;

        if (isNewSeq)
        {
            VisionTrack_UpdateFilteredErrors(eyControl,
                                             eaControl,
                                             controlDegraded);
            VisionTrack_UpdateBodyError(&s_curFrame, controlDegraded);
            turnCandidate = VisionTrack_CalcTurnCandidate(
                dEy, s_curFrame.confidence);
            s_severity = VisionTrack_CalcSeverity(s_bodyEyControl,
                                                   s_filteredEaControl,
                                                   turnCandidate);
            VisionTrack_UpdateCurveHold(&s_curFrame,
                                        s_severity,
                                        controlDegraded);
            s_forwardTarget = VisionTrack_CalcForwardTarget(
                s_severity,
                controlDegraded,
                s_curveHoldFrames);

            if (controlDegraded)
            {
                if (s_degradedFrameStreak < 0xFFU)
                {
                    s_degradedFrameStreak++;
                }
            }
            else
            {
                s_degradedFrameStreak = 0U;
            }
        }

        s_forwardCmdFiltered = VisionTrack_MoveToward(
            s_forwardCmdFiltered,
            s_forwardTarget,
            VISION_TRACK_ACCEL_STEP_CMPS,
            VISION_TRACK_DECEL_STEP_CMPS);

        curveActive = ((s_curveHoldFrames > 0U) ||
            (s_severity >= VISION_TRACK_CURVE_TRIGGER_SEVERITY)) ? 1U : 0U;
        curveReverseAllowed = s_curveReverseAllowed;
        if (isNewSeq)
        {
            curveReverseAllowed = 0U;
            if ((!controlDegraded) &&
                s_curFrame.leftBoundaryValid &&
                s_curFrame.rightBoundaryValid &&
                (s_curFrame.confidence >= VISION_TRACK_TRUSTED_CONFIDENCE) &&
                (VisionTrack_AbsFloat(s_filteredEyControl) >=
                    VISION_TRACK_CURVE_REVERSE_EY_DECI_MM))
            {
                curveReverseAllowed = 1U;
            }
            s_curveReverseAllowed = curveReverseAllowed;
        }

        if (isNewSeq)
        {
            VisionTrack_UpdateTurnCommand(turnCandidate,
                                           controlDegraded,
                                           1U,
                                           s_forwardCmdFiltered,
                                           curveActive,
                                           curveReverseAllowed);
        }
        else
        {
            VisionTrack_UpdateTurnCommand(s_turnTarget,
                                           0U,
                                           0U,
                                           s_forwardCmdFiltered,
                                           curveActive,
                                           0U);
        }

        VisionTrack_ApplyMotorCommand();
        break;

    case VISION_TRACK_LOST:
        VisionTrack_SafeStop();
        if (frameUsable &&
            isNewSeq &&
            s_curFrame.sequence != s_lastRecoverSeq)
        {
            s_lastRecoverSeq = s_curFrame.sequence;
            s_lostRecoverCount++;
            if (s_lostRecoverCount >= VISION_TRACK_ACQUIRE_FRAMES)
            {
                s_lostRecoverCount = 0U;
                s_lastRecoverSeq = 0xFFFFU;
                s_invalidFrameCount = 0U;
                s_lastOverflowCount = curOverflow;
                VisionTrack_EnterState(VISION_TRACK_RUN);
            }
        }
        else if (!frameUsable)
        {
            s_lostRecoverCount = 0U;
        }
        break;

    case VISION_TRACK_STOP:
        VisionTrack_SafeStop();
        break;

    default:
        VisionTrack_SafeStop();
        VisionTrack_EnterState(VISION_TRACK_IDLE);
        break;
    }
}

void App_VisionTrack_Task100ms(void)
{
#if VISION_TRACK_DEBUG_ENABLE
    s_debugElapsedMs += 100U;
    if (s_debugElapsedMs >= VISION_TRACK_DEBUG_PERIOD_MS)
    {
        s_debugElapsedMs =
            (uint16_t)(s_debugElapsedMs - VISION_TRACK_DEBUG_PERIOD_MS);
        VisionTrack_DebugBuildRecord();
        (void)DebugSerial_TrySendBuffer(
            (const uint8_t *)s_debugTxBuffer, s_debugTxLength);
    }
#endif
}

void App_VisionTrack_HandleKey(uint8_t key)
{
    if (key == 0U) return;

    switch (key)
    {
    case 2U:
        if (s_state == VISION_TRACK_IDLE)
        {
            App_VisionLink_Reset();
            App_VisionTrack_Init();
            s_acquireCount = 0U;
            s_lastAcquireSeq = 0xFFFFU;
            App_VisionLink_SendTrackMode();
            VisionTrack_EnterState(VISION_TRACK_ACQUIRE);
        }
        break;

    case 3U:
        VisionTrack_SafeStop();
        App_VisionLink_SendIdleMode();
        s_invalidFrameCount = 0U;
        VisionTrack_EnterState(VISION_TRACK_STOP);
        break;

    case 4U:
        VisionTrack_SafeStop();
        App_VisionLink_SendIdleMode();
        App_VisionLink_Reset();
        App_VisionTrack_Init();
        break;

    default:
        break;
    }
}
