#include "app_vision_track.h"
#include "app_vision_link.h"
#include "app_config.h"
#include "app_car_state.h"
#include "app_control.h"
#include "DebugSerial.h"
#include "Motor.h"
#include <stdint.h>

static VisionTrackState_t s_state = VISION_TRACK_IDLE;

static int16_t  s_lastEy    = 0;
static uint8_t  s_hasLastEy = 0U;

static uint8_t  s_acquireCount   = 0U;
static uint16_t s_lastAcquireSeq = 0xFFFFU;

static uint8_t  s_lostRecoverCount = 0U;
static uint16_t s_lastRecoverSeq   = 0xFFFFU;

static uint8_t  s_invalidFrameCount = 0U;

static uint32_t s_lastOverflowCount = 0U;

static VisionTrackFrame_t s_curFrame;

static float s_lastDEy = 0.0f;
static float s_severity = 0.0f;
static float s_forwardTarget = 0.0f;
static float s_forwardCmdFiltered = 0.0f;
static float s_turnCmd = 0.0f;
static float s_leftTarget = 0.0f;
static float s_rightTarget = 0.0f;

#if VISION_TRACK_DEBUG_ENABLE
static char     s_debugTxBuffer[VISION_TRACK_DEBUG_BUFFER_SIZE];
static uint16_t s_debugTxLength = 0U;
static uint16_t s_debugTxPosition = 0U;
static uint16_t s_debugElapsedMs = 0U;
#endif

static int16_t Vision_CentiDegToDeciDeg(int16_t value)
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

static float VisionTrack_CalcSeverity(const VisionTrackFrame_t *frame,
                                      float turnCmd)
{
    float absEyMm;
    float absEaDeg;
    float eySeverity;
    float eaSeverity;
    float turnSeverity;

    absEyMm = VisionTrack_AbsFloat((float)frame->lateralErrorDeciMm) *
              VISION_TRACK_EY_DECI_MM_TO_MM;
    absEaDeg = VisionTrack_AbsFloat((float)frame->headingErrorCentiDeg) *
               VISION_TRACK_EA_CENTI_DEG_TO_DEG;

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

static float VisionTrack_CalcForwardTarget(float severity, uint8_t degraded)
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

    return VisionTrack_LimitFloat(target,
                                  VISION_TRACK_MIN_SPEED_CMPS,
                                  VISION_TRACK_MAX_SPEED_CMPS);
}

static void VisionTrack_ResetControlHistory(void)
{
    s_lastEy = 0;
    s_hasLastEy = 0U;
    s_lastDEy = 0.0f;
    s_severity = 0.0f;
    s_forwardTarget = 0.0f;
    s_forwardCmdFiltered = 0.0f;
    s_turnCmd = 0.0f;
    s_leftTarget = 0.0f;
    s_rightTarget = 0.0f;
}

static void VisionTrack_PrepareRun(void)
{
    VisionTrack_ResetControlHistory();
    s_forwardTarget = VISION_TRACK_MIN_SPEED_CMPS;
    s_forwardCmdFiltered = VISION_TRACK_MIN_SPEED_CMPS;
    s_leftTarget = VISION_TRACK_MIN_SPEED_CMPS;
    s_rightTarget = VISION_TRACK_MIN_SPEED_CMPS;
}

static uint8_t IsTransportOk(const VisionTrackFrame_t *f)
{
    if (!f->transportValid) return 0U;
    if (App_VisionLink_GetFrameAgeMs() > VISION_TRACK_FRESH_LIMIT_MS) return 0U;
    return 1U;
}

static uint8_t IsVisionValid(const VisionTrackFrame_t *f)
{
    if (!f->transportValid) return 0U;
    if (!f->visionValid) return 0U;
    if (App_VisionLink_GetFrameAgeMs() > VISION_TRACK_FRESH_LIMIT_MS) return 0U;
    return 1U;
}

static void SafeStop(void)
{
    App_Control_ForcePWMZero();
    Motor_StopAll();
    g_carEnable = 0U;
}

static void EnterState(VisionTrackState_t next)
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
     * Normal visual tracking is differential steering, not pivot turning.
     * Limiting turn to forward keeps both wheel targets non-negative.
     */
    s_turnCmd = VisionTrack_LimitFloat(
        s_turnCmd, -s_forwardCmdFiltered, s_forwardCmdFiltered);

    s_leftTarget = s_forwardCmdFiltered - s_turnCmd;
    s_rightTarget = s_forwardCmdFiltered + s_turnCmd;

    g_targetForwardSpeed = s_forwardCmdFiltered;
    g_targetTurnSpeed = s_turnCmd;
    g_carEnable = 1U;
    App_Control_ApplyMotorOutput();
}

void App_VisionTrack_Init(void)
{
    s_state      = VISION_TRACK_IDLE;
    s_lastEy     = 0;
    s_hasLastEy  = 0U;
    s_acquireCount   = 0U;
    s_lastAcquireSeq = 0xFFFFU;
    s_lostRecoverCount = 0U;
    s_lastRecoverSeq   = 0xFFFFU;
    s_invalidFrameCount = 0U;
    s_lastOverflowCount = 0U;
    s_curFrame.transportValid = 0U;
    s_curFrame.visionValid    = 0U;
    s_curFrame.degraded       = 0U;

    VisionTrack_ResetControlHistory();

#if VISION_TRACK_DEBUG_ENABLE
    s_debugTxLength = 0U;
    s_debugTxPosition = 0U;
    s_debugElapsedMs = 0U;
#endif

    SafeStop();
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
    s_debugTxPosition = 0U;

    VisionTrack_DebugAppendString("[vision-track,state=");
    VisionTrack_DebugAppendU32((uint32_t)s_state);
    VisionTrack_DebugAppendString(",seq=");
    VisionTrack_DebugAppendU32((uint32_t)s_curFrame.sequence);
    VisionTrack_DebugAppendString(",age=");
    VisionTrack_DebugAppendU32(App_VisionLink_GetFrameAgeMs());
    VisionTrack_DebugAppendString(",valid=");
    VisionTrack_DebugAppendU32((uint32_t)s_curFrame.visionValid);
    VisionTrack_DebugAppendString(",degraded=");
    VisionTrack_DebugAppendU32((uint32_t)s_curFrame.degraded);
    VisionTrack_DebugAppendString(",conf=");
    VisionTrack_DebugAppendU32((uint32_t)s_curFrame.confidence);
    VisionTrack_DebugAppendString(",ey=");
    VisionTrack_DebugAppendI32((int32_t)s_curFrame.lateralErrorDeciMm);
    VisionTrack_DebugAppendString(",ea=");
    VisionTrack_DebugAppendI32((int32_t)s_curFrame.headingErrorCentiDeg);
    VisionTrack_DebugAppendString(",dey=");
    VisionTrack_DebugAppendI32((int32_t)s_lastDEy);
    VisionTrack_DebugAppendString(",sev=");
    VisionTrack_DebugAppendFloat2(s_severity);
    VisionTrack_DebugAppendString(",fwd_target=");
    VisionTrack_DebugAppendFloat2(s_forwardTarget);
    VisionTrack_DebugAppendString(",fwd_cmd=");
    VisionTrack_DebugAppendFloat2(s_forwardCmdFiltered);
    VisionTrack_DebugAppendString(",turn=");
    VisionTrack_DebugAppendFloat2(s_turnCmd);
    VisionTrack_DebugAppendString(",left_target=");
    VisionTrack_DebugAppendFloat2(s_leftTarget);
    VisionTrack_DebugAppendString(",right_target=");
    VisionTrack_DebugAppendFloat2(s_rightTarget);
    VisionTrack_DebugAppendString(",left_speed=");
    VisionTrack_DebugAppendFloat2(g_leftSpeed);
    VisionTrack_DebugAppendString(",right_speed=");
    VisionTrack_DebugAppendFloat2(g_rightSpeed);
    VisionTrack_DebugAppendString(",left_pwm=");
    VisionTrack_DebugAppendI32((int32_t)g_leftPwm);
    VisionTrack_DebugAppendString(",right_pwm=");
    VisionTrack_DebugAppendI32((int32_t)g_rightPwm);
    VisionTrack_DebugAppendString(",invalidFrameCount=");
    VisionTrack_DebugAppendU32((uint32_t)s_invalidFrameCount);
    VisionTrack_DebugAppendString(",rxOverflowCount=");
    VisionTrack_DebugAppendU32(App_VisionLink_GetRxOverflowCount());
    VisionTrack_DebugAppendString("]\r\n");

    s_debugTxBuffer[s_debugTxLength] = '\0';
}

static void VisionTrack_DebugPump(void)
{
    while (s_debugTxPosition < s_debugTxLength)
    {
        if (!DebugSerial_TrySendByte(
                (uint8_t)s_debugTxBuffer[s_debugTxPosition]))
        {
            break;
        }

        s_debugTxPosition++;
    }

    if (s_debugTxPosition >= s_debugTxLength)
    {
        s_debugTxPosition = 0U;
        s_debugTxLength = 0U;
    }
}
#endif

void App_VisionTrack_Task10ms(void)
{
    uint8_t  isNewSeq;
    uint8_t  transportOk;
    uint8_t  visionValid;
    int16_t  eyControl;
    int16_t  eaControl;
    float    rawTurnCmd;
    float    dEy;
    uint32_t curOverflow;

    (void)App_VisionLink_GetLatest(&s_curFrame);
    isNewSeq     = App_VisionLink_HasNewFrame();
    transportOk  = IsTransportOk(&s_curFrame);
    visionValid  = IsVisionValid(&s_curFrame);
    curOverflow  = App_VisionLink_GetRxOverflowCount();

    switch (s_state)
    {
    case VISION_TRACK_IDLE:
        SafeStop();
        break;

    case VISION_TRACK_ACQUIRE:
        SafeStop();
        if (!visionValid)
        {
            s_acquireCount   = 0U;
            s_lastAcquireSeq = 0xFFFFU;
        }
        else if (isNewSeq && s_curFrame.sequence != s_lastAcquireSeq)
        {
            s_lastAcquireSeq = s_curFrame.sequence;
            s_acquireCount++;
            if (s_acquireCount >= VISION_TRACK_ACQUIRE_FRAMES)
            {
                s_invalidFrameCount = 0U;
                s_lastOverflowCount = App_VisionLink_GetRxOverflowCount();
                EnterState(VISION_TRACK_RUN);
            }
        }
        break;

    case VISION_TRACK_RUN:
        if (curOverflow != s_lastOverflowCount)
        {
            SafeStop();
            s_lostRecoverCount = 0U;
            s_lastRecoverSeq   = 0xFFFFU;
            s_invalidFrameCount = 0U;
            App_VisionLink_Reset();
            EnterState(VISION_TRACK_LOST);
            break;
        }

        if (!transportOk)
        {
            SafeStop();
            s_lostRecoverCount = 0U;
            s_lastRecoverSeq   = 0xFFFFU;
            s_invalidFrameCount = 0U;
            EnterState(VISION_TRACK_LOST);
            break;
        }

        if (isNewSeq)
        {
            if (s_curFrame.visionValid)
            {
                s_invalidFrameCount = 0U;
            }
            else
            {
                if (s_invalidFrameCount < 0xFFU)
                {
                    s_invalidFrameCount++;
                }
            }
        }

        if (s_invalidFrameCount >= VISION_TRACK_INVALID_CONFIRM_FRAMES)
        {
            SafeStop();
            s_lostRecoverCount = 0U;
            s_lastRecoverSeq   = 0xFFFFU;
            s_invalidFrameCount = 0U;
            EnterState(VISION_TRACK_LOST);
            break;
        }

        if (!s_curFrame.visionValid)
        {
            /*
             * A short invalid-frame burst is confirmed before LOST.  Do not
             * touch sentinel ey/ea values; slow toward the degraded speed and
             * gently remove the previous steering command meanwhile.
             */
            s_lastDEy = 0.0f;
            s_severity = 0.0f;
            s_forwardTarget = VISION_TRACK_DEGRADED_SPEED_CMPS;
            s_forwardCmdFiltered = VisionTrack_MoveToward(
                s_forwardCmdFiltered,
                s_forwardTarget,
                VISION_TRACK_ACCEL_STEP_CMPS,
                VISION_TRACK_DECEL_STEP_CMPS);
            s_turnCmd = VisionTrack_MoveToward(
                s_turnCmd,
                0.0f,
                VISION_TRACK_DEGRADED_TURN_STEP_CMPS,
                VISION_TRACK_DEGRADED_TURN_STEP_CMPS);
            VisionTrack_ApplyMotorCommand();
            break;
        }

        eyControl = s_curFrame.lateralErrorDeciMm;
        eaControl = Vision_CentiDegToDeciDeg(s_curFrame.headingErrorCentiDeg);

        if (isNewSeq)
        {
            if (s_hasLastEy)
            {
                dEy = (float)(eyControl - s_lastEy);
            }
            else
            {
                dEy = 0.0f;
            }
            s_lastEy    = eyControl;
            s_hasLastEy = 1U;
        }
        else
        {
            dEy = 0.0f;
        }
        s_lastDEy = dEy;

        rawTurnCmd = VISION_TRACK_TURN_SIGN *
            (VISION_TRACK_KY * (float)eyControl +
             VISION_TRACK_KA * (float)eaControl +
             VISION_TRACK_KD * dEy);

        rawTurnCmd = VisionTrack_LimitFloat(rawTurnCmd,
            -VISION_TRACK_TURN_LIMIT_CMPS,
             VISION_TRACK_TURN_LIMIT_CMPS);

        s_severity = VisionTrack_CalcSeverity(&s_curFrame, rawTurnCmd);
        s_forwardTarget = VisionTrack_CalcForwardTarget(
            s_severity, s_curFrame.degraded);
        s_forwardCmdFiltered = VisionTrack_MoveToward(
            s_forwardCmdFiltered,
            s_forwardTarget,
            VISION_TRACK_ACCEL_STEP_CMPS,
            VISION_TRACK_DECEL_STEP_CMPS);

        if (s_curFrame.degraded)
        {
            s_turnCmd = VisionTrack_MoveToward(
                s_turnCmd,
                rawTurnCmd,
                VISION_TRACK_DEGRADED_TURN_STEP_CMPS,
                VISION_TRACK_DEGRADED_TURN_STEP_CMPS);
        }
        else
        {
            s_turnCmd = rawTurnCmd;
        }

        VisionTrack_ApplyMotorCommand();
        break;

    case VISION_TRACK_LOST:
        SafeStop();
        if (visionValid && isNewSeq && s_curFrame.sequence != s_lastRecoverSeq)
        {
            s_lastRecoverSeq = s_curFrame.sequence;
            s_lostRecoverCount++;
            if (s_lostRecoverCount >= VISION_TRACK_ACQUIRE_FRAMES)
            {
                s_lostRecoverCount = 0U;
                s_lastRecoverSeq   = 0xFFFFU;
                s_invalidFrameCount = 0U;
                s_lastOverflowCount = App_VisionLink_GetRxOverflowCount();
                EnterState(VISION_TRACK_RUN);
            }
        }
        else if (!visionValid)
        {
            s_lostRecoverCount = 0U;
        }
        break;

    case VISION_TRACK_STOP:
        SafeStop();
        break;

    default:
        SafeStop();
        EnterState(VISION_TRACK_IDLE);
        break;
    }

#if VISION_TRACK_DEBUG_ENABLE
    VisionTrack_DebugPump();
#endif
}

void App_VisionTrack_Task100ms(void)
{
#if VISION_TRACK_DEBUG_ENABLE
    s_debugElapsedMs += 100U;
    if (s_debugElapsedMs >= VISION_TRACK_DEBUG_PERIOD_MS)
    {
        s_debugElapsedMs =
            (uint16_t)(s_debugElapsedMs - VISION_TRACK_DEBUG_PERIOD_MS);

        if (s_debugTxLength == 0U)
        {
            VisionTrack_DebugBuildRecord();
        }
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
            s_acquireCount   = 0U;
            s_lastAcquireSeq = 0xFFFFU;
            App_VisionLink_SendTrackMode();
            EnterState(VISION_TRACK_ACQUIRE);
        }
        break;

    case 3U:
        SafeStop();
        App_VisionLink_SendIdleMode();
        s_invalidFrameCount = 0U;
        EnterState(VISION_TRACK_STOP);
        break;

    case 4U:
        SafeStop();
        App_VisionLink_SendIdleMode();
        App_VisionLink_Reset();
        App_VisionTrack_Init();
        break;

    default:
        break;
    }
}
