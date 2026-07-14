#include "app_e_car.h"
#include "app_config.h"
#include "app_control.h"
#include "app_line.h"
#include "Board_Config.h"
#include "BeepLed.h"
#include "Encoder.h"
#include "Key.h"
#include "Motor.h"
#include "OLED.h"
#include "PWM.h"
#include "cmsis_compiler.h"
#include <stdint.h>

/* E 题小车状态机。
 * MSPM0 负责循迹、电机控制、编码器计圈、按键、OLED 状态和串口调参。
 * 远程启动保持默认禁用，必须编译期开关显式打开。
 */
#define E_CAR_CONTROL_PERIOD_MS      ECAR_CONTROL_PERIOD_MS /* 状态机控制周期，单位 ms */
#define E_CAR_TARGET_LAP_MIN         1U                     /* 最小目标圈数 */
#define E_CAR_TARGET_LAP_MAX         5U                     /* 最大目标圈数 */
#define E_CAR_TURN_SIGN              1.0f                   /* 角点固定转向方向符号，必要时调正负 */

#define E_CAR_FAULT_NONE             0U /* 无故障 */
#define E_CAR_FAULT_LINE_LOST        1U /* 普通循迹阶段丢线超时 */
#define E_CAR_FAULT_RECOVER_TIMEOUT  2U /* 角点后恢复循迹超时 */
#define E_CAR_FAULT_CORNER_TIMEOUT   3U /* 角点转向阶段超时 */

#define ECAR_LINE_ERROR_NORM          350.0f
#define ECAR_LINE_MIN_SPEED_CMPS      8.0f
#define ECAR_LINE_ERR_SLOW_GAIN       7.0f
#define ECAR_LINE_DERR_SLOW_GAIN      3.0f

#define ECAR_CORNER_TURN_PULSE_DEFAULT  420


ECarParam_t g_eCarParam =
{
    /* 运动默认值偏保守，实际比赛速度建议通过串口逐步调高。 */
    ECAR_DEFAULT_BASE_SPEED_CMPS,
    ECAR_DEFAULT_RECOVER_SPEED_CMPS,
    ECAR_DEFAULT_CORNER_FORWARD_CMPS,
    ECAR_DEFAULT_CORNER_TURN_CMPS,

    0.035f,
		0.080f,
		ECAR_DEFAULT_TURN_LIMIT_CMPS,

    500U,
    120U,
    220U,
    1400U,

    ECAR_DEFAULT_MIN_CORNER_INTERVAL_PULSE,
    ECAR_DEFAULT_LAP_PULSE,

    5U
};

volatile float g_forwardKp = 2.0f;
volatile float g_forwardKi = 0.0f;
volatile float g_forwardKd = 0.0f;

volatile float g_turnKp = 0.0f;
volatile float g_turnKi = 0.0f;
volatile float g_turnKd = 0.0f;

volatile float g_pwmLimit = (float)PWM_MAX_DUTY * 0.55f;
volatile float g_targetForwardSpeed = 0.0f;
volatile float g_targetTurnSpeed = 0.0f;
volatile uint8_t g_carEnable = 0U;

volatile float g_leftSpeed = 0.0f;
volatile float g_rightSpeed = 0.0f;
volatile float g_forwardSpeed = 0.0f;
volatile float g_turnSpeed = 0.0f;
volatile float g_speedPwm = 0.0f;
volatile float g_diffPwm = 0.0f;
volatile float g_forwardSpeedError = 0.0f;
volatile int16_t g_leftEncoderDelta = 0;
volatile int16_t g_rightEncoderDelta = 0;
volatile int16_t g_leftPwm = 0;
volatile int16_t g_rightPwm = 0;

volatile int32_t g_leftEncoderTotal = 0;
volatile int32_t g_rightEncoderTotal = 0;
volatile int32_t g_forwardEncoderTotal = 0;
volatile int32_t g_turnEncoderTotal = 0;

volatile float g_lineBlackLevelF = 1.0f;
volatile float g_lineReverseOrderF = 0.0f;
volatile float g_lineTurnSign = 1.0f;
volatile float g_lineKp = 0.035f;
volatile float g_lineKd = 0.080f;
volatile float g_lineTurnLimit = 12.0f;
volatile float g_lineFilterAlpha = 0.58f;
volatile int16_t g_lineError = 0;
volatile uint8_t g_lineValid = 0U;
volatile uint8_t g_lineMask = 0U;
volatile uint8_t g_lineRawMask = 0U;
volatile uint8_t g_lineBlackCount = 0U;
volatile uint8_t g_lineBadMaskCount = 0U;
volatile uint8_t g_lineZeroMaskCount = 0U;
volatile uint8_t g_lineCornerMaskStableCount = 0U;
volatile int8_t g_lastLineDir = 1;
volatile uint16_t g_lineLostMs = 0U;

/* 私有运行状态。
 * 部分状态会被不同前台任务读取，因此保留 volatile；更新仍不放在长中断中执行。
 */
static volatile ECarState_t s_state = E_CAR_IDLE;
static volatile uint8_t s_targetLap = 1U;
static volatile uint8_t s_lapCount = 0U;
static volatile uint8_t s_cornerCount = 0U;
static volatile uint32_t s_runningTimeMs = 0U;
static volatile uint32_t s_stateMs = 0U;
static volatile uint32_t s_lostMs = 0U;
static volatile uint32_t s_recoverStableMsCount = 0U;
static volatile int32_t s_lastCornerForwardPulse = 0;
static volatile int32_t s_startForwardPulse = 0;
static volatile int32_t s_lapStartForwardPulse = 0;
static volatile int32_t s_currentLapPulse = 0;
static volatile float s_lapProgress = 0.0f;
static volatile uint8_t s_faultCode = E_CAR_FAULT_NONE;
static volatile uint16_t s_promptMs = 0U;
static volatile uint8_t s_cornerCandidateCount = 0U;

static volatile int32_t s_cornerTurnStartPulse = 0;


static float s_lineDError = 0.0f;
static float s_lastLineError = 0.0f;

static float ECar_LimitFloat(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

static uint8_t ECar_IsMotionState(ECarState_t state)
{
    /* 只有这些状态允许累计运动时间并输出运动命令。 */
    return (uint8_t)(state == E_CAR_LINE_RUN ||
                     state == E_CAR_CORNER_ENTER ||
                     state == E_CAR_CORNER_TURN ||
                     state == E_CAR_LINE_RECOVER);
}

static float ECar_CalcAdaptiveBaseSpeed(float error, float dError)
{
    float eAbs;
    float deAbs;
    float speed;

    eAbs = (error >= 0.0f) ? error : -error;
    deAbs = (dError >= 0.0f) ? dError : -dError;

    /*
     * error 原始范围大约 -350~350。
     * 先归一化，再根据偏差和误差变化率降速。
     */
    eAbs = eAbs / ECAR_LINE_ERROR_NORM;
    deAbs = deAbs / ECAR_LINE_ERROR_NORM;

    if (eAbs > 1.0f) eAbs = 1.0f;
    if (deAbs > 1.0f) deAbs = 1.0f;

    speed = g_eCarParam.base_speed
            - ECAR_LINE_ERR_SLOW_GAIN * eAbs
            - ECAR_LINE_DERR_SLOW_GAIN * deAbs;

    return ECar_LimitFloat(speed,
                           ECAR_LINE_MIN_SPEED_CMPS,
                           g_eCarParam.base_speed);
}

static int32_t ECar_GetForwardPulse(void)
{
    /* 前进距离使用左右编码器总脉冲平均值，单位为原始脉冲。 */
    return g_forwardEncoderTotal;
}

static void ECar_SetState(ECarState_t state)
{
    /* 每次状态切换都清零状态持续时间，供各状态超时判断使用。 */
    s_state = state;
    s_stateMs = 0U;
}

static void ECar_PromptStart(uint16_t ms)
{
    /* 短提示由 1ms tick 递减，不阻塞主循环。 */
    s_promptMs = ms;
    if (ms > 0U)
    {
        BeepLed_AllOn();
    }
    else
    {
        BeepLed_AllOff();
    }
}

void ECar_PromptTick1ms(void)
{
    if (s_promptMs > 0U)
    {
        s_promptMs--;
        if ((s_promptMs % 80U) == 0U)
        {
            BeepLed_AllTurn();
        }
        if (s_promptMs == 0U)
        {
            BeepLed_AllOff();
        }
    }
}

static void ECar_SafeStop(void)
{
    /* 本地先明确清目标/使能，再交给统一硬停车函数处理。 */
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 0U;
    App_Control_ForcePWMZero();
}

static void ECar_ClearEncoderTotals(void)
{
    /* 清零总脉冲和待处理增量时要屏蔽编码器中断更新。 */
    __disable_irq();
    g_leftEncoderTotal = 0;
    g_rightEncoderTotal = 0;
    g_forwardEncoderTotal = 0;
    g_turnEncoderTotal = 0;
    g_leftEncoderDelta = 0;
    g_rightEncoderDelta = 0;
    Encoder_ClearAll();
    __enable_irq();
}

static void ECar_SyncLineParams(void)
{
    /* app_line 读取全局循迹参数，这里把 E 题当前调参同步过去。 */
    g_lineKp = g_eCarParam.line_kp;
    g_lineKd = g_eCarParam.line_kd;
    g_lineTurnLimit = g_eCarParam.turn_limit;
}

static void ECar_ResetLineState(void)
{
    /* 同时清空 app_line 内部状态和状态机侧的循迹记忆。 */
    App_Line_ResetState();
    g_lineError = 0;
    g_lineValid = 0U;
    g_lineMask = 0U;
    g_lineRawMask = 0U;
    g_lineBlackCount = 0U;
    g_lineBadMaskCount = 0U;
    g_lineZeroMaskCount = 0U;
    g_lineCornerMaskStableCount = 0U;
    g_lastLineDir = 1;
    g_lineLostMs = 0U;
    s_lastLineError = 0.0f;
    s_cornerCandidateCount = 0U;
}

static void ECar_ResetRunData(void)
{
    /* 新一轮运行不能继承上一轮圈数、角点、故障或编码器历史。 */
    ECar_ClearEncoderTotals();
    ECar_ResetLineState();

    s_lapCount = 0U;
    s_cornerCount = 0U;
    s_runningTimeMs = 0U;
    s_stateMs = 0U;
    s_lostMs = 0U;
    s_recoverStableMsCount = 0U;
    s_lastCornerForwardPulse = 0;
    s_startForwardPulse = 0;
    s_lapStartForwardPulse = 0;
    s_currentLapPulse = 0;
    s_lapProgress = 0.0f;
    s_faultCode = E_CAR_FAULT_NONE;
    s_cornerCandidateCount = 0U;
}

static void ECar_UpdateLapProgress(void)
{
    int32_t pulse;
    int32_t lapPulse;
    int32_t defaultPulse;
    float progress;

    pulse = ECar_GetForwardPulse();
    lapPulse = pulse - s_lapStartForwardPulse;
    if (lapPulse < 0)
    {
        lapPulse = 0;
    }

    s_currentLapPulse = lapPulse;
    defaultPulse = g_eCarParam.lap_pulse_default;
    if (defaultPulse <= 0)
    {
        defaultPulse = 1;
    }

    /* 进度只用于显示和遥测，真正完赛仍以角点计圈为准。 */
    progress = (float)lapPulse / (float)defaultPulse;
    s_lapProgress = ECar_LimitFloat(progress, 0.0f, 0.999f);
}

static float ECar_CalcLineTurnCmd(void)
{
    float error;
    float dError;
    float turn;

    error = (float)g_lineError;
    dError = error - s_lastLineError;
    s_lastLineError = error;
    s_lineDError = dError;

    /*
     * 原始 error 是 -350~350。
     * line_kp / line_kd 建议按 cm/s 差速来调。
     */
    turn = (-g_lineTurnSign) *
           (error * g_eCarParam.line_kp + dError * g_eCarParam.line_kd);

    return ECar_LimitFloat(turn,
                           -g_eCarParam.turn_limit,
                           g_eCarParam.turn_limit);
}

static float ECar_CalcSearchTurnCmd(void)
{
    float sign;

    /* 丢线搜索时向最近一次检测到黑线的方向偏转。 */
    sign = (s_lastLineError >= 0.0f) ? 1.0f : -1.0f;
    return (-g_lineTurnSign) * sign * g_eCarParam.turn_limit * 0.45f;
}

static void ECar_SetSpeedCmd(float forward, float turn)
{
    /* PID 层读取这里的目标：forward 为 cm/s，turn 为左右轮差速 cm/s。 */
    g_targetForwardSpeed = forward;
    g_targetTurnSpeed = turn;
    g_carEnable = 1U;
}

static uint8_t ECar_IsCornerDetected(void)
{
    int32_t pulse;
    int32_t interval;
    uint8_t cornerBlackCountTh;

    cornerBlackCountTh = g_eCarParam.corner_black_count_th;
    if (cornerBlackCountTh == 0U || cornerBlackCountTh > 8U)
    {
        cornerBlackCountTh = 5U;
    }

    if (g_lineBlackCount < cornerBlackCountTh)
    {
        /* 黑色通道数不足阈值时，不认为是角点，只可能是普通线或噪声。 */
        s_cornerCandidateCount = 0U;
        return 0U;
    }

    if (g_lineCornerMaskStableCount < ECAR_CORNER_CONFIRM_COUNT)
    {
        /* 依赖 app_line 的稳定计数过滤单次黑色毛刺。 */
        return 0U;
    }

    pulse = ECar_GetForwardPulse();
    interval = pulse - s_lastCornerForwardPulse;
    if (interval < g_eCarParam.min_corner_interval_pulse)
    {
        /* 间隔不足只清本状态机候选计数，不破坏 app_line 的稳定计数。 */
        s_cornerCandidateCount = 0U;
        return 0U;
    }

    if (s_cornerCandidateCount < 255U)
    {
        s_cornerCandidateCount++;
    }

    return (uint8_t)((s_cornerCandidateCount >= ECAR_CORNER_CONFIRM_COUNT) ? 1U : 0U);
}

static uint8_t ECar_IsStableLineAfterCorner(void)
{
    uint8_t blackCount;

    if (!g_lineValid)
    {
        /* 角点后恢复要求重新看到窄且连续的正常黑线，而不是角点黑块。 */
        return 0U;
    }

    blackCount = g_lineBlackCount;
    if (blackCount == 0U)
    {
        return 0U;
    }
    if (blackCount >= g_eCarParam.corner_black_count_th)
    {
        return 0U;
    }

    return 1U;
}

static void ECar_EnterFault(uint8_t faultCode)
{
    /* 进入故障状态时先停车，再记录故障码并提示。 */
    s_faultCode = faultCode;
    ECar_SafeStop();
    ECar_SetState(E_CAR_FAULT);
    ECar_PromptStart(900U);
}

static void ECar_EnterFinish(void)
{
    /* 完赛状态保持停车，并将显示进度置为接近 100%。 */
    s_lapProgress = 0.999f;
    ECar_SafeStop();
    ECar_SetState(E_CAR_FINISH);
    ECar_PromptStart(600U);
}

static void ECar_EnterRecover(void)
{
    /* 角点转向后进入恢复状态，等待稳定普通黑线。 */
    s_lostMs = 0U;
    s_recoverStableMsCount = 0U;
    ECar_SetState(E_CAR_LINE_RECOVER);
}

static void ECar_HandleLineRun(void)
{
    float turnCmd;
    float speedCmd;

    if (g_lineValid)
    {
        s_lostMs = 0U;
        turnCmd = ECar_CalcLineTurnCmd();
        speedCmd = ECar_CalcAdaptiveBaseSpeed((float)g_lineError, s_lineDError);
    }
    else if (g_lineBlackCount >= g_eCarParam.corner_black_count_th)
    {
        s_lostMs = 0U;
        turnCmd = 0.0f;
        speedCmd = g_eCarParam.corner_forward_speed;
    }
    else
    {
        if (s_lostMs < 60000U)
        {
            s_lostMs += E_CAR_CONTROL_PERIOD_MS;
        }

        if (s_lostMs >= g_eCarParam.lost_timeout_ms)
        {
            ECar_EnterFault(E_CAR_FAULT_LINE_LOST);
            return;
        }

        turnCmd = ECar_CalcSearchTurnCmd();
        speedCmd = g_eCarParam.recover_speed;
    }

    ECar_SetSpeedCmd(speedCmd, turnCmd);

    if (ECar_IsCornerDetected())
    {
        ECar_SetState(E_CAR_CORNER_ENTER);
        s_cornerCandidateCount = 0U;
    }
}

static void ECar_HandleCornerEnter(void)
{
    int32_t pulse;

    /* 每进入一次 CORNER_ENTER 只计一个角点。 */
    pulse = ECar_GetForwardPulse();
    s_lastCornerForwardPulse = pulse;
    s_cornerCandidateCount = 0U;
		s_cornerTurnStartPulse = g_turnEncoderTotal;
    if (s_cornerCount < 250U)
    {
        s_cornerCount++;
    }

    if ((s_cornerCount % 4U) == 0U)
    {
        /* E 题方形赛道按四个角点计一圈。 */
        if (s_lapCount < 250U)
        {
            s_lapCount++;
        }

        if (s_lapCount >= s_targetLap)
        {
            ECar_EnterFinish();
            return;
        }

        s_lapStartForwardPulse = pulse;
        s_currentLapPulse = 0;
        s_lapProgress = 0.0f;
    }

    s_lostMs = 0U;
    s_recoverStableMsCount = 0U;
    ECar_SetState(E_CAR_CORNER_TURN);
}

static void ECar_HandleCornerTurn(void)
{
	
	
		
			int32_t turnPulse;
			int32_t turnDelta;

			turnPulse = g_turnEncoderTotal - s_cornerTurnStartPulse;
			if (turnPulse < 0)
			{
					turnDelta = -turnPulse;
			}
			else
			{
					turnDelta = turnPulse;
			}

			ECar_SetSpeedCmd(0.0f,
											 g_eCarParam.corner_turn_speed * E_CAR_TURN_SIGN);

			if (turnDelta >= ECAR_CORNER_TURN_PULSE_DEFAULT)
			{
					ECar_EnterRecover();
					return;
			}
	
    /* 角点黑块区域采用开环转弯，保证动作可预测。 */
    ECar_SetSpeedCmd(g_eCarParam.corner_forward_speed,
                     g_eCarParam.corner_turn_speed * E_CAR_TURN_SIGN);

    if (s_stateMs >= g_eCarParam.corner_min_turn_ms)
    {
        if (ECar_IsStableLineAfterCorner())
        {
            ECar_EnterRecover();
            return;
        }
    }

    if (s_stateMs >= g_eCarParam.corner_max_turn_ms)
    {
        if (g_lineValid)
        {
            ECar_EnterRecover();
        }
        else
        {
            ECar_EnterFault(E_CAR_FAULT_CORNER_TIMEOUT);
        }
    }
}

static void ECar_HandleRecover(void)
{
    float turnCmd;
    uint32_t recoverTimeoutMs;

    if (g_lineValid)
    {
        /* 重新看到正常黑线后，用恢复速度闭环跟线。 */
        s_lostMs = 0U;
        turnCmd = ECar_CalcLineTurnCmd();
    }
    else
    {
        if (s_lostMs < 60000U)
        {
            s_lostMs += E_CAR_CONTROL_PERIOD_MS;
        }

        if (s_lostMs >= g_eCarParam.lost_timeout_ms)
        {
            ECar_EnterFault(E_CAR_FAULT_RECOVER_TIMEOUT);
            return;
        }

        turnCmd = ECar_CalcSearchTurnCmd();
    }

    ECar_SetSpeedCmd(g_eCarParam.recover_speed, turnCmd);

    if (ECar_IsStableLineAfterCorner())
    {
        if (s_recoverStableMsCount < 60000U)
        {
            s_recoverStableMsCount += E_CAR_CONTROL_PERIOD_MS;
        }

        if (s_recoverStableMsCount >= g_eCarParam.recover_stable_ms)
        {
            ECar_SetState(E_CAR_LINE_RUN);
            return;
        }
    }
    else
    {
        s_recoverStableMsCount = 0U;
    }

    recoverTimeoutMs = (uint32_t)g_eCarParam.lost_timeout_ms +
                       (uint32_t)g_eCarParam.recover_stable_ms + 500U;
    /* 恢复阶段有独立超时，避免小车长时间原地找线。 */
    if (s_stateMs >= recoverTimeoutMs)
    {
        ECar_EnterFault(E_CAR_FAULT_RECOVER_TIMEOUT);
    }
}

void ECar_Init(void)
{
    ECar_SyncLineParams();
    s_targetLap = 1U;
    ECar_ResetRunData();
    ECar_SafeStop();
    ECar_SetState(E_CAR_IDLE);
}

void ECar_Reset(void)
{
    ECar_SyncLineParams();
    ECar_SafeStop();
    ECar_ResetRunData();
    ECar_SetState(E_CAR_READY);
}

void ECar_Start(void)
{
    /* 启动状态机前先限制目标圈数范围。 */
    if (s_targetLap < E_CAR_TARGET_LAP_MIN)
    {
        s_targetLap = E_CAR_TARGET_LAP_MIN;
    }
    if (s_targetLap > E_CAR_TARGET_LAP_MAX)
    {
        s_targetLap = E_CAR_TARGET_LAP_MAX;
    }

#if ECAR_BOARD_TEST_MODE
    /* 板级测试模式下，启动命令不会进入自动循迹运行。 */
    ECar_SyncLineParams();
    ECar_SafeStop();
    ECar_ResetRunData();
    ECar_SetState(E_CAR_READY);
    ECar_PromptStart(120U);
    return;
#endif

    ECar_SyncLineParams();
    ECar_SafeStop();
    ECar_ResetRunData();

    s_startForwardPulse = ECar_GetForwardPulse();
    s_lapStartForwardPulse = s_startForwardPulse;
    s_lastCornerForwardPulse = s_startForwardPulse;
    ECar_SetState(E_CAR_LINE_RUN);
    g_carEnable = 1U;
    ECar_PromptStart(120U);
}

void ECar_Stop(void)
{
    ECar_SafeStop();
    ECar_ResetRunData();
    ECar_SetState(E_CAR_READY);
    ECar_PromptStart(220U);
}

void ECar_Control10ms(void)
{
    /* 灰度读取、状态机和 PID 均在前台 10ms 任务执行，不放进定时器 ISR。 */
    ECar_SyncLineParams();
    App_Line_Update();

    if (ECar_IsMotionState(s_state))
    {
        if (s_runningTimeMs <= 0xFFFFFFFFU - E_CAR_CONTROL_PERIOD_MS)
        {
            s_runningTimeMs += E_CAR_CONTROL_PERIOD_MS;
        }
    }

    if (s_stateMs <= 0xFFFFFFFFU - E_CAR_CONTROL_PERIOD_MS)
    {
        s_stateMs += E_CAR_CONTROL_PERIOD_MS;
    }

    ECar_UpdateLapProgress();
    App_Control_UpdatePIDParam();

    switch (s_state)
    {
        case E_CAR_IDLE:
        case E_CAR_READY:
        case E_CAR_FINISH:
        case E_CAR_FAULT:
            /* 非运动状态持续强制 PWM 为 0。 */
            ECar_SafeStop();
            break;

        case E_CAR_LINE_RUN:
            ECar_HandleLineRun();
            break;

        case E_CAR_CORNER_ENTER:
            ECar_HandleCornerEnter();
            break;

        case E_CAR_CORNER_TURN:
            ECar_HandleCornerTurn();
            break;

        case E_CAR_LINE_RECOVER:
            ECar_HandleRecover();
            break;

        default:
            ECar_EnterFault(E_CAR_FAULT_LINE_LOST);
            break;
    }

    App_Control_ApplyMotorOutput();
}

void ECar_KeyProcess(void)
{
    uint8_t key;

    key = Key_GetNum();
    if (key == 0U)
    {
        return;
    }

    if (key == 1U)
    {
        /* K1：仅在非运动状态下循环选择目标圈数。 */
        if (!ECar_IsMotionState(s_state) && s_state != E_CAR_FAULT)
        {
            s_targetLap++;
            if (s_targetLap > E_CAR_TARGET_LAP_MAX)
            {
                s_targetLap = E_CAR_TARGET_LAP_MIN;
            }
            ECar_SafeStop();
            ECar_SetState(E_CAR_READY);
            ECar_PromptStart((uint16_t)(80U + 30U * s_targetLap));
        }
        return;
    }

    if (key == 2U)
    {
        /* K2：仅在非运动、非故障状态下启动。 */
        if (!ECar_IsMotionState(s_state) && s_state != E_CAR_FAULT)
        {
            ECar_Start();
        }
        return;
    }

    if (key == 3U)
    {
        /* K3：本地停车。 */
        ECar_Stop();
        return;
    }

    if (key == 4U)
    {
        /* K4：故障时复位；非故障时停车并回到 IDLE。 */
        if (s_state == E_CAR_FAULT)
        {
            ECar_Reset();
            ECar_PromptStart(180U);
        }
        else
        {
            ECar_SafeStop();
            ECar_ResetRunData();
            ECar_SetState(E_CAR_IDLE);
            ECar_PromptStart(260U);
        }
    }
}

void ECar_ShowStatus(void)
{
    uint8_t progressPercent;
    int32_t lineErr;

    if (!g_carEnable && (s_state == E_CAR_IDLE || s_state == E_CAR_READY))
    {
        OLED_Clear();
        OLED_ShowString(0, 0, "E-Car MSPM0", OLED_8X16);
#if BOARD_OLED_USE_H8_SPI
        OLED_ShowString(0, 16, "OLED H8 SPI", OLED_8X16);
        OLED_ShowString(0, 32, "SCL PB9 SDA PB8", OLED_8X16);
#elif BOARD_OLED_USE_H8_I2C
        OLED_ShowString(0, 16, "OLED H8 I2C", OLED_8X16);
        OLED_ShowString(0, 32, "SCL PB9 SDA PB8", OLED_8X16);
#else
        OLED_ShowString(0, 16, "OLED I2C OK", OLED_8X16);
        OLED_ShowString(0, 32, "SCL PA1 SDA PA0", OLED_8X16);
#endif
        OLED_ShowString(0, 48, "Motor OFF", OLED_8X16);
        OLED_Update();
        return;
    }

    /* 限制显示数值范围，避免 OLED 小字段溢出。 */
    progressPercent = (uint8_t)(s_lapProgress * 100.0f);
    if (progressPercent > 99U)
    {
        progressPercent = 99U;
    }

    lineErr = (int32_t)g_lineError;
    if (lineErr > 999)
    {
        lineErr = 999;
    }
    if (lineErr < -999)
    {
        lineErr = -999;
    }

    OLED_Clear();
    OLED_ShowString(0, 0, "E CAR", OLED_8X16);

    OLED_ShowString(0, 16, "N:", OLED_8X16);
    OLED_ShowNum(16, 16, s_targetLap, 1, OLED_8X16);
    OLED_ShowString(40, 16, "L:", OLED_8X16);
    OLED_ShowNum(56, 16, s_lapCount, 1, OLED_8X16);

    OLED_ShowString(0, 32, "C:", OLED_8X16);
    OLED_ShowNum(16, 32, s_cornerCount, 2, OLED_8X16);
    OLED_ShowString(48, 32, "S:", OLED_8X16);
    OLED_ShowNum(64, 32, (uint8_t)s_state, 1, OLED_8X16);
    OLED_ShowString(88, 32, "F:", OLED_8X16);
    OLED_ShowNum(104, 32, s_faultCode, 1, OLED_8X16);

    OLED_ShowString(0, 48, "P:", OLED_8X16);
    OLED_ShowNum(16, 48, progressPercent, 2, OLED_8X16);
    OLED_ShowString(32, 48, "% E:", OLED_8X16);
    OLED_ShowSignedNum(64, 48, lineErr, 3, OLED_8X16);

    OLED_Update();
}

float ECar_GetLapProgress(void)
{
    return s_lapProgress;
}

uint8_t ECar_GetLapCount(void)
{
    return s_lapCount;
}

uint8_t ECar_GetTargetLap(void)
{
    return s_targetLap;
}

ECarState_t ECar_GetState(void)
{
    return s_state;
}

uint8_t ECar_SetTargetLap(uint8_t lap)
{
    if (lap < E_CAR_TARGET_LAP_MIN || lap > E_CAR_TARGET_LAP_MAX)
    {
        return 0U;
    }

    if (ECar_IsMotionState(s_state))
    {
        return 0U;
    }

    s_targetLap = lap;
    if (s_state != E_CAR_FAULT)
    {
        ECar_SafeStop();
        ECar_SetState(E_CAR_READY);
    }

    return 1U;
}

uint8_t ECar_GetCornerCount(void)
{
    return s_cornerCount;
}

uint8_t ECar_GetFaultCode(void)
{
    return s_faultCode;
}

uint32_t ECar_GetRunningTimeMs(void)
{
    return s_runningTimeMs;
}
