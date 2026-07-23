#include "PWM.h"
#include "Board_Config.h"

/* TB6612 电机 PWM 定时器比较值封装。
 * 对外占空比范围为 0..PWM_MAX_DUTY，与底层定时器周期计数解耦。
 */
static uint16_t PWM_Clamp(uint16_t compare)
{
    return (compare > PWM_MAX_DUTY) ? PWM_MAX_DUTY : compare;
}

static uint32_t PWM_ToTimerCompare(uint16_t duty)
{
    uint32_t dutyCounts;

    duty = PWM_Clamp(duty);
    /* 当前 SysConfig 下 PWM 输出为低有效，因此比较值需要反向换算。 */
    dutyCounts = ((uint32_t)MOTOR_PWM_PERIOD_COUNTS * (uint32_t)duty) / PWM_MAX_DUTY;

    if (dutyCounts == 0U)
    {
        return MOTOR_PWM_PERIOD_COUNTS;
    }
    if (dutyCounts >= MOTOR_PWM_PERIOD_COUNTS)
    {
        return 0U;
    }
    return MOTOR_PWM_PERIOD_COUNTS - dutyCounts;
}

void PWM_Init(void)
{
    /* 启动 PWM 定时器前先把两个通道占空比置 0。 */
    PWM_SetCompareA(0U);
    PWM_SetCompareB(0U);
    DL_TimerG_startCounter(MOTOR_PWM_TIMER_INST);
}

void PWM_SetCompareA(uint16_t compare)
{
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_TIMER_INST, PWM_ToTimerCompare(compare), MOTOR_PWM_LEFT_CC_INDEX);
}

void PWM_SetCompareB(uint16_t compare)
{
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_TIMER_INST, PWM_ToTimerCompare(compare), MOTOR_PWM_RIGHT_CC_INDEX);
}
