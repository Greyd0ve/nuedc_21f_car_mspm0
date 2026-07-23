#include "Servo.h"
#include "Board_Config.h"

static uint32_t Servo_ChannelToIndex(uint8_t channel)
{
    switch (channel)
    {
        case 0U:
            return SERVO1_PWM_CC_INDEX;
        case 1U:
            return SERVO2_PWM_CC_INDEX;
        case 2U:
            return SERVO3_PWM_CC_INDEX;
        default:
            return SERVO4_PWM_CC_INDEX;
    }
}

static uint16_t Servo_ClampPulse(uint16_t pulseUs)
{
    if (pulseUs < SERVO_MIN_PULSE_US)
    {
        return SERVO_MIN_PULSE_US;
    }
    if (pulseUs > SERVO_MAX_PULSE_US)
    {
        return SERVO_MAX_PULSE_US;
    }
    return pulseUs;
}

void Servo_Init(void)
{
    Servo_DisableAll();
    DL_TimerA_startCounter(SERVO_PWM_TIMER_INST);
}

void Servo_DisableAll(void)
{
    uint8_t ch;

    for (ch = 0U; ch < SERVO_CHANNEL_COUNT; ch++)
    {
        DL_TimerA_setCaptureCompareValue(
            SERVO_PWM_TIMER_INST, SERVO_PWM_PERIOD_US, Servo_ChannelToIndex(ch));
    }
}

void Servo_SetPulseUs(uint8_t channel, uint16_t pulseUs)
{
    uint32_t compare;

    if (channel >= SERVO_CHANNEL_COUNT)
    {
        return;
    }

    pulseUs = Servo_ClampPulse(pulseUs);
    compare = (uint32_t)SERVO_PWM_PERIOD_US - (uint32_t)pulseUs;
    DL_TimerA_setCaptureCompareValue(SERVO_PWM_TIMER_INST, compare, Servo_ChannelToIndex(channel));
}

void Servo_SetAngle(uint8_t channel, int16_t angleDeg)
{
    uint32_t pulse;

    if (angleDeg < 0)
    {
        angleDeg = 0;
    }
    if (angleDeg > 180)
    {
        angleDeg = 180;
    }

    pulse = (uint32_t)SERVO_MIN_PULSE_US +
            (((uint32_t)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * (uint32_t)angleDeg) / 180U);
    Servo_SetPulseUs(channel, (uint16_t)pulse);
}
