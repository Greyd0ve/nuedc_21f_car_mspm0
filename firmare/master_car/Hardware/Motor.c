#include "Motor.h"
#include "Board_Config.h"
#include "PWM.h"

static int16_t Motor_ClampPWM(int16_t pwm)
{
    if (pwm > (int16_t)PWM_MAX_DUTY)
    {
        return (int16_t)PWM_MAX_DUTY;
    }
    if (pwm < -(int16_t)PWM_MAX_DUTY)
    {
        return -(int16_t)PWM_MAX_DUTY;
    }
    return pwm;
}

static void Motor_SetDirection(
    GPIO_Regs *in1Port, uint32_t in1Pin, GPIO_Regs *in2Port, uint32_t in2Pin, int16_t pwm)
{
    if (pwm > 0)
    {
        DL_GPIO_setPins(in1Port, in1Pin);
        DL_GPIO_clearPins(in2Port, in2Pin);
    }
    else if (pwm < 0)
    {
        DL_GPIO_clearPins(in1Port, in1Pin);
        DL_GPIO_setPins(in2Port, in2Pin);
    }
    else
    {
        DL_GPIO_clearPins(in1Port, in1Pin);
        DL_GPIO_clearPins(in2Port, in2Pin);
    }
}

void Motor_Init(void)
{
    PWM_Init();
    Motor_StopAll();
}

void Motor_SetLeftPWM(int16_t pwm)
{
    int16_t signedPwm = Motor_ClampPWM((int16_t)(pwm * LEFT_MOTOR_DIR));
    uint16_t duty = (signedPwm < 0) ? (uint16_t)(-signedPwm) : (uint16_t)signedPwm;

    Motor_SetDirection(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, signedPwm);
    PWM_SetCompareA(duty);
}

void Motor_SetRightPWM(int16_t pwm)
{
    int16_t signedPwm = Motor_ClampPWM((int16_t)(pwm * RIGHT_MOTOR_DIR));
    uint16_t duty = (signedPwm < 0) ? (uint16_t)(-signedPwm) : (uint16_t)signedPwm;

    Motor_SetDirection(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, signedPwm);
    PWM_SetCompareB(duty);
}

void Motor_SetPWM(int16_t leftPwm, int16_t rightPwm)
{
    Motor_SetLeftPWM(leftPwm);
    Motor_SetRightPWM(rightPwm);
}

void Motor_StopAll(void)
{
    PWM_SetCompareA(0U);
    PWM_SetCompareB(0U);
    DL_GPIO_clearPins(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN);
    DL_GPIO_clearPins(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN);
    DL_GPIO_clearPins(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN);
    DL_GPIO_clearPins(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN);
}
