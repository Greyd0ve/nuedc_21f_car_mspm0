#include "ti_msp_dl_config.h"

#define delay_ms(X)		delay_cycles( (CPUCLK_FREQ/1000) * (X) )

int main(void)
{
    int i = 0;

    SYSCFG_DL_init();

    while (1)
    {

        // 呼吸灯渐亮过程
        for (i = 0; i <= 999; i++)
        {
            // 设置 LED 亮度
            DL_TimerG_setCaptureCompareValue(PWM_LED_INST,i,GPIO_PWM_LED_C1_IDX);
            delay_ms(1);  // 延迟以控制亮度变化速度
        }
        // 呼吸灯渐暗过程
        for (i = 999; i > 0; i--)
        {
            // 设置 LED 亮度
            DL_TimerG_setCaptureCompareValue(PWM_LED_INST,i,GPIO_PWM_LED_C1_IDX);
						delay_ms(1);  // 延迟以控制亮度变化速度
        }
    }
}
