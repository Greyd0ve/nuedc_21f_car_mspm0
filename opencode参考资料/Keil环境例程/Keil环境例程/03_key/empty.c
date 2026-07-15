#include "ti_msp_dl_config.h"

int main(void)
{

        SYSCFG_DL_init();

        while (1)
        {
                //如果读取到的引脚值等于0，说明PB21引脚为低电平
                if( DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) == 0 )
                {
                        DL_GPIO_setPins(LED1_PORT,LED1_PIN_22_PIN);  //LED控制输出高电平
                }
                else//如果PA21引脚为高电平
                {
                        DL_GPIO_clearPins(LED1_PORT,LED1_PIN_22_PIN);//LED控制输出低电平
                }
        }
}