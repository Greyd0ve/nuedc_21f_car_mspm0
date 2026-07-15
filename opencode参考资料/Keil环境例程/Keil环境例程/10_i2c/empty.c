#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "bsp_i2c.h"

#define delay_ms(X)	delay_cycles((CPUCLK_FREQ/1000)*(X))

#define T_ADDR     0xf3   // 温度
#define PH_ADDR    0xf5   // 湿度

/******************************串口重定向***************************************/
#if !defined(__MICROLIB)
//不使用微库的话就需要添加下面的函数
#if (__ARMCLIB_VERSION <= 6000000)
//如果编译器是AC5  就定义下面这个结构体
struct __FILE
{
        int handle;
};
#endif

FILE __stdout;

//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
        x = x;
}
#endif
//printf函数重定义
int fputc(int ch, FILE *stream)
{
        //当串口0忙的时候等待，不忙的时候再发送传进来的字符
        while( DL_UART_isBusy(UART_0_INST) == true );

        DL_UART_Main_transmitData(UART_0_INST, ch);

        return ch;
}
/*********************************************************************/

int main(void)
{
    SYSCFG_DL_init();

    printf("SHT20 Start!!\r\n");

    while(1)
    {
        float TEMP = SHT20_Read(T_ADDR);
        float PH   = SHT20_Read(PH_ADDR);

        printf("Temp = %.2f C\r\n", TEMP);
        printf("Humi = %.2f %%RH\r\n", PH);

        printf("\n");
        delay_ms(1000);
    }
}