#include "ti_msp_dl_config.h"
#include "stdio.h"

#define delay_ms(X)	delay_cycles((CPUCLK_FREQ/1000)*(X))

volatile uint16_t ADC_VALUE[20];

unsigned int adc_getValue(unsigned int number);//读取ADC的数据

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
        unsigned int adc_value = 0;
        unsigned int voltage_value = 0;
        int i = 0;

        SYSCFG_DL_init();

        //设置DMA搬运的起始地址
        DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &ADC0->ULLMEM.MEMRES[0]);
        //设置DMA搬运的目的地址
        DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &ADC_VALUE[0]);
        //开启DMA
        DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
        //开启ADC转换
        DL_ADC12_startConversion(ADC_VOLTAGE_INST);

        printf("adc_dma Demo start\r\n");

        while (1)
        {
                //获取ADC数据
                adc_value = adc_getValue(10);
                printf("adc value:%d\r\n", adc_value);

                //将ADC采集的数据换算为电压
                voltage_value = (int)((adc_value/4095.0*3.3)*100);

                printf("voltage value:%d.%d%d\r\n",
                voltage_value/100,
                voltage_value/10%10,
                voltage_value%10 );

                delay_ms(1000);
        }
}

//读取ADC的数据
unsigned int adc_getValue(unsigned int number)
{
        unsigned int gAdcResult = 0;
        unsigned char i = 0;

        //采集多次累加
        for( i = 0; i < number; i++ )
        {
                gAdcResult += ADC_VALUE[i];
        }
        //均值滤波
        gAdcResult /= number;

        return gAdcResult;
}