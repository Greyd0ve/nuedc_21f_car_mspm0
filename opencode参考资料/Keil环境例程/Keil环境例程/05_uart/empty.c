#include "ti_msp_dl_config.h"

#define delay_ms(X)		delay_cycles((CPUCLK_FREQ/1000)*(X));

volatile unsigned int delay_times = 0;
volatile unsigned char uart_data = 0;

void uart0_send_char(char ch);
void uart0_send_string(char* str);

int main(void)
{
    SYSCFG_DL_init();
    //清除串口中断标志
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    //使能串口中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    while (1)
    {
        //LED引脚输出高电平
        DL_GPIO_setPins(LED1_PORT, LED1_PIN_22_PIN);
        delay_ms(500);
        //LED引脚输出低电平
        DL_GPIO_clearPins(LED1_PORT, LED1_PIN_22_PIN);
        delay_ms(500);
    }
}

//串口发送单个字符
void uart0_send_char(char ch)
{
    //当串口0忙的时候等待，不忙的时候再发送传进来的字符
    while( DL_UART_isBusy(UART_0_INST) == true );
    //发送单个字符
    DL_UART_Main_transmitData(UART_0_INST, ch);
}
//串口发送字符串
void uart0_send_string(char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while(*str!=0&&str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart0_send_char(*str++);
    }
}

//串口的中断服务函数
void UART_0_INST_IRQHandler(void)
{
    //如果产生了串口中断
    switch( DL_UART_getPendingInterrupt(UART_0_INST) )
    {
        case DL_UART_IIDX_RX://如果是接收中断
            //接发送过来的数据保存在变量中
            uart_data = DL_UART_Main_receiveData(UART_0_INST);
            //将保存的数据再发送出去
            uart0_send_char(uart_data);
            break;

        default://其他的串口中断
            break;
    }
}