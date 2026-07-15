#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "bsp_W25Q128.h"

#define delay_ms(X)	delay_cycles((CPUCLK_FREQ/1000)*(X))

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
	unsigned char buff[10] = {0};

	SYSCFG_DL_init();

  delay_ms(1000);//等待部署

	//读取W25Q128的ID
	printf("ID = %X\r\n",W25Q128_readID());

	//读取0地址的5个字节数据到buff
	W25Q128_read(buff, 0, 5);
	//串口输出读取的数据
	printf("buff = %s\r\n",buff);

	//往0地址写入5个字节长度的数据 ABCD
	W25Q128_write((uint8_t*)"ABCD", 0, 5);

	delay_ms(1);//等待稳定

	//读取0地址的5个字节数据到buff
	W25Q128_read(buff, 0, 5);

	//串口输出读取的数据
	printf("buff = %s\r\n",buff);

	while(1)
	{

	}
}