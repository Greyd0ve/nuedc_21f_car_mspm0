#ifndef __BSP_I2C_H__
#define __BSP_I2C_H__

#include "ti_msp_dl_config.h"
#include "stdio.h"

//设置SDA输出模式
#define SDA_OUT()   {                                                \
                        DL_GPIO_initDigitalOutput(I2C_SDA_IOMUX);    \
                        DL_GPIO_setPins(I2C_PORT, I2C_SDA_PIN);      \
                        DL_GPIO_enableOutput(I2C_PORT, I2C_SDA_PIN); \
                    }
//设置SDA输入模式
#define SDA_IN()    { DL_GPIO_initDigitalInput(I2C_SDA_IOMUX); }
//获取SDA引脚的电平变化
#define SDA_GET()   ( ( ( DL_GPIO_readPins(I2C_PORT,I2C_SDA_PIN) & I2C_SDA_PIN ) > 0 ) ? 1 : 0 )
//SDA与SCL输出
#define SDA(x)      ( (x) ? (DL_GPIO_setPins(I2C_PORT,I2C_SDA_PIN)) : (DL_GPIO_clearPins(I2C_PORT,I2C_SDA_PIN)) )
#define SCL(x)      ( (x) ? (DL_GPIO_setPins(I2C_PORT,I2C_SCL_PIN)) : (DL_GPIO_clearPins(I2C_PORT,I2C_SCL_PIN)) )

void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Ack(uint8_t ack);
uint8_t IIC_Wait_Ack(void);
void IIC_Send_Byte(uint8_t dat);
unsigned char IIC_Read_Byte(void);

float SHT20_Read(uint8_t regaddr);

#endif