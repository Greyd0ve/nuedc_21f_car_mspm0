/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_MOTOR */
#define PWM_MOTOR_INST                                                     TIMG0
#define PWM_MOTOR_INST_IRQHandler                               TIMG0_IRQHandler
#define PWM_MOTOR_INST_INT_IRQN                                 (TIMG0_INT_IRQn)
#define PWM_MOTOR_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_C0_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C0_PIN                                     DL_GPIO_PIN_12
#define GPIO_PWM_MOTOR_C0_IOMUX                                  (IOMUX_PINCM34)
#define GPIO_PWM_MOTOR_C0_IOMUX_FUNC                 IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_PWM_MOTOR_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_C1_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C1_PIN                                     DL_GPIO_PIN_13
#define GPIO_PWM_MOTOR_C1_IOMUX                                  (IOMUX_PINCM35)
#define GPIO_PWM_MOTOR_C1_IOMUX_FUNC                 IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_PWM_MOTOR_C1_IDX                                DL_TIMER_CC_1_INDEX

/* Defines for PWM_SERVO */
#define PWM_SERVO_INST                                                     TIMA0
#define PWM_SERVO_INST_IRQHandler                               TIMA0_IRQHandler
#define PWM_SERVO_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define PWM_SERVO_INST_CLK_FREQ                                          1000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_SERVO_C0_PORT                                             GPIOA
#define GPIO_PWM_SERVO_C0_PIN                                     DL_GPIO_PIN_21
#define GPIO_PWM_SERVO_C0_IOMUX                                  (IOMUX_PINCM46)
#define GPIO_PWM_SERVO_C0_IOMUX_FUNC                 IOMUX_PINCM46_PF_TIMA0_CCP0
#define GPIO_PWM_SERVO_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_SERVO_C1_PORT                                             GPIOA
#define GPIO_PWM_SERVO_C1_PIN                                     DL_GPIO_PIN_22
#define GPIO_PWM_SERVO_C1_IOMUX                                  (IOMUX_PINCM47)
#define GPIO_PWM_SERVO_C1_IOMUX_FUNC                 IOMUX_PINCM47_PF_TIMA0_CCP1
#define GPIO_PWM_SERVO_C1_IDX                                DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 2 */
#define GPIO_PWM_SERVO_C2_PORT                                             GPIOA
#define GPIO_PWM_SERVO_C2_PIN                                     DL_GPIO_PIN_15
#define GPIO_PWM_SERVO_C2_IOMUX                                  (IOMUX_PINCM37)
#define GPIO_PWM_SERVO_C2_IOMUX_FUNC                 IOMUX_PINCM37_PF_TIMA0_CCP2
#define GPIO_PWM_SERVO_C2_IDX                                DL_TIMER_CC_2_INDEX
/* GPIO defines for channel 3 */
#define GPIO_PWM_SERVO_C3_PORT                                             GPIOA
#define GPIO_PWM_SERVO_C3_PIN                                     DL_GPIO_PIN_17
#define GPIO_PWM_SERVO_C3_IOMUX                                  (IOMUX_PINCM39)
#define GPIO_PWM_SERVO_C3_IOMUX_FUNC                 IOMUX_PINCM39_PF_TIMA0_CCP3
#define GPIO_PWM_SERVO_C3_IDX                                DL_TIMER_CC_3_INDEX



/* Defines for TIMER_SYS */
#define TIMER_SYS_INST                                                   (TIMG6)
#define TIMER_SYS_INST_IRQHandler                               TIMG6_IRQHandler
#define TIMER_SYS_INST_INT_IRQN                                 (TIMG6_INT_IRQn)
#define TIMER_SYS_INST_LOAD_VALUE                                         (124U)




/* Defines for I2C_SHARED */
#define I2C_SHARED_INST                                                     I2C0
#define I2C_SHARED_INST_IRQHandler                               I2C0_IRQHandler
#define I2C_SHARED_INST_INT_IRQN                                   I2C0_INT_IRQn
#define I2C_SHARED_BUS_SPEED_HZ                                           100000
#define GPIO_I2C_SHARED_SDA_PORT                                           GPIOA
#define GPIO_I2C_SHARED_SDA_PIN                                   DL_GPIO_PIN_28
#define GPIO_I2C_SHARED_IOMUX_SDA                                 (IOMUX_PINCM3)
#define GPIO_I2C_SHARED_IOMUX_SDA_FUNC                  IOMUX_PINCM3_PF_I2C0_SDA
#define GPIO_I2C_SHARED_SCL_PORT                                           GPIOA
#define GPIO_I2C_SHARED_SCL_PIN                                   DL_GPIO_PIN_31
#define GPIO_I2C_SHARED_IOMUX_SCL                                 (IOMUX_PINCM6)
#define GPIO_I2C_SHARED_IOMUX_SCL_FUNC                  IOMUX_PINCM6_PF_I2C0_SCL


/* Defines for UART_DEBUG */
#define UART_DEBUG_INST                                                    UART1
#define UART_DEBUG_INST_FREQUENCY                                       32000000
#define UART_DEBUG_INST_IRQHandler                              UART1_IRQHandler
#define UART_DEBUG_INST_INT_IRQN                                  UART1_INT_IRQn
#define GPIO_UART_DEBUG_RX_PORT                                            GPIOB
#define GPIO_UART_DEBUG_TX_PORT                                            GPIOB
#define GPIO_UART_DEBUG_RX_PIN                                     DL_GPIO_PIN_7
#define GPIO_UART_DEBUG_TX_PIN                                     DL_GPIO_PIN_6
#define GPIO_UART_DEBUG_IOMUX_RX                                 (IOMUX_PINCM24)
#define GPIO_UART_DEBUG_IOMUX_TX                                 (IOMUX_PINCM23)
#define GPIO_UART_DEBUG_IOMUX_RX_FUNC                  IOMUX_PINCM24_PF_UART1_RX
#define GPIO_UART_DEBUG_IOMUX_TX_FUNC                  IOMUX_PINCM23_PF_UART1_TX
#define UART_DEBUG_BAUD_RATE                                              (9600)
#define UART_DEBUG_IBRD_32_MHZ_9600_BAUD                                   (208)
#define UART_DEBUG_FBRD_32_MHZ_9600_BAUD                                    (21)
/* Defines for UART_K230 */
#define UART_K230_INST                                                     UART0
#define UART_K230_INST_FREQUENCY                                        32000000
#define UART_K230_INST_IRQHandler                               UART0_IRQHandler
#define UART_K230_INST_INT_IRQN                                   UART0_INT_IRQn
#define GPIO_UART_K230_RX_PORT                                             GPIOA
#define GPIO_UART_K230_TX_PORT                                             GPIOA
#define GPIO_UART_K230_RX_PIN                                      DL_GPIO_PIN_1
#define GPIO_UART_K230_TX_PIN                                      DL_GPIO_PIN_0
#define GPIO_UART_K230_IOMUX_RX                                   (IOMUX_PINCM2)
#define GPIO_UART_K230_IOMUX_TX                                   (IOMUX_PINCM1)
#define GPIO_UART_K230_IOMUX_RX_FUNC                    IOMUX_PINCM2_PF_UART0_RX
#define GPIO_UART_K230_IOMUX_TX_FUNC                    IOMUX_PINCM1_PF_UART0_TX
#define UART_K230_BAUD_RATE                                             (115200)
#define UART_K230_IBRD_32_MHZ_115200_BAUD                                   (17)
#define UART_K230_FBRD_32_MHZ_115200_BAUD                                   (23)





/* Defines for L_IN1: GPIOB.17 with pinCMx 43 on package pin 14 */
#define GPIO_MOTOR_L_IN1_PORT                                            (GPIOB)
#define GPIO_MOTOR_L_IN1_PIN                                    (DL_GPIO_PIN_17)
#define GPIO_MOTOR_L_IN1_IOMUX                                   (IOMUX_PINCM43)
/* Defines for L_IN2: GPIOB.19 with pinCMx 45 on package pin 16 */
#define GPIO_MOTOR_L_IN2_PORT                                            (GPIOB)
#define GPIO_MOTOR_L_IN2_PIN                                    (DL_GPIO_PIN_19)
#define GPIO_MOTOR_L_IN2_IOMUX                                   (IOMUX_PINCM45)
/* Defines for R_IN1: GPIOA.16 with pinCMx 38 on package pin 9 */
#define GPIO_MOTOR_R_IN1_PORT                                            (GPIOA)
#define GPIO_MOTOR_R_IN1_PIN                                    (DL_GPIO_PIN_16)
#define GPIO_MOTOR_R_IN1_IOMUX                                   (IOMUX_PINCM38)
/* Defines for R_IN2: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_MOTOR_R_IN2_PORT                                            (GPIOB)
#define GPIO_MOTOR_R_IN2_PIN                                    (DL_GPIO_PIN_24)
#define GPIO_MOTOR_R_IN2_IOMUX                                   (IOMUX_PINCM52)
/* Port definition for Pin Group GPIO_ENCODER */
#define GPIO_ENCODER_PORT                                                (GPIOB)

/* Defines for L_A: GPIOB.5 with pinCMx 18 */
// pins affected by this interrupt request:["L_A","L_B","R_A","R_B"]
#define GPIO_ENCODER_INT_IRQN                                   (GPIOB_INT_IRQn)
#define GPIO_ENCODER_INT_IIDX                   (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_ENCODER_L_A_IIDX                                (DL_GPIO_IIDX_DIO5)
#define GPIO_ENCODER_L_A_PIN                                     (DL_GPIO_PIN_5)
#define GPIO_ENCODER_L_A_IOMUX                                   (IOMUX_PINCM18)
/* Defines for L_B: GPIOB.12 with pinCMx 29 */
#define GPIO_ENCODER_L_B_PIN                                    (DL_GPIO_PIN_12)
#define GPIO_ENCODER_L_B_IOMUX                                   (IOMUX_PINCM29)
/* Defines for R_A: GPIOB.8 with pinCMx 25 */
#define GPIO_ENCODER_R_A_IIDX                                (DL_GPIO_IIDX_DIO8)
#define GPIO_ENCODER_R_A_PIN                                     (DL_GPIO_PIN_8)
#define GPIO_ENCODER_R_A_IOMUX                                   (IOMUX_PINCM25)
/* Defines for R_B: GPIOB.0 with pinCMx 12 */
#define GPIO_ENCODER_R_B_PIN                                     (DL_GPIO_PIN_0)
#define GPIO_ENCODER_R_B_IOMUX                                   (IOMUX_PINCM12)
/* Port definition for Pin Group GPIO_GRAYSCALE */
#define GPIO_GRAYSCALE_PORT                                              (GPIOB)

/* Defines for AD2: GPIOB.23 with pinCMx 51 on package pin 22 */
#define GPIO_GRAYSCALE_AD2_PIN                                  (DL_GPIO_PIN_23)
#define GPIO_GRAYSCALE_AD2_IOMUX                                 (IOMUX_PINCM51)
/* Defines for AD1: GPIOB.10 with pinCMx 27 on package pin 62 */
#define GPIO_GRAYSCALE_AD1_PIN                                  (DL_GPIO_PIN_10)
#define GPIO_GRAYSCALE_AD1_IOMUX                                 (IOMUX_PINCM27)
/* Defines for AD0: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_GRAYSCALE_AD0_PIN                                  (DL_GPIO_PIN_13)
#define GPIO_GRAYSCALE_AD0_IOMUX                                 (IOMUX_PINCM30)
/* Defines for OUT: GPIOB.1 with pinCMx 13 on package pin 48 */
#define GPIO_GRAYSCALE_OUT_PIN                                   (DL_GPIO_PIN_1)
#define GPIO_GRAYSCALE_OUT_IOMUX                                 (IOMUX_PINCM13)
/* Defines for KEY1: GPIOB.22 with pinCMx 50 on package pin 21 */
#define GPIO_KEYS_KEY1_PORT                                              (GPIOB)
#define GPIO_KEYS_KEY1_PIN                                      (DL_GPIO_PIN_22)
#define GPIO_KEYS_KEY1_IOMUX                                     (IOMUX_PINCM50)
/* Defines for KEY2: GPIOA.30 with pinCMx 5 on package pin 37 */
#define GPIO_KEYS_KEY2_PORT                                              (GPIOA)
#define GPIO_KEYS_KEY2_PIN                                      (DL_GPIO_PIN_30)
#define GPIO_KEYS_KEY2_IOMUX                                      (IOMUX_PINCM5)
/* Defines for KEY3: GPIOB.27 with pinCMx 58 on package pin 29 */
#define GPIO_KEYS_KEY3_PORT                                              (GPIOB)
#define GPIO_KEYS_KEY3_PIN                                      (DL_GPIO_PIN_27)
#define GPIO_KEYS_KEY3_IOMUX                                     (IOMUX_PINCM58)
/* Defines for KEY4: GPIOB.26 with pinCMx 57 on package pin 28 */
#define GPIO_KEYS_KEY4_PORT                                              (GPIOB)
#define GPIO_KEYS_KEY4_PIN                                      (DL_GPIO_PIN_26)
#define GPIO_KEYS_KEY4_IOMUX                                     (IOMUX_PINCM57)
/* Defines for LED_USER: GPIOB.4 with pinCMx 17 on package pin 52 */
#define GPIO_BOARD_IO_LED_USER_PORT                                      (GPIOB)
#define GPIO_BOARD_IO_LED_USER_PIN                               (DL_GPIO_PIN_4)
#define GPIO_BOARD_IO_LED_USER_IOMUX                             (IOMUX_PINCM17)
/* Defines for BEEP: GPIOA.7 with pinCMx 14 on package pin 49 */
#define GPIO_BOARD_IO_BEEP_PORT                                          (GPIOA)
#define GPIO_BOARD_IO_BEEP_PIN                                   (DL_GPIO_PIN_7)
#define GPIO_BOARD_IO_BEEP_IOMUX                                 (IOMUX_PINCM14)
/* Defines for NRF_CE: GPIOB.20 with pinCMx 48 on package pin 19 */
#define GPIO_NRF_NRF_CE_PORT                                             (GPIOB)
#define GPIO_NRF_NRF_CE_PIN                                     (DL_GPIO_PIN_20)
#define GPIO_NRF_NRF_CE_IOMUX                                    (IOMUX_PINCM48)
/* Defines for NRF_CSN: GPIOA.8 with pinCMx 19 on package pin 54 */
#define GPIO_NRF_NRF_CSN_PORT                                            (GPIOA)
#define GPIO_NRF_NRF_CSN_PIN                                     (DL_GPIO_PIN_8)
#define GPIO_NRF_NRF_CSN_IOMUX                                   (IOMUX_PINCM19)
/* Defines for NRF_SCK: GPIOA.11 with pinCMx 22 on package pin 57 */
#define GPIO_NRF_NRF_SCK_PORT                                            (GPIOA)
#define GPIO_NRF_NRF_SCK_PIN                                    (DL_GPIO_PIN_11)
#define GPIO_NRF_NRF_SCK_IOMUX                                   (IOMUX_PINCM22)
/* Defines for NRF_MOSI: GPIOA.9 with pinCMx 20 on package pin 55 */
#define GPIO_NRF_NRF_MOSI_PORT                                           (GPIOA)
#define GPIO_NRF_NRF_MOSI_PIN                                    (DL_GPIO_PIN_9)
#define GPIO_NRF_NRF_MOSI_IOMUX                                  (IOMUX_PINCM20)
/* Defines for NRF_MISO: GPIOA.10 with pinCMx 21 on package pin 56 */
#define GPIO_NRF_NRF_MISO_PORT                                           (GPIOA)
#define GPIO_NRF_NRF_MISO_PIN                                   (DL_GPIO_PIN_10)
#define GPIO_NRF_NRF_MISO_IOMUX                                  (IOMUX_PINCM21)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_init(void);
void SYSCFG_DL_PWM_SERVO_init(void);
void SYSCFG_DL_TIMER_SYS_init(void);
void SYSCFG_DL_I2C_SHARED_init(void);
void SYSCFG_DL_UART_DEBUG_init(void);
void SYSCFG_DL_UART_K230_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
