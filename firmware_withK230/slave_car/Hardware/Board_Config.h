#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#include "ti_msp_dl_config.h"

/*
 * Board pin map for the MSPM0G3507 slave car (stepper + D36A).
 *
 * Keep application code using these names.  SysConfig-generated names stay
 * below this file and can be changed without touching App/Control logic.
 */

#ifndef ECAR_REAR_LINE_SENSOR_MODE
#define ECAR_REAR_LINE_SENSOR_MODE 1U
#endif

/* Compatibility aliases for SysConfig-generated grouped GPIO names. */
#if !defined(GPIO_I2C0_SCL_PORT) && defined(GPIO_I2C_SHARED_SCL_PORT)
#define GPIO_I2C0_SCL_PORT              GPIO_I2C_SHARED_SCL_PORT
#define GPIO_I2C0_SCL_PIN               GPIO_I2C_SHARED_SCL_PIN
#define GPIO_I2C0_SDA_PORT              GPIO_I2C_SHARED_SDA_PORT
#define GPIO_I2C0_SDA_PIN               GPIO_I2C_SHARED_SDA_PIN
#endif

#if !defined(GPIO_GRAYSCALE_AD0_PORT) && defined(GPIO_GRAYSCALE_PORT)
#define GPIO_GRAYSCALE_AD0_PORT         GPIO_GRAYSCALE_PORT
#define GPIO_GRAYSCALE_AD1_PORT         GPIO_GRAYSCALE_PORT
#define GPIO_GRAYSCALE_AD2_PORT         GPIO_GRAYSCALE_PORT
#define GPIO_GRAYSCALE_OUT_PORT         GPIO_GRAYSCALE_PORT
#endif

#if !defined(GPIO_KEYS_KEY1_PORT) && defined(GPIO_KEYS_PORT)
#define GPIO_KEYS_KEY1_PORT             GPIO_KEYS_PORT
#define GPIO_KEYS_KEY2_PORT             GPIO_KEYS_PORT
#define GPIO_KEYS_KEY3_PORT             GPIO_KEYS_PORT
#define GPIO_KEYS_KEY4_PORT             GPIO_KEYS_PORT
#endif

#if !defined(GPIO_LED_USER_PORT) && defined(GPIO_BOARD_IO_LED_USER_PORT)
#define GPIO_LED_USER_PORT              GPIO_BOARD_IO_LED_USER_PORT
#define GPIO_LED_USER_PIN               GPIO_BOARD_IO_LED_USER_PIN
#define GPIO_LED_USER_IOMUX             GPIO_BOARD_IO_LED_USER_IOMUX
#endif

#if !defined(GPIO_BEEP_PORT) && defined(GPIO_BOARD_IO_BEEP_PORT)
#define GPIO_BEEP_PORT                  GPIO_BOARD_IO_BEEP_PORT
#define GPIO_BEEP_PIN                   GPIO_BOARD_IO_BEEP_PIN
#define GPIO_BEEP_IOMUX                 GPIO_BOARD_IO_BEEP_IOMUX
#endif

/* ---------------- TB6612 motor driver (legacy, not used in stepper config) ---- */
#define MOTOR_USE_STBY                  0U
#define MOTOR_STBY_TIED_TO_5V           1U
#define MOTOR_PWM_TIMER_INST            PWM_MOTOR_INST

#if ECAR_REAR_LINE_SENSOR_MODE
#define MOTOR_L_PWM_CC_INDEX            GPIO_PWM_MOTOR_C1_IDX
#define MOTOR_R_PWM_CC_INDEX            GPIO_PWM_MOTOR_C0_IDX
#define MOTOR_L_IN1_PORT                GPIO_MOTOR_R_IN1_PORT
#define MOTOR_L_IN1_PIN                 GPIO_MOTOR_R_IN1_PIN
#define MOTOR_L_IN1                     MOTOR_L_IN1_PIN
#define MOTOR_L_IN2_PORT                GPIO_MOTOR_R_IN2_PORT
#define MOTOR_L_IN2_PIN                 GPIO_MOTOR_R_IN2_PIN
#define MOTOR_L_IN2                     MOTOR_L_IN2_PIN
#define MOTOR_R_IN1_PORT                GPIO_MOTOR_L_IN1_PORT
#define MOTOR_R_IN1_PIN                 GPIO_MOTOR_L_IN1_PIN
#define MOTOR_R_IN1                     MOTOR_R_IN1_PIN
#define MOTOR_R_IN2_PORT                GPIO_MOTOR_L_IN2_PORT
#define MOTOR_R_IN2_PIN                 GPIO_MOTOR_L_IN2_PIN
#define MOTOR_R_IN2                     MOTOR_R_IN2_PIN
#define LEFT_MOTOR_DIR                  (+1)
#define RIGHT_MOTOR_DIR                 (-1)
#else
#define MOTOR_L_PWM_CC_INDEX            GPIO_PWM_MOTOR_C0_IDX
#define MOTOR_R_PWM_CC_INDEX            GPIO_PWM_MOTOR_C1_IDX
#define MOTOR_L_IN1_PORT                GPIO_MOTOR_L_IN1_PORT
#define MOTOR_L_IN1_PIN                 GPIO_MOTOR_L_IN1_PIN
#define MOTOR_L_IN1                     MOTOR_L_IN1_PIN
#define MOTOR_L_IN2_PORT                GPIO_MOTOR_L_IN2_PORT
#define MOTOR_L_IN2_PIN                 GPIO_MOTOR_L_IN2_PIN
#define MOTOR_L_IN2                     MOTOR_L_IN2_PIN
#define MOTOR_R_IN1_PORT                GPIO_MOTOR_R_IN1_PORT
#define MOTOR_R_IN1_PIN                 GPIO_MOTOR_R_IN1_PIN
#define MOTOR_R_IN1                     MOTOR_R_IN1_PIN
#define MOTOR_R_IN2_PORT                GPIO_MOTOR_R_IN2_PORT
#define MOTOR_R_IN2_PIN                 GPIO_MOTOR_R_IN2_PIN
#define MOTOR_R_IN2                     MOTOR_R_IN2_PIN
#define LEFT_MOTOR_DIR                  (+1)
#define RIGHT_MOTOR_DIR                 (-1)
#endif

#define MOTOR_L_PWM                     MOTOR_L_PWM_CC_INDEX
#define MOTOR_R_PWM                     MOTOR_R_PWM_CC_INDEX
#define MOTOR_PWM_LEFT_CC_INDEX         MOTOR_L_PWM_CC_INDEX
#define MOTOR_PWM_RIGHT_CC_INDEX        MOTOR_R_PWM_CC_INDEX
#define MOTOR_PWM_PERIOD_COUNTS         1600U
#define MOTOR_AIN1_PORT                 MOTOR_L_IN1_PORT
#define MOTOR_AIN1_PIN                  MOTOR_L_IN1_PIN
#define MOTOR_AIN2_PORT                 MOTOR_L_IN2_PORT
#define MOTOR_AIN2_PIN                  MOTOR_L_IN2_PIN
#define MOTOR_BIN1_PORT                 MOTOR_R_IN1_PORT
#define MOTOR_BIN1_PIN                  MOTOR_R_IN1_PIN
#define MOTOR_BIN2_PORT                 MOTOR_R_IN2_PORT
#define MOTOR_BIN2_PIN                  MOTOR_R_IN2_PIN
#define LEFT_MOTOR_DIR_SIGN             LEFT_MOTOR_DIR
#define RIGHT_MOTOR_DIR_SIGN            RIGHT_MOTOR_DIR

/* ===================================================================
 * D36A dual stepper driver — NO EN GPIO (hardware always enabled)
 * ===================================================================
 *
 * STEP_L = PB15 / TIMG7 CC0
 * STEP_R = PB16 / TIMG8 CC1
 * DIR_L  = PB18 / GPIO output
 * DIR_R  = PB25 / GPIO output
 *
 * EN1/EN2 are hardware-tied active.  No MCU GPIO.
 */
#define STEPPER_HAS_ENABLE_GPIO 0U

#define STEPPER_STEP_L_PORT             GPIOB
#define STEPPER_STEP_L_PIN              DL_GPIO_PIN_15
#define STEPPER_STEP_L_IOMUX            (IOMUX_PINCM32)
#define STEPPER_STEP_L_IOMUX_FUNC       IOMUX_PINCM32_PF_TIMG7_CCP0
#define STEPPER_STEP_L_TIMER_INST       TIMG7
#define STEPPER_STEP_L_CC_INDEX         DL_TIMER_CC_0_INDEX

#define STEPPER_STEP_R_PORT             GPIOB
#define STEPPER_STEP_R_PIN              DL_GPIO_PIN_16
#define STEPPER_STEP_R_IOMUX            (IOMUX_PINCM33)
#define STEPPER_STEP_R_IOMUX_FUNC       IOMUX_PINCM33_PF_TIMG8_CCP1
#define STEPPER_STEP_R_TIMER_INST       TIMG8
#define STEPPER_STEP_R_CC_INDEX         DL_TIMER_CC_1_INDEX

#define STEPPER_DIR_L_PORT              GPIOB
#define STEPPER_DIR_L_PIN               DL_GPIO_PIN_18
#define STEPPER_DIR_L_IOMUX             (IOMUX_PINCM44)

#define STEPPER_DIR_R_PORT              GPIOB
#define STEPPER_DIR_R_PIN               DL_GPIO_PIN_25
#define STEPPER_DIR_R_IOMUX             (IOMUX_PINCM56)

/*
 * Direction sign correction.
 * Positive logical frequency must drive the car forward.
 *
 * Measured 2026-07: D36A channels are cross-wired to motors.
 *   STEP_L/DIR_L (PB15/PB18) → physically RIGHT motor
 *   STEP_R/DIR_R (PB16/PB25) → physically LEFT motor
 * Both motors require DIR=LOW for forward rotation.
 */
#define LEFT_STEPPER_DIR_SIGN           (-1)
#define RIGHT_STEPPER_DIR_SIGN          (-1)

/* ===================================================================
 * Encoders — 4× quadrature, all on GPIOB
 * ===================================================================
 *
 * ENC_L_A = PB05, ENC_L_B = PB12
 * ENC_R_A = PB08, ENC_R_B = PB00
 *
 * All four share GPIOB interrupt (GPIOB_INT_IRQn).
 * A/B both configured RISE_FALL for 4× quadrature decoding.
 */

#define ENC_L_A_PORT                    GPIOB
#define ENC_L_A_PIN                     DL_GPIO_PIN_5
#define ENC_L_A_IOMUX                   (IOMUX_PINCM18)

#define ENC_L_B_PORT                    GPIOB
#define ENC_L_B_PIN                     DL_GPIO_PIN_12
#define ENC_L_B_IOMUX                   (IOMUX_PINCM29)

#define ENC_R_A_PORT                    GPIOB
#define ENC_R_A_PIN                     DL_GPIO_PIN_8
#define ENC_R_A_IOMUX                   (IOMUX_PINCM25)

#define ENC_R_B_PORT                    GPIOB
#define ENC_R_B_PIN                     DL_GPIO_PIN_0
#define ENC_R_B_IOMUX                   (IOMUX_PINCM12)

#define ENCODER_GPIO_PORT               GPIOB
#define ENCODER_GPIO_IRQN               GPIOB_INT_IRQn
#define ENCODER_GPIO_INT_IIDX           DL_INTERRUPT_GROUP1_IIDX_GPIOB

/* Legacy aliases for older code. */
#define ENC_L_A                         ENC_L_A_PIN
#define ENC_L_B                         ENC_L_B_PIN
#define ENC_R_A                         ENC_R_A_PIN
#define ENC_R_B                         ENC_R_B_PIN

#define ENCODER_LEFT_A_PORT             ENC_L_A_PORT
#define ENCODER_LEFT_A_PIN              ENC_L_A_PIN
#define ENCODER_LEFT_B_PORT             ENC_L_B_PORT
#define ENCODER_LEFT_B_PIN              ENC_L_B_PIN
#define ENCODER_RIGHT_A_PORT            ENC_R_A_PORT
#define ENCODER_RIGHT_A_PIN             ENC_R_A_PIN
#define ENCODER_RIGHT_B_PORT            ENC_R_B_PORT
#define ENCODER_RIGHT_B_PIN             ENC_R_B_PIN

/*
 * Encoder direction sign:
 * When wheel turns forward, delta must be positive.
 * Verify by pushing car forward after wiring.
 */
#define LEFT_ENCODER_DIR_SIGN           (+1)
#define RIGHT_ENCODER_DIR_SIGN          (+1)

/* Backward compat aliases for Encoder.c internal use. */
#define LEFT_ENCODER_DIR                LEFT_ENCODER_DIR_SIGN
#define RIGHT_ENCODER_DIR               RIGHT_ENCODER_DIR_SIGN
#define LEFT_ENCODER_SIGN               LEFT_ENCODER_DIR_SIGN
#define RIGHT_ENCODER_SIGN              RIGHT_ENCODER_DIR_SIGN

/* ---------------- 8-channel grayscale module ---------------- */
#define GRAY_AD0_PORT                   GPIO_GRAYSCALE_AD0_PORT
#define GRAY_AD0_PIN                    GPIO_GRAYSCALE_AD0_PIN
#define GRAY_AD0                        GRAY_AD0_PIN
#define GRAY_AD1_PORT                   GPIO_GRAYSCALE_AD1_PORT
#define GRAY_AD1_PIN                    GPIO_GRAYSCALE_AD1_PIN
#define GRAY_AD1                        GRAY_AD1_PIN
#define GRAY_AD2_PORT                   GPIO_GRAYSCALE_AD2_PORT
#define GRAY_AD2_PIN                    GPIO_GRAYSCALE_AD2_PIN
#define GRAY_AD2                        GRAY_AD2_PIN
#define GRAY_OUT_PORT                   GPIO_GRAYSCALE_OUT_PORT
#define GRAY_OUT_PIN                    GPIO_GRAYSCALE_OUT_PIN
#define GRAY_OUT                        GRAY_OUT_PIN

#define GRAYSCALE_AD0_PORT              GRAY_AD0_PORT
#define GRAYSCALE_AD0_PIN               GRAY_AD0_PIN
#define GRAYSCALE_AD1_PORT              GRAY_AD1_PORT
#define GRAYSCALE_AD1_PIN               GRAY_AD1_PIN
#define GRAYSCALE_AD2_PORT              GRAY_AD2_PORT
#define GRAYSCALE_AD2_PIN               GRAY_AD2_PIN
#define GRAYSCALE_OUT_PORT              GRAY_OUT_PORT
#define GRAYSCALE_OUT_PIN               GRAY_OUT_PIN

/* ---------------- I2C0 shared bus ---------------- */
#define I2C0_SCL_PORT                   GPIO_I2C0_SCL_PORT
#define I2C0_SCL_PIN                    GPIO_I2C0_SCL_PIN
#define I2C0_SCL                        I2C0_SCL_PIN
#define I2C0_SDA_PORT                   GPIO_I2C0_SDA_PORT
#define I2C0_SDA_PIN                    GPIO_I2C0_SDA_PIN
#define I2C0_SDA                        I2C0_SDA_PIN
#define BOARD_I2C0_INST                 I2C_SHARED_INST
#define BOARD_I2C0_BUS_SPEED_HZ         I2C_SHARED_BUS_SPEED_HZ

#define OLED_I2C_INST                   BOARD_I2C0_INST
#define OLED_I2C_BUS_SPEED_HZ           BOARD_I2C0_BUS_SPEED_HZ
#define OLED_I2C_SCL_PORT               I2C0_SCL_PORT
#define OLED_I2C_SCL_PIN                I2C0_SCL_PIN
#define OLED_I2C_SDA_PORT               I2C0_SDA_PORT
#define OLED_I2C_SDA_PIN                I2C0_SDA_PIN
#define MPU6050_I2C_INST                BOARD_I2C0_INST

/* ---------------- Tianmengxing H8 OLED header ----------------
 * H8-1 GND, H8-2 3V3, H8-3 SCL/PB9, H8-4 SDA/PB8,
 * H8-5 RES/PB10, H8-6 DC/PB11, H8-7 CS/PB14.
 *
 * IMPORTANT: In stepper-test mode, PB08 is used as ENC_R_A.
 * Set BOARD_OLED_USE_H8_I2C=0 and ECAR_OLED_ENABLE=0
 * when using the onboard encoder pins.  PB08 must not be
 * shared between OLED SDA and encoder input.
 */
#ifndef BOARD_OLED_USE_H8_I2C
#define BOARD_OLED_USE_H8_I2C           0U
#endif
#ifndef BOARD_OLED_USE_H8_SPI
#define BOARD_OLED_USE_H8_SPI           0U
#endif
#if BOARD_OLED_USE_H8_I2C && BOARD_OLED_USE_H8_SPI
#error "Choose either H8 I2C OLED or H8 SPI OLED, not both."
#endif
#define BOARD_OLED_H8_SPI_OWNS_GRAY_AD1 BOARD_OLED_USE_H8_SPI
#define BOARD_OLED_H8_SPI_OWNS_KEY12    BOARD_OLED_USE_H8_SPI

#define OLED_H8_PORT                    GPIOB
#define OLED_H8_SCL_PIN                 DL_GPIO_PIN_9
#define OLED_H8_SCL_IOMUX               IOMUX_PINCM26
#define OLED_H8_SDA_PIN                 DL_GPIO_PIN_8
#define OLED_H8_SDA_IOMUX               IOMUX_PINCM25
#define OLED_H8_RES_PIN                 DL_GPIO_PIN_10
#define OLED_H8_RES_IOMUX               IOMUX_PINCM27
#define OLED_H8_DC_PIN                  DL_GPIO_PIN_11
#define OLED_H8_DC_IOMUX                IOMUX_PINCM28
#define OLED_H8_CS_PIN                  DL_GPIO_PIN_14
#define OLED_H8_CS_IOMUX                IOMUX_PINCM31
#define OLED_H8_PIN_MASK                (OLED_H8_SCL_PIN | OLED_H8_SDA_PIN | \
                                         OLED_H8_RES_PIN | OLED_H8_DC_PIN | \
                                         OLED_H8_CS_PIN)

/* ---------------- Beeper and user LED ---------------- */
#define BEEP_PORT                       GPIO_BEEP_PORT
#define BEEP_PIN                        GPIO_BEEP_PIN
#define BEEP                            BEEP_PIN
#define LED_USER_PORT                   GPIO_LED_USER_PORT
#define LED_USER_PIN                    GPIO_LED_USER_PIN
#define LED_USER                        LED_USER_PIN

/* ---------------- Keys, active low with internal pull-up ---------------- */
#define KEY1_PORT                       GPIO_KEYS_KEY1_PORT
#define KEY1_PIN                        GPIO_KEYS_KEY1_PIN
#define KEY1                            KEY1_PIN
#define KEY2_PORT                       GPIO_KEYS_KEY2_PORT
#define KEY2_PIN                        GPIO_KEYS_KEY2_PIN
#define KEY2                            KEY2_PIN
#define KEY3_PORT                       GPIO_KEYS_KEY3_PORT
#define KEY3_PIN                        GPIO_KEYS_KEY3_PIN
#define KEY3                            KEY3_PIN
#define KEY4_PORT                       GPIO_KEYS_KEY4_PORT
#define KEY4_PIN                        GPIO_KEYS_KEY4_PIN
#define KEY4                            KEY4_PIN

#define KEY_K1_PORT                     KEY1_PORT
#define KEY_K1_PIN                      KEY1_PIN
#define KEY_K2_PORT                     KEY2_PORT
#define KEY_K2_PIN                      KEY2_PIN
#define KEY_K3_PORT                     KEY3_PORT
#define KEY_K3_PIN                      KEY3_PIN
#define KEY_K4_PORT                     KEY4_PORT
#define KEY_K4_PIN                      KEY4_PIN

/* ---------------- Servo PWM ---------------- */
#define SERVO_PWM_TIMER_INST            PWM_SERVO_INST
#define SERVO_PWM_PERIOD_US             20000U
#define SERVO_MIN_PULSE_US              500U
#define SERVO_MID_PULSE_US              1500U
#define SERVO_MAX_PULSE_US              2500U
#define SERVO1_PWM_CC_INDEX             DL_TIMER_CC_0_INDEX
#define SERVO2_PWM_CC_INDEX             DL_TIMER_CC_1_INDEX
#define SERVO3_PWM_CC_INDEX             DL_TIMER_CC_2_INDEX
#define SERVO4_PWM_CC_INDEX             DL_TIMER_CC_3_INDEX
#define SERVO1_PWM                      SERVO1_PWM_CC_INDEX
#define SERVO2_PWM                      SERVO2_PWM_CC_INDEX
#define SERVO3_PWM                      SERVO3_PWM_CC_INDEX
#define SERVO4_PWM                      SERVO4_PWM_CC_INDEX

/* ---------------- UARTs ---------------- */
#define SERIAL_UART_INST                UART_K230_INST
#define SERIAL_UART_IRQN                UART_K230_INST_INT_IRQN
#define SERIAL_BAUD_RATE                UART_K230_BAUD_RATE

/* ---------------- System tick ---------------- */
#define SYSTEM_TIMER_INST               TIMER_SYS_INST
#define SYSTEM_TIMER_IRQN               TIMER_SYS_INST_INT_IRQN

#endif
