#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#include "ti_msp_dl_config.h"

/*
 * Board pin map for the MSPM0G3507 car wiring.
 *
 * Keep application code using these names.  SysConfig-generated names stay
 * below this file and can be changed without touching App/Control logic.
 */

/*
 * ECAR_REAR_LINE_SENSOR_MODE:
 *   0 = original front sensor module layout;
 *   1 = rear sensor module layout, new heading is reversed.
 *       Original right wheel becomes logical left wheel,
 *       original left wheel becomes logical right wheel.
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

#if !defined(GPIO_ENCODER_L_A_PORT) && defined(GPIO_ENCODER_PORT)
#define GPIO_ENCODER_L_A_PORT           GPIO_ENCODER_PORT
#define GPIO_ENCODER_L_B_PORT           GPIO_ENCODER_PORT
#define GPIO_ENCODER_R_A_PORT           GPIO_ENCODER_PORT
#define GPIO_ENCODER_R_B_PORT           GPIO_ENCODER_PORT
#endif

#if !defined(GPIO_ENCODER_INT_IRQN) && defined(GPIO_ENCODER_GPIOA_INT_IRQN)
#define GPIO_ENCODER_INT_IRQN           GPIO_ENCODER_GPIOA_INT_IRQN
#elif !defined(GPIO_ENCODER_INT_IRQN) && defined(GPIO_ENCODER_GPIOB_INT_IRQN)
#define GPIO_ENCODER_INT_IRQN           GPIO_ENCODER_GPIOB_INT_IRQN
#endif

/* ---------------- TB6612 motor driver ----------------
 * PWMA -> TIMG0-C0, PWMB -> TIMG0-C1.
 * STBY is tied to 5V on the current PCB.  There is no MCU STBY control.
 */
#define MOTOR_USE_STBY                  0U
#define MOTOR_STBY_TIED_TO_5V           1U
#define MOTOR_PWM_TIMER_INST            PWM_MOTOR_INST

#if ECAR_REAR_LINE_SENSOR_MODE

/* Rear sensor / reversed heading mode:
 * original right wheel is logical left wheel;
 * original left wheel is logical right wheel.
 */
#define MOTOR_L_PWM_CC_INDEX            GPIO_PWM_MOTOR_C1_IDX
#define MOTOR_R_PWM_CC_INDEX            GPIO_PWM_MOTOR_C0_IDX

#define MOTOR_L_IN1_PORT                GPIO_MOTOR_R_IN1_PORT   /* PA16 */
#define MOTOR_L_IN1_PIN                 GPIO_MOTOR_R_IN1_PIN
#define MOTOR_L_IN1                     MOTOR_L_IN1_PIN
#define MOTOR_L_IN2_PORT                GPIO_MOTOR_R_IN2_PORT   /* B24 */
#define MOTOR_L_IN2_PIN                 GPIO_MOTOR_R_IN2_PIN
#define MOTOR_L_IN2                     MOTOR_L_IN2_PIN
#define MOTOR_R_IN1_PORT                GPIO_MOTOR_L_IN1_PORT   /* B17 */
#define MOTOR_R_IN1_PIN                 GPIO_MOTOR_L_IN1_PIN
#define MOTOR_R_IN1                     MOTOR_R_IN1_PIN
#define MOTOR_R_IN2_PORT                GPIO_MOTOR_L_IN2_PORT   /* B19 */
#define MOTOR_R_IN2_PIN                 GPIO_MOTOR_L_IN2_PIN
#define MOTOR_R_IN2                     MOTOR_R_IN2_PIN

/*
 * Measured: positive PWM drives both wheels backward in rear mode.
 * Motor direction signs are inverted so Motor_SetPWM(+150,+150)
 * drives the car toward the new front.
 */
#define LEFT_MOTOR_DIR                  (+1)
#define RIGHT_MOTOR_DIR                 (-1)

#else

/* Original front sensor mode. */
#define MOTOR_L_PWM_CC_INDEX            GPIO_PWM_MOTOR_C0_IDX
#define MOTOR_R_PWM_CC_INDEX            GPIO_PWM_MOTOR_C1_IDX

#define MOTOR_L_IN1_PORT                GPIO_MOTOR_L_IN1_PORT   /* B17 */
#define MOTOR_L_IN1_PIN                 GPIO_MOTOR_L_IN1_PIN
#define MOTOR_L_IN1                     MOTOR_L_IN1_PIN
#define MOTOR_L_IN2_PORT                GPIO_MOTOR_L_IN2_PORT   /* B19 */
#define MOTOR_L_IN2_PIN                 GPIO_MOTOR_L_IN2_PIN
#define MOTOR_L_IN2                     MOTOR_L_IN2_PIN
#define MOTOR_R_IN1_PORT                GPIO_MOTOR_R_IN1_PORT   /* A16 */
#define MOTOR_R_IN1_PIN                 GPIO_MOTOR_R_IN1_PIN
#define MOTOR_R_IN1                     MOTOR_R_IN1_PIN
#define MOTOR_R_IN2_PORT                GPIO_MOTOR_R_IN2_PORT   /* B24 */
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

/* Legacy aliases kept for older modules during transition. */
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

/* ---------------- Encoders ----------------
 * First version uses A-phase GPIO interrupt and B-phase level sampling.
 * ENC_L_A -> PA26, ENC_L_B -> PA27
 * ENC_R_A -> PA25, ENC_R_B -> PA14
 * A phases are on GPIOA, so GROUP1 GPIOA dispatch handles both counters.
 */
#if ECAR_REAR_LINE_SENSOR_MODE

/* Rear sensor / reversed heading mode:
 * original right encoder is logical left encoder;
 * original left encoder is logical right encoder.
 */
#define ENC_L_A_PORT                    GPIO_ENCODER_R_A_PORT
#define ENC_L_A_PIN                     GPIO_ENCODER_R_A_PIN
#define ENC_L_B_PORT                    GPIO_ENCODER_R_B_PORT
#define ENC_L_B_PIN                     GPIO_ENCODER_R_B_PIN
#define ENC_R_A_PORT                    GPIO_ENCODER_L_A_PORT
#define ENC_R_A_PIN                     GPIO_ENCODER_L_A_PIN
#define ENC_R_B_PORT                    GPIO_ENCODER_L_B_PORT
#define ENC_R_B_PIN                     GPIO_ENCODER_L_B_PIN

/*
 * Verify by pushing car toward new front:
 * g_leftEncoderDelta > 0, g_rightEncoderDelta > 0,
 * g_forwardEncoderTotal increases.
 */
#define LEFT_ENCODER_DIR                (-1)
#define RIGHT_ENCODER_DIR               (+1)

#else

/* Original front sensor mode. */
#define ENC_L_A_PORT                    GPIO_ENCODER_L_A_PORT
#define ENC_L_A_PIN                     GPIO_ENCODER_L_A_PIN
#define ENC_L_B_PORT                    GPIO_ENCODER_L_B_PORT
#define ENC_L_B_PIN                     GPIO_ENCODER_L_B_PIN
#define ENC_R_A_PORT                    GPIO_ENCODER_R_A_PORT
#define ENC_R_A_PIN                     GPIO_ENCODER_R_A_PIN
#define ENC_R_B_PORT                    GPIO_ENCODER_R_B_PORT
#define ENC_R_B_PIN                     GPIO_ENCODER_R_B_PIN

#define LEFT_ENCODER_DIR                (-1)
#define RIGHT_ENCODER_DIR               (+1)

#endif

#define ENC_L_A                         ENC_L_A_PIN
#define ENC_L_B                         ENC_L_B_PIN
#define ENC_R_A                         ENC_R_A_PIN
#define ENC_R_B                         ENC_R_B_PIN

#define ENCODER_GPIO_IRQN               GPIO_ENCODER_INT_IRQN

#define ENCODER_LEFT_A_PORT             ENC_L_A_PORT
#define ENCODER_LEFT_A_PIN              ENC_L_A_PIN
#define ENCODER_LEFT_B_PORT             ENC_L_B_PORT
#define ENCODER_LEFT_B_PIN              ENC_L_B_PIN
#define ENCODER_RIGHT_A_PORT            ENC_R_A_PORT
#define ENCODER_RIGHT_A_PIN             ENC_R_A_PIN
#define ENCODER_RIGHT_B_PORT            ENC_R_B_PORT
#define ENCODER_RIGHT_B_PIN             ENC_R_B_PIN

#define LEFT_ENCODER_SIGN               LEFT_ENCODER_DIR
#define RIGHT_ENCODER_SIGN              RIGHT_ENCODER_DIR

/* ---------------- 8-channel grayscale module ----------------
 * P3-1 VCC = 5V, P3-6 GND.
 * GRAY_AD2 -> PB23, GRAY_AD1 -> PB10, GRAY_AD0 -> PB13, GRAY_OUT -> PB01.
 *
 * Hardware warning:
 * The grayscale board is powered from 5V.  GRAY_OUT must be divided or level
 * shifted to <= 3.3V before it reaches MSPM0 PB01.  Software cannot make a
 * direct 5V GPIO input safe.  AD0/AD1/AD2 are normal 3.3V MSPM0 outputs; if
 * the 5V module does not recognize 3.3V high reliably, fix it in hardware.
 */
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

/* ---------------- I2C0 shared bus ----------------
 * Use 3.3V power and pull SDA/SCL up to 3.3V.  Do not pull I2C to 5V.
 * MPU6050 I2C0-SDA -> PA28
 * MPU6050 I2C0-SCL -> PA31
 */
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
 * Tianmengxing H8 1x8 display header:
 * H8-1 GND, H8-2 3V3, H8-3 SCL/PB9, H8-4 SDA/PB8,
 * H8-5 RES/PB10, H8-6 DC/PB11, H8-7 CS/PB14.
 *
 * Current display wiring is a 4-pin IIC OLED plugged into H8 first 4 pins:
 * GND, VCC, SCL/SKC, SDA -> PB9/PB8 through software I2C.
 *
 * H8 IIC mode only uses PB9/PB8. H8 SPI mode also takes PB10/PB11/PB14,
 * so grayscale AD1 and KEY1/KEY2 cannot be used at the same time unless
 * those signals are rewired.
 */
#ifndef BOARD_OLED_USE_H8_I2C
#define BOARD_OLED_USE_H8_I2C           1U
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
#define OLED_H8_SCL_PIN                 DL_GPIO_PIN_9    /* H8-3, SCL/SCK */
#define OLED_H8_SCL_IOMUX               IOMUX_PINCM26
#define OLED_H8_SDA_PIN                 DL_GPIO_PIN_8    /* H8-4, SDA/MOSI */
#define OLED_H8_SDA_IOMUX               IOMUX_PINCM25
#define OLED_H8_RES_PIN                 DL_GPIO_PIN_10   /* H8-5 */
#define OLED_H8_RES_IOMUX               IOMUX_PINCM27
#define OLED_H8_DC_PIN                  DL_GPIO_PIN_11   /* H8-6 */
#define OLED_H8_DC_IOMUX                IOMUX_PINCM28
#define OLED_H8_CS_PIN                  DL_GPIO_PIN_14   /* H8-7 */
#define OLED_H8_CS_IOMUX                IOMUX_PINCM31
#define OLED_H8_PIN_MASK                (OLED_H8_SCL_PIN | OLED_H8_SDA_PIN | \
                                         OLED_H8_RES_PIN | OLED_H8_DC_PIN | \
                                         OLED_H8_CS_PIN)

/* ---------------- Beeper and user LED ---------------- */
#define BEEP_PORT                       GPIO_BEEP_PORT       /* A07 */
#define BEEP_PIN                        GPIO_BEEP_PIN
#define BEEP                            BEEP_PIN
#define LED_USER_PORT                   GPIO_LED_USER_PORT   /* B04 */
#define LED_USER_PIN                    GPIO_LED_USER_PIN
#define LED_USER                        LED_USER_PIN

/* If LED2 uses a 10k series resistor, visible brightness may be low. */

/* ---------------- Keys, active low with internal pull-up ---------------- */
#define KEY1_PORT                       GPIO_KEYS_KEY1_PORT  /* B14 / SW1 */
#define KEY1_PIN                        GPIO_KEYS_KEY1_PIN
#define KEY1                            KEY1_PIN
#define KEY2_PORT                       GPIO_KEYS_KEY2_PORT  /* B11 / SW2 */
#define KEY2_PIN                        GPIO_KEYS_KEY2_PIN
#define KEY2                            KEY2_PIN
#define KEY3_PORT                       GPIO_KEYS_KEY3_PORT  /* B27 / SW3 */
#define KEY3_PIN                        GPIO_KEYS_KEY3_PIN
#define KEY3                            KEY3_PIN
#define KEY4_PORT                       GPIO_KEYS_KEY4_PORT  /* B26 / SW4 */
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

/* ---------------- Servo PWM ----------------
 * TIMA0 C0..C3, 50Hz, 20ms period, default output disabled unless test macro
 * enables a center pulse.
 * SERVO1_PWM -> PA21 / TIMA0-C0, SERVO2_PWM -> PA22 / TIMA0-C1,
 * SERVO3_PWM -> PA15 / TIMA0-C2, SERVO4_PWM -> PA17 / TIMA0-C3.
 * Use an independent high-current servo supply and common ground with the
 * MSPM0 board.
 */
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

/* ---------------- UARTs ----------------
 * UART_DEBUG uses the schematic TX1/RX1 connector:
 * UART1_TX -> PB6/TX1, UART1_RX -> PB7/RX1, 9600 8N1.
 * UART2 and UART3 are reserved.
 * Do not connect two active TX devices to the same UART RX. External modules
 * must share ground with the controller.
 */
#define SERIAL_UART_INST                UART_DEBUG_INST
#define SERIAL_UART_IRQN                UART_DEBUG_INST_INT_IRQN
#define SERIAL_BAUD_RATE                UART_DEBUG_BAUD_RATE

/* ---------------- System tick ---------------- */
#define SYSTEM_TIMER_INST               TIMER_SYS_INST
#define SYSTEM_TIMER_IRQN               TIMER_SYS_INST_INT_IRQN

/*
 * SWDIO/SWCLK are not configured as GPIO here.  Future hardware should route
 * RST to the SWD connector.
 *
 * Reserved headers:
 * H1: B05, B15, A10, B16, A11, B12.  B05 is a future TB6612_STBY candidate.
 * H3: B25, B18, B21, B22, A30, B00.
 */

#endif
