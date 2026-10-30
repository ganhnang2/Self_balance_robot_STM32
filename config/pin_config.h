/**
 * @file    pin_config.h
 * @author  Nguyen Hai Nam
 * @brief   Hardware Pin Mapping and GPIO Initialization for Self-Balancing Robot.
 *          Centralizes all pin definitions and GPIO configurations for sensors,
 *          actuators, communications, and peripherals on STM32F401CCU6.
 */

#ifndef PIN_CONFIG_H_
#define PIN_CONFIG_H_

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

/* =========================================================================
 * 1. HC-05 BLUETOOTH MODULE (USART2)
 * ========================================================================= */
#define PIN_HC05_KEY_PORT           GPIOA
#define PIN_HC05_KEY_PIN            GPIO_PIN_1      /*!< KEY Pin: High = AT Mode, Low = Data Mode */

#define PIN_HC05_TX_PORT            GPIOA
#define PIN_HC05_TX_PIN             GPIO_PIN_2      /*!< USART2_TX (AF7) -> HC-05 RXD */

#define PIN_HC05_RX_PORT            GPIOA
#define PIN_HC05_RX_PIN             GPIO_PIN_3      /*!< USART2_RX (AF7) -> HC-05 TXD */

/* =========================================================================
 * 2. MPU-6050 6-DOF IMU SENSOR (I2C1)
 * ========================================================================= */
#define PIN_MPU6050_SCL_PORT        GPIOB
#define PIN_MPU6050_SCL_PIN         GPIO_PIN_8      /*!< I2C1_SCL (AF4, Open-Drain) */

#define PIN_MPU6050_SDA_PORT        GPIOB
#define PIN_MPU6050_SDA_PIN         GPIO_PIN_9      /*!< I2C1_SDA (AF4, Open-Drain) */

/* =========================================================================
 * 3. SSD1306 OLED DISPLAY 128x64 (I2C2)
 * ========================================================================= */
#define PIN_OLED_SCL_PORT           GPIOB
#define PIN_OLED_SCL_PIN            GPIO_PIN_10     /*!< I2C2_SCL (AF4, Open-Drain) */

#define PIN_OLED_SDA_PORT           GPIOB
#define PIN_OLED_SDA_PIN            GPIO_PIN_3      /*!< I2C2_SDA (AF9 on STM32F401, Open-Drain) */

/* =========================================================================
 * 4. TB6612FNG MOTOR DRIVER (Direction & PWM TIM3)
 * ========================================================================= */
#define PIN_MOTOR_STBY_PORT         GPIOA
#define PIN_MOTOR_STBY_PIN          GPIO_PIN_12     /*!< Standby: High = Enable, Low = Standby */

/* Right Motor Direction (Motor A) */
#define PIN_MOTOR_AIN1_PORT         GPIOB
#define PIN_MOTOR_AIN1_PIN          GPIO_PIN_13     /*!< Right Motor Direction 1 */

#define PIN_MOTOR_AIN2_PORT         GPIOB
#define PIN_MOTOR_AIN2_PIN          GPIO_PIN_12     /*!< Right Motor Direction 2 */

/* Right Motor Direction (Motor B) */
#define PIN_MOTOR_BIN1_PORT         GPIOB
#define PIN_MOTOR_BIN1_PIN          GPIO_PIN_14     /*!< Left Motor Direction 1 */

#define PIN_MOTOR_BIN2_PORT         GPIOB
#define PIN_MOTOR_BIN2_PIN          GPIO_PIN_15     /*!< Left Motor Direction 2 */

/* PWM Outputs (TIM3) */
#define PIN_MOTOR_PWMA_PORT         GPIOB
#define PIN_MOTOR_PWMA_PIN          GPIO_PIN_4      /*!< TIM3_CH1 (AF2) -> Right Motor PWM */

#define PIN_MOTOR_PWMB_PORT         GPIOB
#define PIN_MOTOR_PWMB_PIN          GPIO_PIN_5      /*!< TIM3_CH2 (AF2) -> Left Motor PWM */

/* =========================================================================
 * 5. WHEEL ENCODERS (Quadrature Encoder TIM1 & TIM4)
 * ========================================================================= */
/* Left Encoder -> TIM1 (PA8 / PA9) */
#define PIN_ENC_LEFT_A_PORT         GPIOA
#define PIN_ENC_LEFT_A_PIN          GPIO_PIN_8      /*!< TIM1_CH1 (AF1) -> Left Encoder Phase A */

#define PIN_ENC_LEFT_B_PORT         GPIOA
#define PIN_ENC_LEFT_B_PIN          GPIO_PIN_9      /*!< TIM1_CH2 (AF1) -> Left Encoder Phase B */

/* Right Encoder -> TIM4 (PB6 / PB7) */
#define PIN_ENC_RIGHT_B_PORT        GPIOB
#define PIN_ENC_RIGHT_B_PIN         GPIO_PIN_6      /*!< TIM4_CH1 (AF2) -> Right Encoder Phase B */

#define PIN_ENC_RIGHT_A_PORT        GPIOB
#define PIN_ENC_RIGHT_A_PIN         GPIO_PIN_7      /*!< TIM4_CH2 (AF2) -> Right Encoder Phase A */

/* =========================================================================
 * 6. ON-BOARD BUILT-IN LED (PC13)
 * ========================================================================= */
#define PIN_LED_PORT                GPIOC
#define PIN_LED_PIN                 GPIO_PIN_13     /*!< Built-in LED: PC13 (Active Low on STM32F4 Black Pill) */

/* Aliases for flexibility */
#define PIN_LED_BUILTIN_PORT        PIN_LED_PORT
#define PIN_LED_BUILTIN_PIN         PIN_LED_PIN

/* Control Macros for Built-in LED */
#define LED_BUILTIN_ON()            GPIO_Write_Pin(PIN_LED_PORT, PIN_LED_PIN, GPIO_PIN_RESET)
#define LED_BUILTIN_OFF()           GPIO_Write_Pin(PIN_LED_PORT, PIN_LED_PIN, GPIO_PIN_SET)
#define LED_BUILTIN_TOGGLE()        GPIO_Toggle_Pin(PIN_LED_PORT, PIN_LED_PIN)

/* =========================================================================
 * API Function Prototypes
 * ========================================================================= */

/**
 * @brief  Configure GPIO pin for Built-in LED (PC13: Output Push-Pull, default OFF).
 */
void Pin_Config_LED(void);

/**
 * @brief  Configure GPIO pins for HC-05 module (PA1: KEY, PA2: TX, PA3: RX).
 */
void Pin_Config_HC05(void);

/**
 * @brief  Configure GPIO pins for MPU-6050 (PB8: SCL, PB9: SDA, Open-Drain).
 */
void Pin_Config_MPU6050(void);

/**
 * @brief  Configure GPIO pins for SSD1306 OLED (PB10: SCL, PB3: SDA, Open-Drain).
 */
void Pin_Config_OLED(void);

/**
 * @brief  Configure GPIO pins for TB6612 motor driver (PA12: STBY, PB12-15: DIR, PB4-5: PWM).
 */
void Pin_Config_Motor(void);

/**
 * @brief  Configure GPIO pins for Quadrature Encoders (PA8-9: Left TIM1, PB6-7: Right TIM4).
 */
void Pin_Config_Encoder(void);

/**
 * @brief  Initialize all GPIO pins for the complete Self-Balancing Robot system.
 */
void Pin_Config_All(void);

#endif /* PIN_CONFIG_H_ */
