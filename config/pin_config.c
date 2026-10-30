/**
 * @file    pin_config.c
 * @author  Nguyen Hai Nam
 * @brief   Implementation of Hardware Pin Initialization for Self-Balancing Robot.
 *          Utilizes the project's GPIO Driver (stm32f4xx_gpio.h) to configure
 *          alternate functions, input/output modes, and pull-up/down resistors.
 */

#include "pin_config.h"

/* =========================================================================
 * 1. HC-05 BLUETOOTH MODULE (PA1: KEY, PA2: TX, PA3: RX)
 * ========================================================================= */
void Pin_Config_HC05(void)
{
    GPIO_Handle_t pinHandle;

    /* 1. KEY Pin (PA1) - Output Push-Pull, Low by default (Data Mode) */
    pinHandle.pGPIOx                 = PIN_HC05_KEY_PORT;
    pinHandle.GPIO_Config.pin        = PIN_HC05_KEY_PIN;
    pinHandle.GPIO_Config.mode       = GPIO_MODE_OUTPUT;
    pinHandle.GPIO_Config.outputType = GPIO_OTYPE_PP;
    pinHandle.GPIO_Config.speed      = GPIO_SPEED_HIGH;
    pinHandle.GPIO_Config.pull       = GPIO_PUPD_NONE;
    GPIO_Init(&pinHandle);
    GPIO_Write_Pin(PIN_HC05_KEY_PORT, PIN_HC05_KEY_PIN, GPIO_PIN_RESET);

    /* 2. USART2_TX (PA2) - AF7 */
    pinHandle.pGPIOx                 = PIN_HC05_TX_PORT;
    pinHandle.GPIO_Config.pin        = PIN_HC05_TX_PIN;
    pinHandle.GPIO_Config.mode       = GPIO_MODE_ALTFN;
    pinHandle.GPIO_Config.outputType = GPIO_OTYPE_PP;
    pinHandle.GPIO_Config.speed      = GPIO_SPEED_HIGH;
    pinHandle.GPIO_Config.pull       = GPIO_PUPD_PULLUP;
    pinHandle.GPIO_Config.altFn      = GPIO_AF7;
    GPIO_Init(&pinHandle);

    /* 3. USART2_RX (PA3) - AF7 */
    pinHandle.pGPIOx                 = PIN_HC05_RX_PORT;
    pinHandle.GPIO_Config.pin        = PIN_HC05_RX_PIN;
    pinHandle.GPIO_Config.mode       = GPIO_MODE_ALTFN;
    pinHandle.GPIO_Config.outputType = GPIO_OTYPE_PP;
    pinHandle.GPIO_Config.speed      = GPIO_SPEED_HIGH;
    pinHandle.GPIO_Config.pull       = GPIO_PUPD_PULLUP;
    pinHandle.GPIO_Config.altFn      = GPIO_AF7;
    GPIO_Init(&pinHandle);
}

/* =========================================================================
 * 2. MPU-6050 IMU SENSOR (PB8: I2C1_SCL, PB9: I2C1_SDA)
 * ========================================================================= */
void Pin_Config_MPU6050(void)
{
    GPIO_Handle_t pinHandle;

    /* SCL Pin (PB8) - I2C1_SCL (AF4, Open-Drain) */
    pinHandle.pGPIOx                 = PIN_MPU6050_SCL_PORT;
    pinHandle.GPIO_Config.pin        = PIN_MPU6050_SCL_PIN;
    pinHandle.GPIO_Config.mode       = GPIO_MODE_ALTFN;
    pinHandle.GPIO_Config.outputType = GPIO_OTYPE_OD;
    pinHandle.GPIO_Config.speed      = GPIO_SPEED_HIGH;
    pinHandle.GPIO_Config.pull       = GPIO_PUPD_NONE;
    pinHandle.GPIO_Config.altFn      = GPIO_AF4;
    GPIO_Init(&pinHandle);

    /* SDA Pin (PB9) - I2C1_SDA (AF4, Open-Drain) */
    pinHandle.pGPIOx                 = PIN_MPU6050_SDA_PORT;
    pinHandle.GPIO_Config.pin        = PIN_MPU6050_SDA_PIN;
    pinHandle.GPIO_Config.mode       = GPIO_MODE_ALTFN;
    pinHandle.GPIO_Config.outputType = GPIO_OTYPE_OD;
    pinHandle.GPIO_Config.speed      = GPIO_SPEED_HIGH;
    pinHandle.GPIO_Config.pull       = GPIO_PUPD_NONE;
    pinHandle.GPIO_Config.altFn      = GPIO_AF4;
    GPIO_Init(&pinHandle);
}

/* =========================================================================
 * 3. SSD1306 OLED (PB10: I2C2_SCL, PB3: I2C2_SDA)
 * ========================================================================= */
void Pin_Config_OLED(void)
{
    GPIO_Handle_t pinHandle;

    /* SCL Pin (PB10) - I2C2_SCL (AF4, Open-Drain) */
    pinHandle.pGPIOx                 = PIN_OLED_SCL_PORT;
    pinHandle.GPIO_Config.pin        = PIN_OLED_SCL_PIN;
    pinHandle.GPIO_Config.mode       = GPIO_MODE_ALTFN;
    pinHandle.GPIO_Config.outputType = GPIO_OTYPE_OD;
    pinHandle.GPIO_Config.speed      = GPIO_SPEED_HIGH;
    pinHandle.GPIO_Config.pull       = GPIO_PUPD_NONE;
    pinHandle.GPIO_Config.altFn      = GPIO_AF4;
    GPIO_Init(&pinHandle);

    /* SDA Pin (PB3) - I2C2_SDA (AF9 on STM32F401, Open-Drain) */
    pinHandle.pGPIOx                 = PIN_OLED_SDA_PORT;
    pinHandle.GPIO_Config.pin        = PIN_OLED_SDA_PIN;
    pinHandle.GPIO_Config.mode       = GPIO_MODE_ALTFN;
    pinHandle.GPIO_Config.outputType = GPIO_OTYPE_OD;
    pinHandle.GPIO_Config.speed      = GPIO_SPEED_HIGH;
    pinHandle.GPIO_Config.pull       = GPIO_PUPD_NONE;
    pinHandle.GPIO_Config.altFn      = GPIO_AF9;
    GPIO_Init(&pinHandle);
}

/* =========================================================================
 * 4. TB6612FNG MOTOR DRIVER (PA12: STBY, PB12-15: DIR, PB4-5: TIM3 PWM)
 * ========================================================================= */
void Pin_Config_Motor(void)
{
    GPIO_Handle_t pinHandle;

    /* 1. Standby Pin (PA12) - Output Push-Pull, High to enable driver */
    pinHandle.pGPIOx                 = PIN_MOTOR_STBY_PORT;
    pinHandle.GPIO_Config.pin        = PIN_MOTOR_STBY_PIN;
    pinHandle.GPIO_Config.mode       = GPIO_MODE_OUTPUT;
    pinHandle.GPIO_Config.outputType = GPIO_OTYPE_PP;
    pinHandle.GPIO_Config.speed      = GPIO_SPEED_FAST;
    pinHandle.GPIO_Config.pull       = GPIO_PUPD_NONE;
    GPIO_Init(&pinHandle);
    GPIO_Write_Pin(PIN_MOTOR_STBY_PORT, PIN_MOTOR_STBY_PIN, GPIO_PIN_SET);

    /* 2. Direction Pins (PB12: AIN2, PB13: AIN1, PB14: BIN1, PB15: BIN2) */
    pinHandle.pGPIOx                 = GPIOB;
    pinHandle.GPIO_Config.mode       = GPIO_MODE_OUTPUT;
    pinHandle.GPIO_Config.outputType = GPIO_OTYPE_PP;
    pinHandle.GPIO_Config.speed      = GPIO_SPEED_FAST;
    pinHandle.GPIO_Config.pull       = GPIO_PUPD_NONE;

    pinHandle.GPIO_Config.pin = PIN_MOTOR_AIN1_PIN;
    GPIO_Init(&pinHandle);
    pinHandle.GPIO_Config.pin = PIN_MOTOR_AIN2_PIN;
    GPIO_Init(&pinHandle);
    pinHandle.GPIO_Config.pin = PIN_MOTOR_BIN1_PIN;
    GPIO_Init(&pinHandle);
    pinHandle.GPIO_Config.pin = PIN_MOTOR_BIN2_PIN;
    GPIO_Init(&pinHandle);

    /* Clear all direction pins initially (motor coast/stop) */
    GPIO_Write_Pin(GPIOB, PIN_MOTOR_AIN1_PIN, GPIO_PIN_RESET);
    GPIO_Write_Pin(GPIOB, PIN_MOTOR_AIN2_PIN, GPIO_PIN_RESET);
    GPIO_Write_Pin(GPIOB, PIN_MOTOR_BIN1_PIN, GPIO_PIN_RESET);
    GPIO_Write_Pin(GPIOB, PIN_MOTOR_BIN2_PIN, GPIO_PIN_RESET);

    /* 3. PWM Pins (PB4: TIM3_CH1, PB5: TIM3_CH2) - AF2 Push-Pull */
    pinHandle.pGPIOx                 = GPIOB;
    pinHandle.GPIO_Config.mode       = GPIO_MODE_ALTFN;
    pinHandle.GPIO_Config.outputType = GPIO_OTYPE_PP;
    pinHandle.GPIO_Config.speed      = GPIO_SPEED_FAST;
    pinHandle.GPIO_Config.pull       = GPIO_PUPD_NONE;
    pinHandle.GPIO_Config.altFn      = GPIO_AF2;

    pinHandle.GPIO_Config.pin = PIN_MOTOR_PWMA_PIN;
    GPIO_Init(&pinHandle);

    pinHandle.GPIO_Config.pin = PIN_MOTOR_PWMB_PIN;
    GPIO_Init(&pinHandle);
}

/* =========================================================================
 * 5. WHEEL ENCODERS (PA8/PA9: TIM1 Left, PB6/PB7: TIM4 Right)
 * ========================================================================= */
void Pin_Config_Encoder(void)
{
    GPIO_Handle_t pinHandle;

    /* 1. Left Motor Encoder -> TIM1 (PA8: CH1, PA9: CH2) - AF1 with Pull-up */
    pinHandle.pGPIOx                 = GPIOA;
    pinHandle.GPIO_Config.mode       = GPIO_MODE_ALTFN;
    pinHandle.GPIO_Config.outputType = GPIO_OTYPE_PP;
    pinHandle.GPIO_Config.speed      = GPIO_SPEED_FAST;
    pinHandle.GPIO_Config.pull       = GPIO_PUPD_PULLUP;
    pinHandle.GPIO_Config.altFn      = GPIO_AF1;

    pinHandle.GPIO_Config.pin = PIN_ENC_LEFT_A_PIN;
    GPIO_Init(&pinHandle);

    pinHandle.GPIO_Config.pin = PIN_ENC_LEFT_B_PIN;
    GPIO_Init(&pinHandle);

    /* 2. Right Motor Encoder -> TIM4 (PB6: CH1, PB7: CH2) - AF2 with Pull-up */
    pinHandle.pGPIOx                 = GPIOB;
    pinHandle.GPIO_Config.mode       = GPIO_MODE_ALTFN;
    pinHandle.GPIO_Config.outputType = GPIO_OTYPE_PP;
    pinHandle.GPIO_Config.speed      = GPIO_SPEED_FAST;
    pinHandle.GPIO_Config.pull       = GPIO_PUPD_PULLUP;
    pinHandle.GPIO_Config.altFn      = GPIO_AF2;

    pinHandle.GPIO_Config.pin = PIN_ENC_RIGHT_A_PIN;
    GPIO_Init(&pinHandle);

    pinHandle.GPIO_Config.pin = PIN_ENC_RIGHT_B_PIN;
    GPIO_Init(&pinHandle);
}

/* =========================================================================
 * 6. ON-BOARD BUILT-IN LED (PC13)
 * ========================================================================= */
void Pin_Config_LED(void)
{
    GPIO_Handle_t pinHandle;

    /* Built-in LED (PC13) - Output Push-Pull, Fast Speed, No Pull */
    pinHandle.pGPIOx                 = PIN_LED_PORT;
    pinHandle.GPIO_Config.pin        = PIN_LED_PIN;
    pinHandle.GPIO_Config.mode       = GPIO_MODE_OUTPUT;
    pinHandle.GPIO_Config.outputType = GPIO_OTYPE_PP;
    pinHandle.GPIO_Config.speed      = GPIO_SPEED_FAST;
    pinHandle.GPIO_Config.pull       = GPIO_PUPD_NONE;
    GPIO_Init(&pinHandle);

    /* Turn off LED initially (Active Low on STM32 Black Pill: SET = OFF) */
    GPIO_Write_Pin(PIN_LED_PORT, PIN_LED_PIN, GPIO_PIN_SET);
}

/* =========================================================================
 * 7. ALL PINS INITIALIZATION
 * ========================================================================= */
void Pin_Config_All(void)
{
    Pin_Config_LED();
    Pin_Config_HC05();
    Pin_Config_MPU6050();
    Pin_Config_OLED();
    Pin_Config_Motor();
    Pin_Config_Encoder();
}
