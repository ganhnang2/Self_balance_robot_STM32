/**
 * @file    stm32f4xx_gpio.h
 * @author  Nguyen Hai Nam
 * @brief   GPIO Driver for STM32F4xx microcontrollers.
 *          This file contains data structure definitions, configurations, and API prototypes
 *          for interfacing with the GPIO (General Purpose Input/Output) peripheral.
 *
 * @details
 * === IMPLEMENTED FEATURES ===
 *
 * 1. Initialization & De-initialization:
 *    - GPIO_Init()      : Configures a GPIO pin (mode, speed, pull, alternate function, output type).
 *    - GPIO_DeInit()    : Resets an entire GPIO port to its default state via RCC reset.
 *
 * 2. Digital I/O (Polling):
 *    - GPIO_Read_Pin()  : Reads the logic level of an input pin.
 *    - GPIO_Read_Port() : Reads the entire Input Data Register of a port.
 *    - GPIO_Write_Pin() : Writes a logic level (SET/RESET) to an output pin.
 *    - GPIO_Write_Port(): Writes a 16-bit value to the Output Data Register.
 *    - GPIO_Toggle_Pin(): Toggles the current output level of a pin.
 *
 * 3. Interrupt (EXTI) mode:
 *    - GPIO_Init() with mode GPIO_MODE_IT_RISING / FALLING / RISING_FALLING
 *      automatically configures the SYSCFG EXTI mux, EXTI edge triggers,
 *      and unmasks the EXTI line. The user only needs to enable the
 *      corresponding IRQ in NVIC and implement GPIO_EXTI_Callback().
 *    - GPIO_EXTI_IRQHandling() : Clears the pending EXTI flag, then calls
 *                                GPIO_EXTI_Callback() with the pin number.
 *    - GPIO_EXTI_Callback()    : Weak function — override in application code.
 *
 * === GPIO INITIALIZATION PROCEDURE ===
 * 1. Fill in a GPIO_PinConfig_t structure with desired pin, mode, speed, etc.
 * 2. Call GPIO_Init(&config) — the function enables the GPIO clock automatically.
 * 3. For interrupt pins, additionally call NVIC_EnableIRQ(EXTIx_IRQn) in application.
 * 4. Implement GPIO_EXTI_Callback() in application to handle the interrupt.
 *
 * === SUPPORTED GPIO PORTS ===
 * GPIOA .. GPIOI  (availability depends on the specific STM32F4xx device)
 *
 * === ALTERNATE FUNCTION NUMBERS (AF0 .. AF15) ===
 * Refer to the device datasheet Alternate Function table. Common examples:
 *   AF0  = SYS (MCO, JTAG/SWD)
 *   AF1  = TIM1/TIM2
 *   AF2  = TIM3/TIM4/TIM5
 *   AF4  = I2C1/I2C2/I2C3
 *   AF5  = SPI1/SPI2
 *   AF6  = SPI3
 *   AF7  = USART1/USART2/USART3
 *   AF8  = USART4/USART5/USART6
 *   AF9  = CAN1/CAN2/TIM12/TIM13/TIM14
 *   AF10 = OTG_FS/OTG_HS
 *   AF11 = ETH
 *   AF12 = FSMC/SDIO/OTG_HS
 *   AF15 = EVENTOUT
 */

#ifndef STM32F4XX_GPIO_H_
#define STM32F4XX_GPIO_H_

#include "stm32f401xc.h"
#include "core_cm4.h"

#ifndef ENABLE
#define ENABLE  1
#endif
#ifndef DISABLE
#define DISABLE 0
#endif

/* =========================================================================
 * @defgroup GPIO_Pin_Number  GPIO Pin Numbers
 * ========================================================================= */
#define GPIO_PIN_0      0U
#define GPIO_PIN_1      1U
#define GPIO_PIN_2      2U
#define GPIO_PIN_3      3U
#define GPIO_PIN_4      4U
#define GPIO_PIN_5      5U
#define GPIO_PIN_6      6U
#define GPIO_PIN_7      7U
#define GPIO_PIN_8      8U
#define GPIO_PIN_9      9U
#define GPIO_PIN_10     10U
#define GPIO_PIN_11     11U
#define GPIO_PIN_12     12U
#define GPIO_PIN_13     13U
#define GPIO_PIN_14     14U
#define GPIO_PIN_15     15U

/* =========================================================================
 * @defgroup GPIO_Mode  GPIO Pin Mode
 *   Bits [1:0] = base mode (matching MODER register encoding)
 *   Bit  [2]   = interrupt flag (set internally)
 * ========================================================================= */
#define GPIO_MODE_INPUT             0U  /*!< Digital input (floating / pulled)       */
#define GPIO_MODE_OUTPUT            1U  /*!< Digital output (push-pull or open-drain) */
#define GPIO_MODE_ALTFN             2U  /*!< Alternate function                       */
#define GPIO_MODE_ANALOG            3U  /*!< Analog / ADC input                       */
#define GPIO_MODE_IT_RISING         4U  /*!< External interrupt, rising edge trigger  */
#define GPIO_MODE_IT_FALLING        5U  /*!< External interrupt, falling edge trigger */
#define GPIO_MODE_IT_RISING_FALLING 6U  /*!< External interrupt, both edges           */

/* =========================================================================
 * @defgroup GPIO_OutputType  GPIO Output Type
 * ========================================================================= */
#define GPIO_OTYPE_PP   0U  /*!< Push-pull  (default for digital output) */
#define GPIO_OTYPE_OD   1U  /*!< Open-drain (required for I2C, etc.)     */

/* =========================================================================
 * @defgroup GPIO_Speed  GPIO Output Speed
 *   Maps directly to OSPEEDR register encoding.
 * ========================================================================= */
#define GPIO_SPEED_LOW      0U  /*!<  2 MHz   */
#define GPIO_SPEED_MEDIUM   1U  /*!< 25 MHz   */
#define GPIO_SPEED_FAST     2U  /*!< 50 MHz   */
#define GPIO_SPEED_HIGH     3U  /*!< 100 MHz  */

/* =========================================================================
 * @defgroup GPIO_PullUpDown  GPIO Pull-up / Pull-down
 *   Maps directly to PUPDR register encoding.
 * ========================================================================= */
#define GPIO_PUPD_NONE      0U  /*!< No pull resistor          */
#define GPIO_PUPD_PULLUP    1U  /*!< Internal pull-up enabled  */
#define GPIO_PUPD_PULLDOWN  2U  /*!< Internal pull-down enabled */

/* =========================================================================
 * @defgroup GPIO_AltFn  Alternate Function Numbers
 * ========================================================================= */
#define GPIO_AF0    0U
#define GPIO_AF1    1U
#define GPIO_AF2    2U
#define GPIO_AF3    3U
#define GPIO_AF4    4U
#define GPIO_AF5    5U
#define GPIO_AF6    6U
#define GPIO_AF7    7U
#define GPIO_AF8    8U
#define GPIO_AF9    9U
#define GPIO_AF10   10U
#define GPIO_AF11   11U
#define GPIO_AF12   12U
#define GPIO_AF13   13U
#define GPIO_AF14   14U
#define GPIO_AF15   15U

/* =========================================================================
 * @defgroup GPIO_PinState  GPIO Pin Logic State
 * ========================================================================= */
#define GPIO_PIN_RESET  0U  /*!< Logic LOW  */
#define GPIO_PIN_SET    1U  /*!< Logic HIGH */

/* =========================================================================
 * @brief  GPIO Pin Configuration Structure
 *
 * @note   Fill in all fields relevant to the chosen mode:
 *         - INPUT / IT modes  : pin, mode, pull
 *         - OUTPUT mode       : pin, mode, speed, pull, outputType
 *         - ALTFN  mode       : pin, mode, speed, pull, outputType, altFn
 *         - ANALOG mode       : pin, mode  (only)
 * ========================================================================= */
typedef struct
{
    uint8_t pin;        /*!< Pin number: GPIO_PIN_0 .. GPIO_PIN_15                  */
    uint8_t mode;       /*!< Pin mode: GPIO_MODE_INPUT / OUTPUT / ALTFN / ANALOG /
                             GPIO_MODE_IT_RISING / IT_FALLING / IT_RISING_FALLING   */
    uint8_t speed;      /*!< Output speed: GPIO_SPEED_LOW .. GPIO_SPEED_HIGH        */
    uint8_t pull;       /*!< Pull-up/down: GPIO_PUPD_NONE / PULLUP / PULLDOWN       */
    uint8_t outputType; /*!< Output type: GPIO_OTYPE_PP (push-pull) or GPIO_OTYPE_OD (open-drain) */
    uint8_t altFn;      /*!< Alternate function number: GPIO_AF0 .. GPIO_AF15       */
} GPIO_PinConfig_t;

/* =========================================================================
 * @brief  GPIO Handle Structure
 * ========================================================================= */
typedef struct
{
    GPIO_TypeDef     *pGPIOx;  			/*!< Pointer to the GPIO port base address (GPIOA..GPIOI) */
    GPIO_PinConfig_t  GPIO_Config; 		 /*!< Pin configuration parameters                          */
} GPIO_Handle_t;

/* =========================================================================
 * @defgroup GPIO_APIs  GPIO Driver API
 * @{
 * ========================================================================= */

/** @name Initialization */
/** @{ */

/**
 * @brief  Initialize a GPIO pin according to GPIO_Handle_t configuration.
 *
 * @details  Automatically:
 *           - Enables the GPIO port clock (AHB1 bus).
 *           - Configures MODER, OTYPER, OSPEEDR, PUPDR, and AFR registers.
 *           - For interrupt modes (GPIO_MODE_IT_*): configures SYSCFG EXTI mux,
 *             EXTI rising/falling trigger registers, and unmasks the EXTI line.
 *
 * @param  pHandle  Pointer to a GPIO_Handle_t structure with pGPIOx and config filled in.
 *
 * @return None
 */
void GPIO_Init(GPIO_Handle_t *pHandle);

/**
 * @brief  Reset an entire GPIO port to its reset state via RCC peripheral reset.
 *
 * @param  pGPIOx  Pointer to the GPIO port base address (e.g., GPIOA, GPIOB).
 *
 * @return None
 */
void GPIO_DeInit(GPIO_TypeDef *pGPIOx);

/** @} */

/** @name Digital I/O */
/** @{ */

/**
 * @brief  Read the digital input level of a single GPIO pin.
 *
 * @param  pGPIOx  Pointer to the GPIO port base address.
 * @param  pin     Pin number: GPIO_PIN_0 .. GPIO_PIN_15.
 *
 * @return GPIO_PIN_SET (1) or GPIO_PIN_RESET (0).
 */
uint8_t GPIO_Read_Pin(GPIO_TypeDef *pGPIOx, uint8_t pin);

/**
 * @brief  Read the entire 16-bit Input Data Register of a GPIO port.
 *
 * @param  pGPIOx  Pointer to the GPIO port base address.
 *
 * @return 16-bit value of IDR register.
 */
uint16_t GPIO_Read_Port(GPIO_TypeDef *pGPIOx);

/**
 * @brief  Write a logic level to a GPIO output pin.
 *
 * @param  pGPIOx  Pointer to the GPIO port base address.
 * @param  pin     Pin number: GPIO_PIN_0 .. GPIO_PIN_15.
 * @param  state   GPIO_PIN_SET or GPIO_PIN_RESET.
 *
 * @return None
 */
void GPIO_Write_Pin(GPIO_TypeDef *pGPIOx, uint8_t pin, uint8_t state);

/**
 * @brief  Write a 16-bit value directly to the Output Data Register of a port.
 *
 * @param  pGPIOx  Pointer to the GPIO port base address.
 * @param  value   16-bit value to write to ODR.
 *
 * @return None
 */
void GPIO_Write_Port(GPIO_TypeDef *pGPIOx, uint16_t value);

/**
 * @brief  Toggle the current output level of a GPIO pin.
 *
 * @note   Uses ODR XOR for an atomic toggle (single read-modify-write).
 *
 * @param  pGPIOx  Pointer to the GPIO port base address.
 * @param  pin     Pin number: GPIO_PIN_0 .. GPIO_PIN_15.
 *
 * @return None
 */
void GPIO_Toggle_Pin(GPIO_TypeDef *pGPIOx, uint8_t pin);

/** @} */

/** @name External Interrupt Handling */
/** @{ */

/**
 * @brief  Clear the EXTI pending flag and invoke the application callback.
 *
 * @note   Call this function from the EXTIx_IRQHandler in your application:
 *
 *         void EXTI0_IRQHandler(void) { GPIO_EXTI_IRQHandling(GPIO_PIN_0); }
 *
 * @param  pin  The GPIO pin number whose EXTI line fired (GPIO_PIN_0 .. GPIO_PIN_15).
 *
 * @return None
 */
void GPIO_EXTI_IRQHandling(uint8_t pin);

/**
 * @brief  Application-level EXTI callback — implement this in your application code.
 *
 * @note   This is declared with __attribute__((weak)) so it compiles even if
 *         the user does not override it.
 *
 * @param  pin  The GPIO pin number that triggered the interrupt.
 *
 * @return None
 */
void GPIO_EXTI_Callback(uint8_t pin);

/** @} */

/** @} */ /* End of GPIO_APIs */

#endif /* STM32F4XX_GPIO_H_ */
