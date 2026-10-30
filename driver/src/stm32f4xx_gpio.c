/**
 * @file    stm32f4xx_gpio.c
 * @author  Nguyen Hai Nam
 * @brief   GPIO Driver implementation for STM32F4xx.
 *
 * @details
 *  Registers used per operation:
 *   - MODER   : Pin mode (2 bits/pin)
 *   - OTYPER  : Output type (1 bit/pin)
 *   - OSPEEDR : Output speed (2 bits/pin)
 *   - PUPDR   : Pull-up/down (2 bits/pin)
 *   - AFR[0]  : Alternate function for pins 0-7  (4 bits/pin)
 *   - AFR[1]  : Alternate function for pins 8-15 (4 bits/pin)
 *   - IDR     : Input Data Register (read-only)
 *   - ODR     : Output Data Register
 *   - BSRR    : Bit Set/Reset Register (write-only, atomic)
 *   - SYSCFG->EXTICR[]: Maps GPIO port to EXTI line
 *   - EXTI->RTSR / FTSR : Rising/Falling trigger select
 *   - EXTI->IMR         : Interrupt mask (unmask = enable)
 */

#include "stm32f4xx_gpio.h"

/* =========================================================================
 * Private helper prototypes
 * ========================================================================= */
static void gpio_clock_enable(GPIO_TypeDef *pGPIOx);
static void gpio_clock_disable(GPIO_TypeDef *pGPIOx);
static uint8_t gpio_get_port_code(GPIO_TypeDef *pGPIOx);

/* =========================================================================
 * @brief  Enable the AHB1 clock for the given GPIO port.
 * ========================================================================= */
static void gpio_clock_enable(GPIO_TypeDef *pGPIOx)
{
    if      (pGPIOx == GPIOA) { RCC->AHB1ENR |= (1U << RCC_AHB1ENR_GPIOAEN_Pos); }
    else if (pGPIOx == GPIOB) { RCC->AHB1ENR |= (1U << RCC_AHB1ENR_GPIOBEN_Pos); }
    else if (pGPIOx == GPIOC) { RCC->AHB1ENR |= (1U << RCC_AHB1ENR_GPIOCEN_Pos); }
    else if (pGPIOx == GPIOD) { RCC->AHB1ENR |= (1U << RCC_AHB1ENR_GPIODEN_Pos); }
    else if (pGPIOx == GPIOE) { RCC->AHB1ENR |= (1U << RCC_AHB1ENR_GPIOEEN_Pos); }
#ifdef GPIOF
    else if (pGPIOx == GPIOF) { RCC->AHB1ENR |= (1U << RCC_AHB1ENR_GPIOFEN_Pos); }
#endif
#ifdef GPIOG
    else if (pGPIOx == GPIOG) { RCC->AHB1ENR |= (1U << RCC_AHB1ENR_GPIOGEN_Pos); }
#endif
    else if (pGPIOx == GPIOH) { RCC->AHB1ENR |= (1U << RCC_AHB1ENR_GPIOHEN_Pos); }
#ifdef GPIOI
    else if (pGPIOx == GPIOI) { RCC->AHB1ENR |= (1U << RCC_AHB1ENR_GPIOIEN_Pos); }
#endif
}

/* =========================================================================
 * @brief  Disable the AHB1 clock for the given GPIO port.
 * ========================================================================= */
static void gpio_clock_disable(GPIO_TypeDef *pGPIOx)
{
    if      (pGPIOx == GPIOA) { RCC->AHB1ENR &= ~(1U << RCC_AHB1ENR_GPIOAEN_Pos); }
    else if (pGPIOx == GPIOB) { RCC->AHB1ENR &= ~(1U << RCC_AHB1ENR_GPIOBEN_Pos); }
    else if (pGPIOx == GPIOC) { RCC->AHB1ENR &= ~(1U << RCC_AHB1ENR_GPIOCEN_Pos); }
    else if (pGPIOx == GPIOD) { RCC->AHB1ENR &= ~(1U << RCC_AHB1ENR_GPIODEN_Pos); }
    else if (pGPIOx == GPIOE) { RCC->AHB1ENR &= ~(1U << RCC_AHB1ENR_GPIOEEN_Pos); }
#ifdef GPIOF
    else if (pGPIOx == GPIOF) { RCC->AHB1ENR &= ~(1U << RCC_AHB1ENR_GPIOFEN_Pos); }
#endif
#ifdef GPIOG
    else if (pGPIOx == GPIOG) { RCC->AHB1ENR &= ~(1U << RCC_AHB1ENR_GPIOGEN_Pos); }
#endif
    else if (pGPIOx == GPIOH) { RCC->AHB1ENR &= ~(1U << RCC_AHB1ENR_GPIOHEN_Pos); }
#ifdef GPIOI
    else if (pGPIOx == GPIOI) { RCC->AHB1ENR &= ~(1U << RCC_AHB1ENR_GPIOIEN_Pos); }
#endif
}

/* =========================================================================
 * @brief  Return the SYSCFG EXTICR port-select code (0=A, 1=B, ... 8=I).
 *         This value is written into SYSCFG->EXTICR[pin/4] bits [(pin%4)*4+3 : (pin%4)*4].
 * ========================================================================= */
static uint8_t gpio_get_port_code(GPIO_TypeDef *pGPIOx)
{
    if      (pGPIOx == GPIOA) { return 0U; }
    else if (pGPIOx == GPIOB) { return 1U; }
    else if (pGPIOx == GPIOC) { return 2U; }
    else if (pGPIOx == GPIOD) { return 3U; }
    else if (pGPIOx == GPIOE) { return 4U; }
#ifdef GPIOF
    else if (pGPIOx == GPIOF) { return 5U; }
#endif
#ifdef GPIOG
    else if (pGPIOx == GPIOG) { return 6U; }
#endif
    else if (pGPIOx == GPIOH) { return 7U; }
    else                       { return 8U; } /* GPIOI */
}

/* =========================================================================
 *                       PUBLIC API IMPLEMENTATIONS
 * ========================================================================= */

/**
 * @brief  Initialize a GPIO pin according to the configuration in pHandle.
 *
 * Steps:
 *  1. Enable the port clock.
 *  2. Configure the pin mode (MODER).
 *  3. For interrupt modes: configure SYSCFG EXTI mux + edge triggers + unmask.
 *  4. Configure output type (OTYPER) — meaningful only for OUTPUT/ALTFN.
 *  5. Configure output speed (OSPEEDR) — meaningful only for OUTPUT/ALTFN.
 *  6. Configure pull-up/pull-down (PUPDR).
 *  7. Configure alternate function (AFR) — meaningful only for ALTFN mode.
 */
void GPIO_Init(GPIO_Handle_t *pHandle)
{
    uint8_t pin    = pHandle->GPIO_Config.pin;
    uint8_t mode   = pHandle->GPIO_Config.mode;

    /* ---- 1. Enable peripheral clock ---- */
    gpio_clock_enable(pHandle->pGPIOx);

    /* ---- 2. Configure MODER ---- */
    /*
     * For interrupt modes the physical pin must be configured as digital INPUT
     * (MODER = 00). We still use a separate mode value (4/5/6) to know we need
     * to set up EXTI, but we write 0b00 into the MODER register.
     */
    pHandle->pGPIOx->MODER &= ~(0x3U << (pin * 2U));

    if (mode <= GPIO_MODE_ANALOG)
    {
        /* Standard modes: INPUT=0, OUTPUT=1, ALTFN=2, ANALOG=3 */
        pHandle->pGPIOx->MODER |= ((uint32_t)(mode & 0x03U) << (pin * 2U));
    }
    /* else: IT modes → MODER stays at 00 (INPUT) */

    /* ---- 3. Configure EXTI for interrupt modes ---- */
    if (mode >= GPIO_MODE_IT_RISING)
    {
        /* 3a. Enable SYSCFG clock (needed to write EXTICR) */
        RCC->APB2ENR |= (1U << RCC_APB2ENR_SYSCFGEN_Pos);

        /* 3b. Select GPIO port in SYSCFG_EXTICR[n] */
        uint8_t exticr_idx  = pin / 4U;            /* which EXTICR register (0..3)  */
        uint8_t exticr_shift = (pin % 4U) * 4U;    /* bit offset within the register */
        SYSCFG->EXTICR[exticr_idx] &= ~(0xFU << exticr_shift);
        SYSCFG->EXTICR[exticr_idx] |= ((uint32_t)gpio_get_port_code(pHandle->pGPIOx) << exticr_shift);

        /* 3c. Configure edge trigger in EXTI RTSR / FTSR */
        EXTI->RTSR &= ~(1U << pin);
        EXTI->FTSR &= ~(1U << pin);

        if (mode == GPIO_MODE_IT_RISING || mode == GPIO_MODE_IT_RISING_FALLING)
        {
            EXTI->RTSR |= (1U << pin);
        }
        if (mode == GPIO_MODE_IT_FALLING || mode == GPIO_MODE_IT_RISING_FALLING)
        {
            EXTI->FTSR |= (1U << pin);
        }

        /* 3d. Unmask the EXTI line (enable interrupt delivery to NVIC) */
        EXTI->IMR |= (1U << pin);
    }

    /* ---- 4. Configure Output Type (OTYPER) ---- */
    pHandle->pGPIOx->OTYPER &= ~(1U << pin);
    pHandle->pGPIOx->OTYPER |= ((uint32_t)(pHandle->GPIO_Config.outputType & 0x01U) << pin);

    /* ---- 5. Configure Output Speed (OSPEEDR) ---- */
    pHandle->pGPIOx->OSPEEDR &= ~(0x3U << (pin * 2U));
    pHandle->pGPIOx->OSPEEDR |= ((uint32_t)(pHandle->GPIO_Config.speed & 0x03U) << (pin * 2U));

    /* ---- 6. Configure Pull-up / Pull-down (PUPDR) ---- */
    pHandle->pGPIOx->PUPDR &= ~(0x3U << (pin * 2U));
    pHandle->pGPIOx->PUPDR |= ((uint32_t)(pHandle->GPIO_Config.pull & 0x03U) << (pin * 2U));

    /* ---- 7. Configure Alternate Function (AFR) ---- */
    if (mode == GPIO_MODE_ALTFN)
    {
        /*
         * AFR[0] covers pins 0-7  (4 bits each → shift = pin * 4)
         * AFR[1] covers pins 8-15 (4 bits each → shift = (pin - 8) * 4)
         */
        uint8_t afr_idx   = pin / 8U;       /* 0 for pins 0-7, 1 for pins 8-15 */
        uint8_t afr_shift = (pin % 8U) * 4U;

        pHandle->pGPIOx->AFR[afr_idx] &= ~(0xFU << afr_shift);
        pHandle->pGPIOx->AFR[afr_idx] |=  ((uint32_t)(pHandle->GPIO_Config.altFn & 0x0FU) << afr_shift);
    }
}

/**
 * @brief  Reset an entire GPIO port using the RCC peripheral reset mechanism.
 *
 *         All registers of the port are restored to their reset values.
 *         The port clock is disabled after the reset.
 */
void GPIO_DeInit(GPIO_TypeDef *pGPIOx)
{
    /* Pulse the reset bit for the target port, then clear it */
    if      (pGPIOx == GPIOA)
    {
        RCC->AHB1RSTR |= (1U << RCC_AHB1RSTR_GPIOARST_Pos);
        RCC->AHB1RSTR &= ~(1U << RCC_AHB1RSTR_GPIOARST_Pos);
    }
    else if (pGPIOx == GPIOB)
    {
        RCC->AHB1RSTR |= (1U << RCC_AHB1RSTR_GPIOBRST_Pos);
        RCC->AHB1RSTR &= ~(1U << RCC_AHB1RSTR_GPIOBRST_Pos);
    }
    else if (pGPIOx == GPIOC)
    {
        RCC->AHB1RSTR |= (1U << RCC_AHB1RSTR_GPIOCRST_Pos);
        RCC->AHB1RSTR &= ~(1U << RCC_AHB1RSTR_GPIOCRST_Pos);
    }
    else if (pGPIOx == GPIOD)
    {
        RCC->AHB1RSTR |= (1U << RCC_AHB1RSTR_GPIODRST_Pos);
        RCC->AHB1RSTR &= ~(1U << RCC_AHB1RSTR_GPIODRST_Pos);
    }
    else if (pGPIOx == GPIOE)
    {
        RCC->AHB1RSTR |= (1U << RCC_AHB1RSTR_GPIOERST_Pos);
        RCC->AHB1RSTR &= ~(1U << RCC_AHB1RSTR_GPIOERST_Pos);
    }
#ifdef GPIOF
    else if (pGPIOx == GPIOF)
    {
        RCC->AHB1RSTR |= (1U << RCC_AHB1RSTR_GPIOFRST_Pos);
        RCC->AHB1RSTR &= ~(1U << RCC_AHB1RSTR_GPIOFRST_Pos);
    }
#endif
#ifdef GPIOG
    else if (pGPIOx == GPIOG)
    {
        RCC->AHB1RSTR |= (1U << RCC_AHB1RSTR_GPIOGRST_Pos);
        RCC->AHB1RSTR &= ~(1U << RCC_AHB1RSTR_GPIOGRST_Pos);
    }
#endif
    else if (pGPIOx == GPIOH)
    {
        RCC->AHB1RSTR |= (1U << RCC_AHB1RSTR_GPIOHRST_Pos);
        RCC->AHB1RSTR &= ~(1U << RCC_AHB1RSTR_GPIOHRST_Pos);
    }
#ifdef GPIOI
    else if (pGPIOx == GPIOI)
    {
        RCC->AHB1RSTR |= (1U << RCC_AHB1RSTR_GPIOIRST_Pos);
        RCC->AHB1RSTR &= ~(1U << RCC_AHB1RSTR_GPIOIRST_Pos);
    }
#endif

    /* Disable port clock after reset */
    gpio_clock_disable(pGPIOx);
}

/**
 * @brief  Read the logic level of one GPIO input pin.
 *
 * @return GPIO_PIN_SET (1) or GPIO_PIN_RESET (0).
 */
uint8_t GPIO_Read_Pin(GPIO_TypeDef *pGPIOx, uint8_t pin)
{
    return (uint8_t)((pGPIOx->IDR >> pin) & 0x01U);
}

/**
 * @brief  Read all 16 input pins of a GPIO port.
 *
 * @return Lower 16 bits of IDR.
 */
uint16_t GPIO_Read_Port(GPIO_TypeDef *pGPIOx)
{
    return (uint16_t)(pGPIOx->IDR & 0xFFFFU);
}

/**
 * @brief  Write a logic level to a GPIO output pin.
 *
 */
void GPIO_Write_Pin(GPIO_TypeDef *pGPIOx, uint8_t pin, uint8_t state)
{
    if (state == GPIO_PIN_SET)
    {
        pGPIOx->ODR |= (1U << pin);
    }
    else
    {
        pGPIOx->ODR &= ~(1U << pin);
    }
}

/**
 * @brief  Write a 16-bit value directly to the GPIO Output Data Register.
 */
void GPIO_Write_Port(GPIO_TypeDef *pGPIOx, uint16_t value)
{
    pGPIOx->ODR = (uint32_t)value;
}

/**
 * @brief  Toggle the output level of a GPIO pin using XOR on ODR.
 */
void GPIO_Toggle_Pin(GPIO_TypeDef *pGPIOx, uint8_t pin)
{
    pGPIOx->ODR ^= (1U << pin);
}

/**
 * @brief  Clear the EXTI pending flag for the given pin, then call the callback.
 *
 *         The EXTI_PR register is write-1-to-clear, so writing the bit back
 *         clears the pending flag without affecting any other pending lines.
 *
 *         Call this from the EXTIx_IRQHandler in your application:
 *
 *           void EXTI0_IRQHandler(void) { GPIO_EXTI_IRQHandling(GPIO_PIN_0); }
 *           void EXTI9_5_IRQHandler(void)
 *           {
 *               for (uint8_t pin = 5; pin <= 9; pin++)
 *                   if (EXTI->PR & (1U << pin)) GPIO_EXTI_IRQHandling(pin);
 *           }
 */
void GPIO_EXTI_IRQHandling(uint8_t pin)
{
    /* Clear the pending flag (write 1 to clear) */
    if (EXTI->PR & (1U << pin))
    {
        EXTI->PR |= (1U << pin);
    }

    /* Call the user callback */
    GPIO_EXTI_Callback(pin);
}

/**
 * @brief  Default (weak) implementation of the application EXTI callback.
 *
 *         Override this in your application source file:
 *
 *           void GPIO_EXTI_Callback(uint8_t pin)
 *           {
 *               if (pin == GPIO_PIN_0) { ... }
 *           }
 */
__attribute__((weak)) void GPIO_EXTI_Callback(uint8_t pin)
{
    (void)pin;  /* Prevent unused-parameter warning */
    /* User must override this function in application code */
}
