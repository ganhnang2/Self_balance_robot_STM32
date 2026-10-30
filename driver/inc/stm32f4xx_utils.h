/**
 * @file    stm32f4xx_utils.h
 * @author  Nguyen Hai Nam
 * @brief   Utility functions, clock configuration, and delay services for STM32F4xx.
 *
 * @note
 * 1. Clock configuration: System can be clocked from HSE directly (enable_external_clock)
 *    or boosted via PLL (enable_PLL_clock). Only one clock selection function should be active.
 * 2. SystemCoreClock: Global variable storing the active system clock frequency in Hz.
 *    Automatically updated after calling enable_external_clock() or enable_PLL_clock().
 * 3. Delay implementation: Delay functions rely on the TIM4 peripheral (1 tick = 1 us).
 *    Avoid using TIM4 in the main application, or adapt delay functions to SysTick if needed.
 * 4. SystemInit(): Automatically coordinates FPU activation, clock configuration, and delay timer initialization.
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h> 
#include "stm32f401xc.h"
#include "core_cm4.h"

#ifndef ENABLE
#define ENABLE		1
#endif
#ifndef DISABLE
#define DISABLE		0
#endif

/**
 * @brief  Global variable storing current system core clock frequency in Hz.
 * @note   Initialized to HSI_VALUE and updated by clock configuration functions.
 */
extern uint32_t SystemCoreClock;

/**
 * @brief  AHB and APB prescalers lookup tables (values represent right-shift counts)
 * @note   Standard CMSIS tables used to calculate HCLK, PCLK1, and PCLK2 frequencies from RCC->CFGR.
 */
extern const uint8_t AHBPrescTable[16];
extern const uint8_t APBPrescTable[8];

/* Choose clock source for PLL: HSE or HSI */
#define USE_HSE_CLOCK

/* Default external crystal oscillator frequency (25MHz for BlackPill) */
#define HSE_VALUE 25000000UL

/* Default internal high-speed RC oscillator frequency (16MHz) */
#define HSI_VALUE 16000000UL

/**
 * @brief  Low-level hardware system initialization.
 *         Prepares CPU core and fundamental peripherals before application entry.
 *
 * @param  None
 * @return None
 *
 * @note   Called in Reset_Handler (startup) before calling main().
 *         Executes: enable_FPU(), enable_external_clock(), and delay_init().
 */
void SystemInit(void);

/**
 * @brief  Initialize Timer 4 (TIM4) as the dedicated timebase for microsecond/millisecond delay.
 *
 * @param  None
 * @return None
 *
 * @note   Configures TIM4 prescaler to achieve 1 MHz counter frequency (1 count = 1 us).
 *         Requires SystemCoreClock to be correctly configured before invocation.
 */
void delay_init(void);

/**
 * @brief  Blocking delay in milliseconds.
 *
 * @param  ms  Number of milliseconds to delay (0 to 65535).
 * @return None
 *
 * @note   Internally calls delay_us(1000) repeatedly to conserve timer resources.
 */
void delay(uint16_t ms);

/**
 * @brief  Blocking delay in microseconds.
 *
 * @param  us  Number of microseconds to delay (0 to 65535).
 * @return None
 *
 * @note   Configures TIM4 auto-reload register and waits for the update flag (UIF).
 *         Accurate for small delays without software loop calibration.
 */
void delay_us(uint16_t us);

/**
 * @brief  Switch the system clock (SYSCLK) source to the external crystal oscillator (HSE).
 *
 * @param  None
 * @return None
 *
 * @note   Enables HSE, polls HSERDY until stable, switches SW bits in RCC_CFGR,
 *         and updates SystemCoreClock to HSE_VALUE.
 */
void enable_external_clock(void);

/**
 * @brief  Configure and activate the Phase-Locked Loop (PLL) as system clock.
 *
 * @param  None
 * @return None
 *
 * @note   Sets up Flash latency wait states (2 WS for 84MHz on STM32F401),
 *         configures PLL M/N/P factors, bus prescalers (AHB=/1, APB1=/2, APB2=/1),
 *         and updates SystemCoreClock to 84 MHz.
 */
void enable_PLL_clock(void);

/**
 * @brief  Put the ARM Cortex-M4 CPU core into low-power Sleep mode.
 *
 * @param  None
 * @return None
 *
 * @note   Clears SLEEPDEEP and SLEEPONEXIT in SCB->SCR, issues __DSB() barrier,
 *         and executes the __WFI() (Wait For Interrupt) instruction.
 */
void enter_sleep_mode(void);

/**
 * @brief  Enable the Floating Point Unit (FPU) hardware on ARM Cortex-M4.
 *
 * @param  None
 * @return None
 *
 * @note   Grants Full Access to Coprocessors CP10 and CP11 (SCB->CPACR bits [23:20] = 0b1111).
 *         Executes __DSB() and __ISB() instructions to synchronize memory and instruction pipeline.
 */
void enable_FPU(void);

void enable_DWT(void);

uint32_t System_Get_Tick();

#endif /* UTILS_H_ */
