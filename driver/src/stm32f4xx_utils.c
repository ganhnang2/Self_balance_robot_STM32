#include "stm32f4xx_utils.h"

uint32_t SystemCoreClock = HSI_VALUE;

const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};
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
void SystemInit(void)
{
    // 1. Enable floating-point unit 
    enable_FPU();
	
    // 2. Set system clock as external clock HSE (or PLL)
    enable_external_clock();
    // enable_PLL_clock();
	
	// 3. Data Watchpoint and Trace
	enable_DWT();
	
    // 4. Initialize SysTick to use delay, delay_us function
    delay_init();
}

/**
 * @brief  Initialize SysTick (System_Timer) as the dedicated timebase for microsecond/millisecond delay.
 *
 * @param  None
 * @return None
 *
 * @note   Configures SysTick clock source to processor clock (AHB).
 *         Requires SystemCoreClock to be correctly configured before invocation.
 */
void delay_init(void)
{
	// Set SysTick clock source to processor clock (AHB)
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
	
	// Disable SysTick during setup
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

/**
 * @brief  Blocking delay in microseconds using SysTick.
 *
 * @param  us  Number of microseconds to delay (0 to 65535).
 * @return None
 *
 * @note   Configures SysTick load register and waits for the COUNTFLAG.
 *         Accurate for small delays without software loop calibration.
 */
void delay_us(uint16_t us)
{
	if (us == 0) 
		return;
	
	// Load the number of ticks required for 'us' microseconds
	SysTick->LOAD = (SystemCoreClock / 1000000) * us - 1;
	
	// Clear the current value register and the COUNTFLAG
	SysTick->VAL = 0;
	
	// Enable SysTick
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	
	// Wait until the COUNTFLAG is set (indicates counter reached 0)
	while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
	
	// Disable SysTick
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

/**
 * @brief  Blocking delay in milliseconds.
 *
 * @param  ms  Number of milliseconds to delay (0 to 65535).
 * @return None
 *
 * @note   Internally calls delay_us(1000) repeatedly to conserve timer resources.
 */
void delay(uint16_t ms)
{
	while (ms > 0)
	{
		delay_us(1000);
		ms--;
	}
}

/**
 * @brief  Switch the system clock (SYSCLK) source to the external crystal oscillator (HSE).
 *
 * @param  None
 * @return None
 *
 * @note   Enables HSE, polls HSERDY until stable, switches SW bits in RCC_CFGR,
 *         and updates SystemCoreClock to HSE_VALUE.
 */
void enable_external_clock(void)
{
	// Enable external clock
	RCC->CR |= RCC_CR_HSEON;
	
	// Wait until HSE clock is ready 
	while(!(RCC->CR & RCC_CR_HSERDY));
	
	// System clock switch to external clock
	RCC->CFGR |= RCC_CFGR_SW_HSE;
	
	// Wait until system clock is ready
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE);
	
	// Update the system clock frequency using HSE_VALUE
	SystemCoreClock = HSE_VALUE;
}

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
void enable_PLL_clock(void)
{
    // Bật bộ đệm Prefetch và cấu hình Flash Latency cho 84MHz (cần 2 Wait states)
    FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_2WS;

#ifdef USE_HSE_CLOCK
    // 1. Dùng HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    
    // 2. Cấu hình PLL: Nguồn HSE, hệ số chia M, N, P
    // M = (HSE_VALUE / 1000000) / 2 => Chia để VCO input = 2MHz
    uint32_t pll_m = HSE_VALUE / 2000000UL;
    uint32_t pll_n = 84; // Nhân lên 168MHz
    uint32_t pll_p = 0;  // 0 <=> Chia 2 => SYSCLK = 84MHz
    
    RCC->PLLCFGR = (pll_m << RCC_PLLCFGR_PLLM_Pos) | 
                   (pll_n << RCC_PLLCFGR_PLLN_Pos) | 
                   (pll_p << RCC_PLLCFGR_PLLP_Pos) | 
                   RCC_PLLCFGR_PLLSRC_HSE;
    
    SystemCoreClock = 84000000UL;
#else
    // 1. Dùng HSI (mặc định đã bật 16MHz)
    // 2. Cấu hình PLL: Nguồn HSI
    // M = 8 (VCO in = 2MHz), N = 84 (VCO = 168MHz), P = 2 (SYSCLK = 84MHz)
    uint32_t pll_m = 8;
    uint32_t pll_n = 84;
    uint32_t pll_p = 0; // 0 <=> Chia 2
    
    RCC->PLLCFGR = (pll_m << RCC_PLLCFGR_PLLM_Pos) | 
                   (pll_n << RCC_PLLCFGR_PLLN_Pos) | 
                   (pll_p << RCC_PLLCFGR_PLLP_Pos) | 
                   RCC_PLLCFGR_PLLSRC_HSI;
                   
    SystemCoreClock = 84000000UL;
#endif

    // 3. Bật PLL và chờ ổn định
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // 4. Cấu hình Prescaler cho các Bus 
    // AHB = /1 (84MHz), APB1 = /2 (42MHz, vì max của APB1 là 42MHz), APB2 = /1 (84MHz)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    RCC->CFGR &= ~RCC_CFGR_PPRE2; // Đảm bảo APB2 không bị chia

    // 5. Chuyển System Clock sang PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

/**
 * @brief  Put the ARM Cortex-M4 CPU core into low-power Sleep mode.
 *
 * @param  None
 * @return None
 *
 * @note   Clears SLEEPDEEP and SLEEPONEXIT in SCB->SCR, issues __DSB() barrier,
 *         and executes the __WFI() (Wait For Interrupt) instruction.
 */
void enter_sleep_mode(void)
{
	// Cortex system control register
	// 0: sleep
	// 1: deep sleep (Depending on bit PDDS in PWR_CR, it will be Standby or Stop mode)
	SCB->SCR &= ~(1 << SCB_SCR_SLEEPDEEP_Pos);			

	// SLEEPONEXIT: Indicates sleep-on-exit when returning from handler
	// mode to thread mode:
	// 0: do not sleep when returning to thread mode. 
	// 1: enter sleep, or deep sleep, on return from an ISR.
	SCB->SCR &= ~(1 << SCB_SCR_SLEEPONEXIT_Pos); 

	__DSB (); 	// Ensure that the Last store takes effect 
	__WFI (); 	// Switch processor into the sleep mode 
}

/**
 * @brief  Enable the Floating Point Unit (FPU) hardware on ARM Cortex-M4.
 *
 * @param  None
 * @return None
 *
 * @note   Grants Full Access mode to Coprocessor CP10 and CP11
 *         (SCB->CPACR bits [23:20] = 0b1111).
 *         Executes __DSB() and __ISB() to synchronize the pipeline.
 */
void enable_FPU(void)
{
	// Grant Full Access (bits = 0b11) to Coprocessor CP10 and CP11
	SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));

	__DSB(); 	// Data Synchronization Barrier: Ensure completion of CPACR write
	__ISB(); 	// Instruction Synchronization Barrier: Flush instruction pipeline
}

// Bật bộ đếm chu kỳ CPU trong Cortex-M4
void enable_DWT(void)
{
	// 1. Cho phép khối Trace hoạt động (bắt buộc trước khi đụng vào DWT)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	
	// 2. Đặt lại giá trị ban đầu của bộ đếm về 0
    DWT->CYCCNT = 0;
	
	// 3. Kích hoạt bộ đếm chu kỳ
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            
}

uint32_t System_Get_Tick()
{
	return (uint32_t)DWT->CYCCNT;
}
