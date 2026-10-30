#include <stdint.h>

/*
 * Startup file for STM32F401CCU6
 * Adapted from startup_stm32f407.c
 * Author: Nguyen Hai Nam
 *
 * STM32F401CCU6 Memory Map:
 *   Flash : 0x08000000 - 0x0803FFFF (256KB)
 *   SRAM  : 0x20000000 - 0x2000FFFF (64KB)
 */

#define SRAM_START  0x20000000U
#define SRAM_SIZE   (64U * 1024U)   /* STM32F401CCU6: 64KB SRAM */
#define SRAM_END    ((SRAM_START) + (SRAM_SIZE))
 
#define STACK_START  SRAM_END

extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

extern int main(void);
extern void SystemInit(void);
extern void __libc_init_array(void);

void Reset_Handler(void);
void Default_Handler(void);

/* ---------------------------------------------------------------------------
 * ARM Cortex-M4 Core Exception Handlers
 * --------------------------------------------------------------------------- */
void NMI_Handler(void)          __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)    __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)    __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)          __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)       __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)      __attribute__((weak, alias("Default_Handler")));

/* ---------------------------------------------------------------------------
 * STM32F401CCU6 Peripheral Interrupt Handlers (IRQ0 .. IRQ84)
 * Reference: RM0368 Rev 5, Table 38 - Vector table for STM32F401xB/C/D/E
 * --------------------------------------------------------------------------- */
void WWDG_IRQHandler(void)                    __attribute__((weak, alias("Default_Handler")));  /* IRQ0  */
void PVD_IRQHandler(void)                     __attribute__((weak, alias("Default_Handler")));  /* IRQ1  */
void TAMP_STAMP_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));  /* IRQ2  */
void RTC_WKUP_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));  /* IRQ3  */
void FLASH_IRQHandler(void)                   __attribute__((weak, alias("Default_Handler")));  /* IRQ4  */
void RCC_IRQHandler(void)                     __attribute__((weak, alias("Default_Handler")));  /* IRQ5  */
void EXTI0_IRQHandler(void)                   __attribute__((weak, alias("Default_Handler")));  /* IRQ6  */
void EXTI1_IRQHandler(void)                   __attribute__((weak, alias("Default_Handler")));  /* IRQ7  */
void EXTI2_IRQHandler(void)                   __attribute__((weak, alias("Default_Handler")));  /* IRQ8  */
void EXTI3_IRQHandler(void)                   __attribute__((weak, alias("Default_Handler")));  /* IRQ9  */
void EXTI4_IRQHandler(void)                   __attribute__((weak, alias("Default_Handler")));  /* IRQ10 */
void DMA1_Stream0_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* IRQ11 */
void DMA1_Stream1_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* IRQ12 */
void DMA1_Stream2_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* IRQ13 */
void DMA1_Stream3_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* IRQ14 */
void DMA1_Stream4_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* IRQ15 */
void DMA1_Stream5_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* IRQ16 */
void DMA1_Stream6_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* IRQ17 */
void ADC_IRQHandler(void)                     __attribute__((weak, alias("Default_Handler")));  /* IRQ18 */
/* IRQ19..IRQ22: Reserved - no CAN peripheral on F401CC */
void EXTI9_5_IRQHandler(void)                 __attribute__((weak, alias("Default_Handler"))); 	/* IRQ23 */
void TIM1_BRK_TIM9_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* IRQ24 */
void TIM1_UP_TIM10_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* IRQ25 */
void TIM1_TRG_COM_TIM11_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));  /* IRQ26 */
void TIM1_CC_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));  /* IRQ27 */
void TIM2_IRQHandler(void)                   __attribute__((weak, alias("Default_Handler")));  /* IRQ28 */
void TIM3_IRQHandler(void)                   __attribute__((weak, alias("Default_Handler")));  /* IRQ29 */
void TIM4_IRQHandler(void)                   __attribute__((weak, alias("Default_Handler")));  /* IRQ30 */
void I2C1_EV_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));  /* IRQ31 */
void I2C1_ER_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));  /* IRQ32 */
void I2C2_EV_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));  /* IRQ33 */
void I2C2_ER_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));  /* IRQ34 */
void SPI1_IRQHandler(void)                   __attribute__((weak, alias("Default_Handler")));  /* IRQ35 */
void SPI2_IRQHandler(void)                   __attribute__((weak, alias("Default_Handler")));  /* IRQ36 */
void USART1_IRQHandler(void)                 __attribute__((weak, alias("Default_Handler")));  /* IRQ37 */
void USART2_IRQHandler(void)                 __attribute__((weak, alias("Default_Handler")));  /* IRQ38 */
/* IRQ39: Reserved - no USART3 on F401CC */
void EXTI15_10_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));  /* IRQ40 */
void RTC_Alarm_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));  /* IRQ41 */
void OTG_FS_WKUP_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* IRQ42 */
/* IRQ43..IRQ46: Reserved - no TIM8/TIM12-14 on F401CC */
void DMA1_Stream7_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* IRQ47 */
/* IRQ48: Reserved - no FSMC on F401CC */
void SDIO_IRQHandler(void)                   __attribute__((weak, alias("Default_Handler")));  /* IRQ49 */
void TIM5_IRQHandler(void)                   __attribute__((weak, alias("Default_Handler")));  /* IRQ50 */
void SPI3_IRQHandler(void)                   __attribute__((weak, alias("Default_Handler")));  /* IRQ51 */
/* IRQ52..IRQ55: Reserved - no UART4/UART5/TIM6_DAC/TIM7 on F401CC */
void DMA2_Stream0_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* IRQ56 */
void DMA2_Stream1_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* IRQ57 */
void DMA2_Stream2_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* IRQ58 */
void DMA2_Stream3_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* IRQ59 */
void DMA2_Stream4_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* IRQ60 */
/* IRQ61..IRQ66: Reserved - no ETH/CAN2 on F401CC */
void OTG_FS_IRQHandler(void)                 __attribute__((weak, alias("Default_Handler")));  /* IRQ67 */
void DMA2_Stream5_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* IRQ68 */
void DMA2_Stream6_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* IRQ69 */
void DMA2_Stream7_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* IRQ70 */
void USART6_IRQHandler(void)                 __attribute__((weak, alias("Default_Handler")));  /* IRQ71 */
void I2C3_EV_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));  /* IRQ72 */
void I2C3_ER_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));  /* IRQ73 */
/* IRQ74..IRQ80: Reserved - no OTG_HS/DCMI/CRYP/HASH on F401CC */
void FPU_IRQHandler(void)                    __attribute__((weak, alias("Default_Handler")));  /* IRQ81 */
/* IRQ82..IRQ83: Reserved */
void SPI4_IRQHandler(void)                   __attribute__((weak, alias("Default_Handler")));  /* IRQ84 */

/* ---------------------------------------------------------------------------
 * Interrupt Vector Table
 * Placed in .isr_vector section (mapped to start of Flash by linker script)
 * Reference: RM0368 Rev 5, Table 38
 * --------------------------------------------------------------------------- */
uint32_t vectors[] __attribute__((section(".isr_vector"))) = {
    /* Initial Stack Pointer */
    STACK_START,

    /* ARM Cortex-M4 Core Exceptions */
    (uint32_t)Reset_Handler,                    /* Reset Handler                      */
    (uint32_t)NMI_Handler,                      /* NMI Handler                        */
    (uint32_t)HardFault_Handler,                /* Hard Fault Handler                 */
    (uint32_t)MemManage_Handler,                /* MPU Fault Handler                  */
    (uint32_t)BusFault_Handler,                 /* Bus Fault Handler                  */
    (uint32_t)UsageFault_Handler,               /* Usage Fault Handler                */
    0,                                           /* Reserved                           */
    0,                                           /* Reserved                           */
    0,                                           /* Reserved                           */
    0,                                           /* Reserved                           */
    (uint32_t)SVC_Handler,                      /* SVCall Handler                     */
    (uint32_t)DebugMon_Handler,                 /* Debug Monitor Handler              */
    0,                                           /* Reserved                           */
    (uint32_t)PendSV_Handler,                   /* PendSV Handler                     */
    (uint32_t)SysTick_Handler,                  /* SysTick Handler                    */

    /* External Interrupts */
    (uint32_t)WWDG_IRQHandler,                  /* IRQ0:  Window WatchDog             */
    (uint32_t)PVD_IRQHandler,                   /* IRQ1:  PVD via EXTI Line           */
    (uint32_t)TAMP_STAMP_IRQHandler,            /* IRQ2:  Tamper and TimeStamp        */
    (uint32_t)RTC_WKUP_IRQHandler,              /* IRQ3:  RTC Wakeup                  */
    (uint32_t)FLASH_IRQHandler,                 /* IRQ4:  Flash global                */
    (uint32_t)RCC_IRQHandler,                   /* IRQ5:  RCC global                  */
    (uint32_t)EXTI0_IRQHandler,                 /* IRQ6:  EXTI Line 0                 */
    (uint32_t)EXTI1_IRQHandler,                 /* IRQ7:  EXTI Line 1                 */
    (uint32_t)EXTI2_IRQHandler,                 /* IRQ8:  EXTI Line 2                 */
    (uint32_t)EXTI3_IRQHandler,                 /* IRQ9:  EXTI Line 3                 */
    (uint32_t)EXTI4_IRQHandler,                 /* IRQ10: EXTI Line 4                 */
    (uint32_t)DMA1_Stream0_IRQHandler,          /* IRQ11: DMA1 Stream 0               */
    (uint32_t)DMA1_Stream1_IRQHandler,          /* IRQ12: DMA1 Stream 1               */
    (uint32_t)DMA1_Stream2_IRQHandler,          /* IRQ13: DMA1 Stream 2               */
    (uint32_t)DMA1_Stream3_IRQHandler,          /* IRQ14: DMA1 Stream 3               */
    (uint32_t)DMA1_Stream4_IRQHandler,          /* IRQ15: DMA1 Stream 4               */
    (uint32_t)DMA1_Stream5_IRQHandler,          /* IRQ16: DMA1 Stream 5               */
    (uint32_t)DMA1_Stream6_IRQHandler,          /* IRQ17: DMA1 Stream 6               */
    (uint32_t)ADC_IRQHandler,                   /* IRQ18: ADC1                        */
    0,                                           /* IRQ19: Reserved (no CAN1_TX)       */
    0,                                           /* IRQ20: Reserved (no CAN1_RX0)      */
    0,                                           /* IRQ21: Reserved (no CAN1_RX1)      */
    0,                                           /* IRQ22: Reserved (no CAN1_SCE)      */
    (uint32_t)EXTI9_5_IRQHandler,               /* IRQ23: EXTI Lines 9..5             */
    (uint32_t)TIM1_BRK_TIM9_IRQHandler,         /* IRQ24: TIM1 Break + TIM9           */
    (uint32_t)TIM1_UP_TIM10_IRQHandler,         /* IRQ25: TIM1 Update + TIM10         */
    (uint32_t)TIM1_TRG_COM_TIM11_IRQHandler,    /* IRQ26: TIM1 Trig/Com + TIM11       */
    (uint32_t)TIM1_CC_IRQHandler,               /* IRQ27: TIM1 Capture Compare        */
    (uint32_t)TIM2_IRQHandler,                  /* IRQ28: TIM2                        */
    (uint32_t)TIM3_IRQHandler,                  /* IRQ29: TIM3                        */
    (uint32_t)TIM4_IRQHandler,                  /* IRQ30: TIM4                        */
    (uint32_t)I2C1_EV_IRQHandler,               /* IRQ31: I2C1 Event                  */
    (uint32_t)I2C1_ER_IRQHandler,               /* IRQ32: I2C1 Error                  */
    (uint32_t)I2C2_EV_IRQHandler,               /* IRQ33: I2C2 Event                  */
    (uint32_t)I2C2_ER_IRQHandler,               /* IRQ34: I2C2 Error                  */
    (uint32_t)SPI1_IRQHandler,                  /* IRQ35: SPI1                        */
    (uint32_t)SPI2_IRQHandler,                  /* IRQ36: SPI2                        */
    (uint32_t)USART1_IRQHandler,                /* IRQ37: USART1                      */
    (uint32_t)USART2_IRQHandler,                /* IRQ38: USART2                      */
    0,                                           /* IRQ39: Reserved (no USART3)        */
    (uint32_t)EXTI15_10_IRQHandler,             /* IRQ40: EXTI Lines 15..10           */
    (uint32_t)RTC_Alarm_IRQHandler,             /* IRQ41: RTC Alarm A and B           */
    (uint32_t)OTG_FS_WKUP_IRQHandler,           /* IRQ42: USB OTG FS Wakeup           */
    0,                                           /* IRQ43: Reserved (no TIM8_BRK_TIM12) */
    0,                                           /* IRQ44: Reserved (no TIM8_UP_TIM13)  */
    0,                                           /* IRQ45: Reserved (no TIM8_TRG_COM_TIM14) */
    0,                                           /* IRQ46: Reserved (no TIM8_CC)       */
    (uint32_t)DMA1_Stream7_IRQHandler,          /* IRQ47: DMA1 Stream 7               */
    0,                                           /* IRQ48: Reserved (no FSMC)          */
    (uint32_t)SDIO_IRQHandler,                  /* IRQ49: SDIO                        */
    (uint32_t)TIM5_IRQHandler,                  /* IRQ50: TIM5                        */
    (uint32_t)SPI3_IRQHandler,                  /* IRQ51: SPI3                        */
    0,                                           /* IRQ52: Reserved (no UART4)         */
    0,                                           /* IRQ53: Reserved (no UART5)         */
    0,                                           /* IRQ54: Reserved (no TIM6_DAC)      */
    0,                                           /* IRQ55: Reserved (no TIM7)          */
    (uint32_t)DMA2_Stream0_IRQHandler,          /* IRQ56: DMA2 Stream 0               */
    (uint32_t)DMA2_Stream1_IRQHandler,          /* IRQ57: DMA2 Stream 1               */
    (uint32_t)DMA2_Stream2_IRQHandler,          /* IRQ58: DMA2 Stream 2               */
    (uint32_t)DMA2_Stream3_IRQHandler,          /* IRQ59: DMA2 Stream 3               */
    (uint32_t)DMA2_Stream4_IRQHandler,          /* IRQ60: DMA2 Stream 4               */
    0,                                           /* IRQ61: Reserved (no ETH)           */
    0,                                           /* IRQ62: Reserved (no ETH_WKUP)      */
    0,                                           /* IRQ63: Reserved (no CAN2_TX)       */
    0,                                           /* IRQ64: Reserved (no CAN2_RX0)      */
    0,                                           /* IRQ65: Reserved (no CAN2_RX1)      */
    0,                                           /* IRQ66: Reserved (no CAN2_SCE)      */
    (uint32_t)OTG_FS_IRQHandler,                /* IRQ67: USB OTG FS                  */
    (uint32_t)DMA2_Stream5_IRQHandler,          /* IRQ68: DMA2 Stream 5               */
    (uint32_t)DMA2_Stream6_IRQHandler,          /* IRQ69: DMA2 Stream 6               */
    (uint32_t)DMA2_Stream7_IRQHandler,          /* IRQ70: DMA2 Stream 7               */
    (uint32_t)USART6_IRQHandler,                /* IRQ71: USART6                      */
    (uint32_t)I2C3_EV_IRQHandler,               /* IRQ72: I2C3 Event                  */
    (uint32_t)I2C3_ER_IRQHandler,               /* IRQ73: I2C3 Error                  */
    0,                                           /* IRQ74: Reserved (no OTG_HS_EP1_OUT) */
    0,                                           /* IRQ75: Reserved (no OTG_HS_EP1_IN)  */
    0,                                           /* IRQ76: Reserved (no OTG_HS_WKUP)    */
    0,                                           /* IRQ77: Reserved (no OTG_HS)         */
    0,                                           /* IRQ78: Reserved (no DCMI)           */
    0,                                           /* IRQ79: Reserved (no CRYP)           */
    0,                                           /* IRQ80: Reserved (no HASH_RNG)       */
    (uint32_t)FPU_IRQHandler,                   /* IRQ81: FPU                         */
    0,                                           /* IRQ82: Reserved                    */
    0,                                           /* IRQ83: Reserved                    */
    (uint32_t)SPI4_IRQHandler,                  /* IRQ84: SPI4                        */
};

/* ---------------------------------------------------------------------------
 * Default Handler: infinite loop for unhandled interrupts
 * --------------------------------------------------------------------------- */
void Default_Handler(void)
{
    while (1);
}

/* ---------------------------------------------------------------------------
 * Reset Handler: initialize .data, zero .bss, then call main()
 * --------------------------------------------------------------------------- */
void Reset_Handler(void)
{
    /* Copy .data section from Flash (LMA) to SRAM (VMA) */
    uint32_t size = (uint32_t)&_edata - (uint32_t)&_sdata;
    uint8_t *pDst = (uint8_t *)&_sdata;    /* destination: SRAM            */
    uint8_t *pSrc = (uint8_t *)&_sidata;   /* source: Flash (load address) */
    for (uint32_t i = 0; i < size; i++) {
        *pDst++ = *pSrc++;
    }

    /* Zero-initialize .bss section in SRAM */
    size = (uint32_t)&_ebss - (uint32_t)&_sbss;
    pDst = (uint8_t *)&_sbss;
    for (uint32_t i = 0; i < size; i++) {
        *pDst++ = 0;
    }
	
	/* Initialize hardware configuration (Clock, FPU, Delay) */
	SystemInit();

    /* Initialize C standard library (newlib) */
    __libc_init_array();

    /* Call main application */
    main();

    /* Should never reach here */
    while (1);
}
