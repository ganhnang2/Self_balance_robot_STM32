/**
 * @file    nvic_config.c
 * @author  Nguyen Hai Nam
 * @brief   Implementation of Centralized NVIC Interrupt Configuration.
 *          Applies strict priority levels to ensure hard real-time PID loops
 *          are never blocked or delayed by communication tasks.
 */

#include "nvic_config.h"

void NVIC_Config_PriorityGrouping(void)
{
    /* Set Priority Grouping: 4 bits Preemption, 0 bits Subpriority */
    NVIC_SetPriorityGrouping(NVIC_DEFAULT_PRIORITY_GROUP);
}

void NVIC_Config_ControlLoop(IRQn_Type timerIrq)
{
    /* Level 0: Highest priority for PID balancing control loop */
    NVIC_SetPriority(timerIrq, NVIC_PRIO_CONTROL_LOOP);
    NVIC_EnableIRQ(timerIrq);
}

void NVIC_Config_HC05(void)
{	
    /* Level 1: Medium priority for Bluetooth command parsing */
    NVIC_SetPriority(DMA1_Stream6_IRQn, NVIC_PRIO_HC05_BLUETOOTH_RX);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	
	/* Level 2: Medium priority for Bluetooth IMU Frame sending */
	NVIC_SetPriority(USART2_IRQn, NVIC_PRIO_HC05_BLUETOOTH_TX);
    NVIC_EnableIRQ(USART2_IRQn);
}

void NVIC_Config_OLED_DMA(void)
{
    /* Level 3: Low priority for OLED framebuffer DMA complete interrupt */
    NVIC_SetPriority(DMA1_Stream7_IRQn, NVIC_PRIO_OLED_DMA);
    NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}

void NVIC_Config_Debug(IRQn_Type uartIrq)
{
    /* Level 3: Lowest active priority for serial printf debug */
    NVIC_SetPriority(uartIrq, NVIC_PRIO_DEBUG_CONSOLE);
    NVIC_EnableIRQ(uartIrq);
}

void NVIC_Config_All(void)
{
    /* 1. Setup priority grouping */
    NVIC_Config_PriorityGrouping();

    /* 2. Setup HC-05 Bluetooth interrupt (USART2) */
    NVIC_Config_HC05();

    /* 3. Setup default Robot Balancing Loop Timer (TIM2) */
    NVIC_Config_ControlLoop(TIM2_IRQn);

    /* 4. Setup OLED Framebuffer DMA Interrupt (DMA1 Stream 7) */
    NVIC_Config_OLED_DMA();
}
