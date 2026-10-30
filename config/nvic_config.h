/**
 * @file    nvic_config.h
 * @author  Nguyen Hai Nam
 * @brief   Centralized NVIC Interrupt & Priority Configuration for Self-Balancing Robot.
 *          Defines priority levels, interrupt grouping, and initialization functions
 *          to ensure real-time stability for PID control and communication.
 */

#ifndef NVIC_CONFIG_H_
#define NVIC_CONFIG_H_

#include "stm32f4xx.h"
#include "core_cm4.h"

/* =========================================================================
 * 1. NVIC PRIORITY GROUPING DEFINITIONS
 * In Cortex-M4 with 4 priority bits (__NVIC_PRIO_BITS = 4):
 * Group 4: 4 bits preemption (0..15), 0 bits subpriority (RECOMMENDED)
 * ========================================================================= */
#define NVIC_GROUP_4_PREEMPT_0_SUB   3U  /*!< 4 bits preemption, 0 bits subpriority */
#define NVIC_GROUP_3_PREEMPT_1_SUB   4U  /*!< 3 bits preemption, 1 bit  subpriority */
#define NVIC_GROUP_2_PREEMPT_2_SUB   5U  /*!< 2 bits preemption, 2 bits subpriority */
#define NVIC_GROUP_1_PREEMPT_3_SUB   6U  /*!< 1 bit  preemption, 3 bits subpriority */
#define NVIC_GROUP_0_PREEMPT_4_SUB   7U  /*!< 0 bits preemption, 4 bits subpriority */

#define NVIC_DEFAULT_PRIORITY_GROUP  NVIC_GROUP_4_PREEMPT_0_SUB

/* =========================================================================
 * 2. INTERRUPT PRIORITY TABLE (0 = Highest Priority, 15 = Lowest Priority)
 * 
 * DESIGN RATIONALE FOR SELF-BALANCING ROBOT:
 * - LEVEL 0 (HIGHEST): PID Balancing Control Loop (Timer). Must NEVER be
 *   delayed or preempted by communication to prevent robot from falling!
 * - LEVEL 1: Wheel Encoder feedback / high-speed sensor readings.
 * - LEVEL 2: HC-05 Bluetooth commands (User direction & PID tuning).
 * - LEVEL 3: Debug printf / telemetry console (Lowest priority).
 * ========================================================================= */
#define NVIC_PRIO_CONTROL_LOOP       0U  	/*!< Control Loop (TIM2 / TIM5: 10ms PID loop) */
#define NVIC_PRIO_HC05_BLUETOOTH_RX  1U  	/*!< Bluetooth HC-05 (USART2 RX/IDLE interrupt) */
#define NVIC_PRIO_HC05_BLUETOOTH_TX  2U  	/*!< Bluetooth HC-05 (USART2 TX/DMA interrupt) */
#define NVIC_PRIO_OLED_DMA           3U  	/*!< OLED Display DMA (DMA1 Stream 7 transfer complete) */
#define NVIC_PRIO_DEBUG_CONSOLE      3U  	/*!< Serial Printf Debug (USART1 / USART6 / DMA) */

/* =========================================================================
 * API Function Prototypes
 * ========================================================================= */

/**
 * @brief  Configure the NVIC Priority Grouping (Default: 4 bits Preemption).
 */
void NVIC_Config_PriorityGrouping(void);

/**
 * @brief  Configure priority and enable interrupt for the Robot Control Loop Timer.
 * @param  timerIrq IRQn of the control timer (e.g., TIM2_IRQn or TIM5_IRQn).
 */
void NVIC_Config_ControlLoop(IRQn_Type timerIrq);

/**
 * @brief  Configure priority and enable interrupt for HC-05 Bluetooth (USART2).
 */
void NVIC_Config_HC05(void);

/**
 * @brief  Configure priority and enable interrupt for OLED Framebuffer DMA (DMA1 Stream 7).
 */
void NVIC_Config_OLED_DMA(void);

/**
 * @brief  Configure priority and enable interrupt for Debug Serial (USART1 or USART6).
 * @param  uartIrq IRQn of the debug USART (e.g., USART1_IRQn or USART6_IRQn).
 */
void NVIC_Config_Debug(IRQn_Type uartIrq);

/**
 * @brief  Initialize all standard interrupts and priority levels for the robot.
 */
void NVIC_Config_All(void);

#endif /* NVIC_CONFIG_H_ */
