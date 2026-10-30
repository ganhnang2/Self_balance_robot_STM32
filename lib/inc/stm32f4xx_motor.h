/**
 * @file    stm32f4xx_motor.h
 * @brief   TB6612FNG Motor Driver + JGA25-370 Encoder Interface for STM32F4xx.
 *
 * @details
 *
 * - PIN CONFIGURATION (Hardcoded):
 * 1. TB6612FNG:
 * 	  - AIN1: PB13, AIN2: PB12, PWMA: PB4 	-> TIM3_CH1 (Right motor)
 * 	  - BIN1: PB14, BIN2: PB15, PWMB: PB5	-> TIM3_CH2 (Left motor)
 *    - STBY: PA12
 *    - AO1: DC1 (RED), AO2: DC2 (WHITE) - Right motor
 *    - BO1: DC1 (RED), BO2: DC2 (WHITE) - Left motor
 * 2. Encoder JGA25-370 (Quadrature, x4 mode)
 * - ENC1_A (YELLOW): PA8, ENC1_B (GREEN): PA9 		-> TIM1_CH1 / TIM1_CH2 (Left motor)
 * - ENC2_A (YELLOW): PB7, ENC2_B (GREEN): PB6 		-> TIM4_CH1 / TIM4_CH2 (Right motor)
 */
#ifndef STM32F4XX_MOTOR_H_
#define STM32F4XX_MOTOR_H_

#include "stm32f4xx.h"
#include <stdint.h>

/* =========================================================================
 * PWM Configuration
 * ========================================================================= */
#define PWM_FREQ            12500UL    	/*!< PWM frequency: 12.5 kHz 	*/
#define PWM_PRES_VAL        0UL         /*!< TIM3 prescaler value    	*/

/* =========================================================================
 * Encoder Physical Parameters — JGA25-370
 * ========================================================================= */
#define MOTOR_PULSES_PER_REV    	11U     /*!< Raw encoder lines per motor rev    */
#define MOTOR_X4_MULTIPLIER      	4U      /*!< Quadrature x4 counting             */
#define MOTOR_GEAR_RATIO        	21.3f   /*!< Gearbox ratio (e.g. 21.3 for 1:21.3) */
#define WHEEL_DIAMETER_MM       	65.0f   /*!< Wheel outer diameter in mm         */

/** Derived: total encoder counts for one full wheel revolution */
#define ENCODER_COUNTS_PER_WHEEL_REV \
    ((float)(MOTOR_PULSES_PER_REV * MOTOR_X4_MULTIPLIER) * MOTOR_GEAR_RATIO)

/** Derived: linear travel distance per single encoder count (mm) */
#define ENCODER_MM_PER_COUNT \
    ((3.14159265f * WHEEL_DIAMETER_MM) / ENCODER_COUNTS_PER_WHEEL_REV)

/* =========================================================================
 * Encoder State Structure
 * ========================================================================= */
typedef struct
{
    uint16_t prevCnt;       /*!< CNT register snapshot from the last update    */
	uint16_t currCnt;		/*!< CNT register snapshot from the current update    */
    int32_t  deltaCounts;   /*!< Signed pulse count since last Motor_Encoder_Update() */
} Encoder_t;

/* =========================================================================
 * Motor Control API
 * ========================================================================= */

/**
 * @brief  Initialise TB6612FNG driver pins and TIM3 PWM output.
 *         Must be called before TB6612_Control_Motors().
 */
void TB6612_Init(void);

/**
 * @brief  Set motor speeds.
 * @param  leftSpeed   Signed PWM compare value for left  motor (negative = reverse).
 * @param  rightSpeed  Signed PWM compare value for right motor (negative = reverse).
 */
void TB6612_Control_Motors(int32_t leftSpeed, int32_t rightSpeed);

/**
 * @brief  Short-brake both motors (AIN/BIN both HIGH, PWM = 0).
 */
void TB6612_Short_Brake(void);

/**
 * @brief  Coast both motors (AIN/BIN both LOW, PWM = 0).
 */
void TB6612_Coast(void);

/* =========================================================================
 * Encoder API
 * ========================================================================= */

/**
 * @brief  Initialise GPIO pins (PA8/PA9 for TIM1, PB6/PB7 for TIM4) and
 *         configure TIM1 / TIM4 in Encoder Interface mode (x4 quadrature).
 *         Must be called before Motor_Encoder_Update().
 */
void Motor_Encoder_Init(void);

/**
 * @brief  Read both encoder CNT registers and update internal Encoder_t state
 *         (deltaCounts, totalCounts) using wrap-around subtraction.
 *         Call this at a fixed sampling interval (e.g. every 10 ms in a timer ISR).
 * @note
 * ENCODER READING TECHNIQUE:
 * 	 - Uses "Subtraction Wrap-Around" to correctly handle all timer overflow
 *   and underflow cases without any conditional branching:
 *     delta = (int16_t)(uint16_t)(current_CNT - previous_CNT)
 *   - This works because unsigned 16-bit subtraction wraps in [0, 65535],
 *   and re-interpreting as int16_t maps values > 32767 to negative numbers,
 *   which correctly represents backward motion.
 */
void Motor_Encoder_Update(void);

/**
 * @brief  Reset cumulative counters (totalCounts) on both encoders and
 *         re-sync prevCnt to the current CNT value.
 *         Does NOT stop the hardware timers.
 */
void Motor_Encoder_Reset(void);

/**
 * @brief  Return the signed delta pulse count of the LEFT wheel since the
 *         last Motor_Encoder_Update() call.
 *         Positive = forward, Negative = backward.
 * @return int32_t  delta counts (range: -32768 .. +32767 per sampling period)
 */
int32_t Motor_Get_Encoder_Left(void);

/**
 * @brief  Return the signed delta pulse count of the RIGHT wheel since the
 *         last Motor_Encoder_Update() call.
 *         Positive = forward, Negative = backward.
 * @return int32_t  delta counts (range: -32768 .. +32767 per sampling period)
 */
int32_t Motor_Get_Encoder_Right(void);

/**
 * @brief  Return the current raw CNT register value of the LEFT wheel encoder (TIM1).
 *         Reflects the absolute position of the timer counter in the range [0, 65535].
 *         Resets to 0 only when Motor_Encoder_Reset() is called.
 * @note   This value wraps around at 65535 → 0 (overflow) or 0 → 65535 (underflow)
 *         and does NOT accumulate across wrap events. Use deltaCounts for motion tracking.
 * @return uint32_t  Current TIM1->CNT snapshot captured at the last Motor_Encoder_Update().
 */
uint32_t Motor_Get_Encoder_Counter_Left(void);

/**
 * @brief  Return the current raw CNT register value of the RIGHT wheel encoder (TIM4).
 *         Reflects the absolute position of the timer counter in the range [0, 65535].
 *         Resets to 0 only when Motor_Encoder_Reset() is called.
 * @note   This value wraps around at 65535 → 0 (overflow) or 0 → 65535 (underflow)
 *         and does NOT accumulate across wrap events. Use deltaCounts for motion tracking.
 * @return uint32_t  Current TIM4->CNT snapshot captured at the last Motor_Encoder_Update().
 */
uint32_t Motor_Get_Encoder_Counter_Right(void);

#endif /* STM32F4XX_MOTOR_H_ */