/**
 * @file    stm32f4xx_pid.h
 * @author  Nguyen Hai Nam
 * @brief   Header file for PID Controller implementation.
 *
 * @note    Designed for closed-loop balance and velocity control of
 *          Self-Balancing Robot. Supports anti-windup clamping and output saturation.
 */

#ifndef STM32F4XX_PID_H_
#define STM32F4XX_PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h> 
#include "stm32f4xx.h"

/**
 * @brief PID Controller State and Parameter Structure
 * @note  Holds gain coefficients, state accumulators, saturation limits, and sampling time.
 */
typedef struct {
	float kp;          			/*!< Proportional gain coefficient: Kp */
	float ki;          			/*!< Integral gain coefficient: Ki */
	float kd;          			/*!< Derivative gain coefficient: Kd */
	float integral;    			/*!< Accumulated integral term */
	float prevError;   			/*!< Previous error value for derivative calculation */
	float prevMeasurement;   	/*!< Previous measurement value for derivative calculation */
	float dTerm;				/*!< The value of deriavtive part through Low-pass filter */	
	float maxIntegral; 			/*!< Maximum limit for integral accumulator (Anti-windup limit) */
	float maxOutput;   			/*!< Maximum absolute limit for control output */
	float dt;         	 		/*!< Sampling period in seconds */
} PID_t;

/**
 * @brief  Initialize the PID controller instance with default settings.
 * @param  pid: Pointer to PID_t controller structure.
 * @return None
 * @note   Initializes gains to zero, resets integral accumulator and previous error,
 *         max output and max integral and sets default sampling time dt = 0.02s (50 Hz).
 */
void PID_Init(PID_t* pid);

/**
 * @brief  Compute the PID control output based on current setpoint and measurement.
 * @param  pid: Pointer to PID_t controller structure.
 * @param  setpoint: Target desired reference value.
 * @param  measured_value: Current measured process value from sensors.
 * @return Computed control output clamped to [-maxOutput, +maxOutput].
 * @note   Applies anti-windup clamping on the integral term to [-maxIntegral, +maxIntegral].
 *         Uses backward difference for derivative calculation.
 */
float PID_Compute(PID_t* pid, float setpoint, float measured_value, float dt);

float PID_Compute_Advanced(PID_t* pid, float setpoint, float measurement, float dt);

/**
 * @brief  Configure or update the PID controller gain coefficients.
 * @param  pid: Pointer to PID_t controller structure.
 * @param  kp: Proportional gain coefficient.
 * @param  ki: Integral gain coefficient.
 * @param  kd: Derivative gain coefficient.
 * @return None
 * @note   Can be invoked at runtime for online PID tuning or parameter scheduling.
 */
void PID_Set_Parameters(PID_t* pid, float kp, float ki, float kd);

/**
 * @brief  Set the sampling time period (dt) for the PID calculation.
 * @param  pid: Pointer to PID_t controller structure.
 * @param  dt: Sampling time period in seconds (must be positive).
 * @return None
 * @note   Must match the actual loop/timer invocation period (e.g., 0.02s for 50 Hz).
 */
void PID_Set_Sampling_Time(PID_t* pid, float dt);

void PID_Set_Max_Output(PID_t* pid, float maxOutput);

void PID_Set_Max_Integral(PID_t* pid, float maxIntegral);

void PID_Reset_State(PID_t* pid);

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_PID_H_ */