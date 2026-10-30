/**
 * @file    stm32f4xx_pid.c
 * @author  Nguyen Hai Nam
 * @brief   PID Controller implementation file.
 *
 * @note    Contains initialization, computation, and tuning functions
 *          for discrete PID control with anti-windup and output saturation.
 */

#include "stm32f4xx_pid.h"

/**
 * @brief  Initialize the PID controller instance with default parameters.
 * @param  pid: Pointer to PID_t controller structure.
 * @return None
 * @note   Initializes gains to zero, resets integral accumulator and previous error,
 *         max output and max integral and sets default sampling time dt = 0.02s (50 Hz).
 */
void PID_Init(PID_t* pid)
{
	// Tuning parameters: Kp = Ki = Kd = 0.0f
	pid->kp = 0.0f;
	pid->ki = 0.0f;
	pid->kd = 0.0f;
	
	// Maximum limit of output
	pid->maxOutput = 0.0f;
	
	// State variables and anti-windup initialization
	pid->integral = 0.0f;
	pid->prevError = 0.0f;
	pid->maxIntegral = 0.0f;
	pid->prevMeasurement = 0.0f;
	
	// Sampling time: 0.005s (200Hz)
	pid->dt = 0.005f;
}

/**
 * @brief  Compute the PID control output based on error between setpoint and measured value.
 * @param  pid: Pointer to PID_t controller structure.
 * @param  setpoint: Target desired reference value.
 * @param  measured_value: Current measured value from sensor/feedback.
 * @return Computed control output value, saturated within [-maxOutput, +maxOutput].
 * @note   Computes proportional, integral, and derivative terms.
 *         Applies clamping anti-windup to integral accumulator ([-maxIntegral, +maxIntegral]).
 *         Derivative is computed via backward finite difference divided by dt.
 */
float PID_Compute(PID_t* pid, float setpoint, float measured_value, float dt)
{
	// Proportional (P)
	float error = setpoint - measured_value;
    
    // Integral (I) term and Anti-windup
    pid->integral += error * dt;
    if (pid->integral > pid->maxIntegral) 
	{
		pid->integral = pid->maxIntegral;
	}
    else if (pid->integral < -pid->maxIntegral) 
	{
		pid->integral = -pid->maxIntegral;
	}
    
    // Derivative (D)
    float derivative = (error - pid->prevError) / dt;
    pid->prevError = error;
    
    // Output
    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    
    // Output limit
    if (output > pid->maxOutput) 
	{
		output = pid->maxOutput;
	}
    else if (output < -pid->maxOutput) 
	{
		output = -pid->maxOutput;
	}
    
    return output;
}

/* --------------------------------------------------------------------------
 * PID Compute — D-term trên measurement (tránh derivative kick)
 *
 * u = Kp*e + Ki*integral(e) - Kd*d(measurement)/dt
 *
 * Anti-windup: Integral Discharge — khi output bão hòa, xả integral dần.
 * -------------------------------------------------------------------------- */
float PID_Compute_Advanced(PID_t* pid, float setpoint, float measurement, float dt)
{
	float error = setpoint - measurement;

    /* --- P term --- */
    float pTerm = pid->kp * error;

    /* --- D term trên measurement (qua EMA Low-Pass Filter) --- */
    float rawD = -(measurement - pid->prevMeasurement) / dt;
    pid->dTerm = 0.25f * rawD + (1.0f - 0.25f) * pid->dTerm;		// Điều này giúp việc thay đổi đột ngột tốc độ (tránh derivative kick)
    float dTerm = pid->kd * pid->dTerm;
    pid->prevMeasurement = measurement;

    /* --- I term với clamp --- */
    pid->integral += error * dt;
    float iTerm    = pid->ki * pid->integral;
	
	// Anti-windup
	if (pid->integral > pid->maxIntegral) 
	{
		pid->integral = pid->maxIntegral;
	}
    else if (pid->integral < -pid->maxIntegral) 
	{
		pid->integral = -pid->maxIntegral;
	}

    /* --- Output --- */
    float output = pTerm + iTerm + dTerm;	

    /* --- Integral discharge khi bão hòa --- */
	// Hard clamp chỉ ngăn tích phân tăng thêm, nhưng không giải phóng integral đã tích lũy. 
	// Khi motor thoát bão hòa, integral vẫn còn lớn và gây overshoot. Discharge xả integral ra nhanh hơn → thoát windup nhanh hơn.
	/*
    if (output > pid->maxOutput)
    {
        pid->integral -= 0.15f * pid->integral * dt;
        output = pid->maxOutput;
    }
    else if (output < -pid->maxOutput)
    {
        pid->integral -= 0.15f * pid->integral * dt;
        output = -pid->maxOutput;
    }
	*/

	return (-1 * output);		// Đảo dấu để cùng chiều với động cơ
}

/**
 * @brief  Update the PID controller gain coefficients (Kp, Ki, Kd).
 * @param  pid: Pointer to PID_t controller structure.
 * @param  kp: Proportional gain coefficient.
 * @param  ki: Integral gain coefficient.
 * @param  kd: Derivative gain coefficient.
 * @return None
 * @note   Can be called dynamically during robot operation for PID tuning.
 */
void PID_Set_Parameters(PID_t* pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

/**
 * @brief  Update the discrete sampling period (dt) for the PID calculation.
 * @param  pid: Pointer to PID_t controller structure.
 * @param  dt: Sampling time period in seconds.
 * @return None
 * @note   Should match the actual execution period of the PID control loop to prevent
 *         inaccurate integration and derivative scaling.
 */
void PID_Set_Sampling_Time(PID_t* pid, float dt)
{
	pid->dt = dt;
}

/**
 * @brief  Update the maximum limit of output
 * @param  pid: Pointer to PID_t controller structure.
 * @param  maxOutput: Maximum speed applied to motor
 * @return None
 */
void PID_Set_Max_Output(PID_t* pid, float maxOutput)
{
	pid->maxOutput = maxOutput;
}

/**
 * @brief  Update the maximum integral to avoid wind-up
 * @param  pid: Pointer to PID_t controller structure.
 * @param  maxOutput: The maximum value of intergral part can reach
 * @return None
 */
void PID_Set_Max_Integral(PID_t* pid, float maxIntegral)
{
	pid->maxIntegral = maxIntegral;
}

void PID_Reset_State(PID_t* pid)
{
	pid->integral = 0.0f;
	pid->prevError = 0.0f;
	pid->dTerm = 0.0f;
	pid->prevMeasurement = 0.0f;
}