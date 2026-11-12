/**
 * @file    stm32f4xx_lqr.h
 * @author  Nguyen Hai Nam
 * @brief   Linear Quadratic Regulator (LQR) Controller for Self-Balancing Robot.
 *
 * @note    State Vector: x = [p, p_dot, theta, theta_dot]^T
 *          - p:         Linear position along longitudinal axis (meters)
 *          - p_dot:     Linear velocity (m/s)
 *          - theta:     Tilt/pitch angle relative to upright vertical (radians)
 *          - theta_dot: Tilt/pitch angular velocity (rad/s)
 *
 *          Control Law:
 *          u = -K * (x - x_ref)
 *            = - [ K1*(p - p_ref) + K2*(p_dot - v_ref) + K3*(theta - theta_ref) + K4*theta_dot ]
 *
 *          Output u is motor armature voltage Va in Volts.
 */

#ifndef STM32F4XX_LQR_H_
#define STM32F4XX_LQR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief LQR Controller State & Parameter Structure
 */
typedef struct {
    /* State-feedback gain vector K = [K1, K2, K3, K4] */
    float K1;               /*!< Position gain (V/m) */
    float K2;               /*!< Velocity gain (V/(m/s)) */
    float K3;               /*!< Pitch angle gain (V/rad) */
    float K4;               /*!< Pitch angular velocity gain (V/(rad/s)) */

    /* Reference states */
    float pos_ref;          /*!< Desired linear position (meters) */
    float vel_ref;          /*!< Desired linear velocity (m/s) */
    float theta_ref;        /*!< Desired tilt angle trim (radians) */

    /* Hardware limits & conversion */
    float max_voltage;      /*!< Maximum motor voltage (default: 12.0f V) */
    float deadband_pwm;     /*!< Deadband PWM to overcome gearbox stiction */
} LQR_t;

/**
 * @brief  Initialize LQR controller with nominal gains and parameters.
 * @param  lqr: Pointer to LQR_t instance.
 */
void LQR_Init(LQR_t* lqr);

/**
 * @brief  Update LQR feedback gains K1, K2, K3, K4.
 * @param  lqr: Pointer to LQR_t instance.
 * @param  k1:  Position gain.
 * @param  k2:  Velocity gain.
 * @param  k3:  Angle gain.
 * @param  k4:  Angular velocity gain.
 */
void LQR_Set_Gains(LQR_t* lqr, float k1, float k2, float k3, float k4);

/**
 * @brief  Update reference states for position, velocity, and tilt angle trim.
 * @param  lqr:       Pointer to LQR_t instance.
 * @param  pos_ref:   Desired position in meters.
 * @param  vel_ref:   Desired velocity in m/s.
 * @param  theta_ref: Desired tilt angle in radians.
 */
void LQR_Set_References(LQR_t* lqr, float pos_ref, float vel_ref, float theta_ref);

/**
 * @brief  Compute the LQR control output voltage Va.
 * @param  lqr:       Pointer to LQR_t instance.
 * @param  p:         Current measured position in meters.
 * @param  v:         Current measured velocity in m/s.
 * @param  theta:     Current measured tilt angle in radians.
 * @param  omega:     Current measured tilt angular velocity in rad/s.
 * @return Computed motor voltage Va clamped to [-max_voltage, +max_voltage].
 */
float LQR_Compute(LQR_t* lqr, float p, float v, float theta, float omega);

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_LQR_H_ */
