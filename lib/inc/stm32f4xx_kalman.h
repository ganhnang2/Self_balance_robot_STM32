/**
 * @file    	- kalman.h
 * @author  	- Nguyen Hai Nam (Ported from Kristian Lauszus - TKJ Electronics)
 * @note		- This library has been based on Kristian Lauszus's library, I have converted his library into a C program.
 * @brief   	- 2D Linear Kalman Filter implementation for IMU Sensor Fusion (MPU6050).
 *          	- Designed for Self-Balancing Robot pitch/roll angle estimation.
 */

#ifndef STM32F4XX_KALMAN_H_
#define STM32F4XX_KALMAN_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Kalman Filter State Structure
 */
typedef struct {
    /* Tuning parameters */
    float Q_angle;   /*!< Accelerometer process noise covariance (Default: 0.001f) */
    float Q_bias;    /*!< Gyroscope drift bias process noise covariance (Default: 0.003f) */
    float R_measure; /*!< Measurement noise covariance from accelerometer (Default: 0.03f) */

    /* State estimates */
    float angle;     /*!< The estimated angle (output of the filter) */
    float bias;      /*!< The estimated gyroscope bias drift */
    float rate;      /*!< Unbiased angular velocity calculated from gyro */

    /* Error covariance matrix P */
    float P[2][2];   /*!< Estimation error covariance matrix (2x2) */
} Kalman_t;

/**
 * @brief  Initialize the Kalman filter with default parameters.
 * @param  k: Pointer to Kalman_t instance.
 * @return None
 */
void Kalman_Init(Kalman_t *k);

/**
 * @brief  Calculate and estimate the optimal filtered angle.
 * @param  k: Pointer to Kalman_t instance.
 * @param  newAngle: Angle calculated from accelerometer (in degrees).
 * @param  newRate:  Angular rate measured by gyroscope (in degrees/second).
 * @param  dt:       Delta time between updates (in seconds, e.g., 0.005f for 200Hz).
 * @return Estimated optimal angle (in degrees).
 */
float Kalman_GetAngle(Kalman_t *k, float newAngle, float newRate, float dt);

/**
 * @brief  Set initial angle to prevent startup jump.
 * @param  k: Pointer to Kalman_t instance.
 * @param  angle: Initial angle (in degrees).
 * @return None
 */
void Kalman_SetAngle(Kalman_t *k, float angle);

/**
 * @brief  Set process noise covariance for angle (Q_angle).
 * @param  k: Pointer to Kalman_t instance.
 * @param  Q_angle: Value of Q_angle.
 * @return None
 */
void Kalman_SetQangle(Kalman_t *k, float Q_angle);

/**
 * @brief  Set process noise covariance for gyro bias (Q_bias).
 * @param  k: Pointer to Kalman_t instance.
 * @param  Q_bias: Value of Q_bias.
 * @return None
 */
void Kalman_SetQbias(Kalman_t *k, float Q_bias);

/**
 * @brief  Set measurement noise covariance (R_measure).
 * @param  k: Pointer to Kalman_t instance.
 * @param  R_measure: Value of R_measure.
 * @return None
 */
void Kalman_SetRmeasure(Kalman_t *k, float R_measure);

/**
 * @brief  Get the current estimated rate (without bias).
 * @param  k: Pointer to Kalman_t instance.
 * @return Unbiased angular velocity (degrees/second).
 */
float Kalman_GetRate(Kalman_t *k);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_H_ */
