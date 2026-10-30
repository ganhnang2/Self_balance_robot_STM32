/**
 * @file    kalman.c
 * @author  Nguyen Hai Nam (Ported from Kristian Lauszus - TKJ Electronics)
 * @brief   2D Linear Kalman Filter implementation for IMU Sensor Fusion.
 *
 * @note
 * 1. State equations:
 * - x_k = A * x_(k-1) + B * u_k
 * - z_k = H * x_k
 * - Measurement Measurement Covariance Matrix: Q = [k->Q_angle*dt 		0		]
 *													[		0		k->Q_bias*dt]
 * - Measurement Noise Covariance Matrix: R = [k->R_measure]
 * 2. Prediction:
 * - 
 */

#include "stm32f4xx_kalman.h"

/** Tuning
 * Test 1: Q_angle = 0.0005f, Q_bias = 0.003f, R_measure = 0.15f 
 * - angle: [-3.602  2.069], Độ lệch chuẩn: 0.867 độ, mean = -0.173 độ
 * - Thực hiện thay đổi tốc độ động cơ ngẫu nhiên (từ từ + đột ngột)
 * Test 2: Q_angle = 0.0001f, Q_bias = 0.0005f, R_measure = 0.90f 
 * Môi trường: Tốc độ động cơ ngẫu nhiên (từ từ + đột ngột)
 * - angle: [-2.343   1.8370], Độ lệch chuẩn: 0.6805 độ, mean = -0.2736 độ
 * Môi trường: Tốc độ tăng và giảm từ từ
 * - angle: [-0.5460  0.4200], Độ lệch chuẩn: 0.1885 độ, mean = 0.0389 độ
 * Test 3: Q_angle = 0.0001f, Q_bias = 0.0008f, R_measure = 1.50f 
 * Môi trường: Tốc độ động cơ ngẫu nhiên (từ từ + đột ngột)
 * - angle: [-2.3100  1.2280], Độ lệch chuẩn: 0.6618 độ, mean = -0.1716 độ
 * Môi trường: Tốc độ tăng và giảm từ từ
 * - angle: [-0.2700  0.4580], Độ lệch chuẩn: 0.1348 độ, mean = 0.0512 độ
 * Test 4: Q_angle = 0.0001f, Q_bias = 0.0008f, R_measure = 0.8f 
 * Môi trường: Tốc độ động cơ ngẫu nhiên (từ từ + đột ngột)
 * - angle: [-3.0520  1.4470], Độ lệch chuẩn: 0.7577 độ, mean = -0.1186 độ
 * Môi trường: Tốc độ tăng và giảm từ từ
 * - angle: [-0.5760  0.6010], Độ lệch chuẩn: 0.1916 độ, mean = 0.0696 độ
 */
void Kalman_Init(Kalman_t *k) {
    /* Default tuning values for MPU6050 on self-balancing robots */
	// Q_angle = 0.001f, Q_bias = 0.003f, R_measure = 0.03f (From the manufacturer)
    k->Q_angle   = 0.0005f;
    k->Q_bias    = 0.003f;
    k->R_measure = 0.8f;

	/* Two states: angle, bias */
	// x =  [k->angle, k->bias]
    k->angle = 0.0f;
    k->bias  = 0.0f;
    k->rate  = 0.0f;

    /* Initialize error covariance matrix P */
    k->P[0][0] = 0.0f;			
    k->P[0][1] = 0.0f;			
    k->P[1][0] = 0.0f;			
    k->P[1][1] = 0.0f;			
}

float Kalman_GetAngle(Kalman_t *k, float newAngle, float newRate, float dt) {
    /* Step 1: Predict state ahead */
    k->rate = newRate - k->bias;
    k->angle += dt * k->rate;
	// Needn't update k->bias because the previous value is equal to the next one (k->bias (current) = k->bias (previous))

    /* Update error covariance matrix P = A * P * A' + Q */
    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt;

    /* Step 2: Measurement update */
    /* Estimate error S = H * P * H' + R */
    float S = k->P[0][0] + k->R_measure;

    /* Calculate Kalman Gain K = P * H' / S */
    float K[2];
    K[0] = k->P[0][0] / S;
    K[1] = k->P[1][0] / S;

    /* Innovation / Angle residual y = z - H * x */
    float y = newAngle - k->angle;

    /* Update state estimates: x = x + K * y */
    k->angle += K[0] * y;
    k->bias  += K[1] * y;

    /* Update error covariance matrix P = (I - K * H) * P */
    float P00_temp = k->P[0][0];
    float P01_temp = k->P[0][1];

    k->P[0][0] -= K[0] * P00_temp;
    k->P[0][1] -= K[0] * P01_temp;
    k->P[1][0] -= K[1] * P00_temp;
    k->P[1][1] -= K[1] * P01_temp;

    return k->angle;
}

void Kalman_SetAngle(Kalman_t *k, float angle) {
    k->angle = angle;
}

void Kalman_SetQangle(Kalman_t *k, float Q_angle) {
    k->Q_angle = Q_angle;
}

void Kalman_SetQbias(Kalman_t *k, float Q_bias) {
    k->Q_bias = Q_bias;
}

void Kalman_SetRmeasure(Kalman_t *k, float R_measure) {
    k->R_measure = R_measure;
}

float Kalman_GetRate(Kalman_t *k) {
    return k->rate;
}
