#include <stdint.h>
#include <math.h>
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_utils.h"

/** @brief Complementary Filter state + drift estimator */
typedef struct {
	float alpha;
	float gyroBias;
	float angle;
} Complementary_Filter_t;

void Complementary_Init(Complementary_Filter_t* cp);

/**
  * Blend two sources:
  *   - Gyro integration  → fast response, but drifts over time
  *   - Accel angle       → slow, noisy under vibration, but drift-free
  *
  * angle = alpha * (prevAngle + gyroRate*dt) + (1-alpha) * accelAngle
  *
  * Typical: alpha = 0.98, dt = 0.005 (200Hz)
  */
float Complementary_Compute(Complementary_Filter_t* cp, float prevAngle, float gyroRate, float accelAngle, float dt);

/**
  * Complementary Filter + Gyro Drift Estimator
  *
  * angle(k) = alpha * [angle(k-1) + (gyroRate - bias) * dt]
  *          + (1-alpha) * accelAngle
  *
  * bias  += (fusedAngle - accelAngle) * (dt / TC)
  *
  */
float Complementary_Compute_Advanced(Complementary_Filter_t* cp, float accelAngle, float gyroRate, float dt);

void Complementary_Reset(Complementary_Filter_t* cp);

void Complementary_Set_Alpha(Complementary_Filter_t* cp, float alpha);

float Complementary_Get_Angle(Complementary_Filter_t* cp);
