#include "stm32f4xx_cp_filter.h"

void Complementary_Init(Complementary_Filter_t* cp)
{
	cp->alpha = 0.98f;
	cp->angle = 0.0f;
	cp->gyroBias = 0.0f;		
}

/**
  * Blend two sources:
  *   - Gyro integration  → fast response, but drifts over time
  *   - Accel angle       → slow, noisy under vibration, but drift-free
  *
  * angle = alpha * (prevAngle + gyroRate*dt) + (1-alpha) * accelAngle
  *
  * Typical: alpha = 0.98, dt = 0.005 (200Hz)
  */
float Complementary_Compute(Complementary_Filter_t* cp, float prevAngle, float gyroRate, float accelAngle, float dt)
{
	return (cp->alpha * (prevAngle + gyroRate * dt)) + ((1.0f - cp->alpha) * accelAngle);
}

/**
 * Complementary Filter + Gyro Drift Estimator
 *
 * angle(k) = alpha * [angle(k-1) + (gyroRate - bias) * dt]
 *          + (1-alpha) * accelAngle
 *
 * bias  += (fusedAngle - accelAngle) * (dt / TC)
 *
 */
float Complementary_Compute_Advanced(Complementary_Filter_t* cf, float accelAngle, float gyroRate, float dt)
{
	/* Reject invalid time steps; do not integrate or adapt bias on bad timing. */
	if (dt <= 0.0f || dt > 0.05f)
	{
		return cf->angle;
	}

	/* alpha is the gyro weight at the nominal 200 Hz sample rate. */
	float alpha = cf->alpha;
	if (alpha < 0.0f) alpha = 0.0f;
	if (alpha > 1.0f) alpha = 1.0f;

	/* Integrate gyro rate (degrees/second) after subtracting estimated bias. */
	float gyroAngle = cf->angle + (gyroRate - cf->gyroBias) * dt;
	cf->angle = alpha * gyroAngle + (1.0f - alpha) * accelAngle;

	/*
	 * Slowly adapt gyro bias. The correction gain has units 1/second^2,
	 * so angle error * dt * gain updates bias in degrees/second.
	 * Clamp the residual to reduce bias learning during strong acceleration.
	 */
	float residual = cf->angle - accelAngle;
	if (residual > 10.0f) residual = 10.0f;
	else if (residual < -10.0f) residual = -10.0f;

	const float biasAdaptationGain = 0.02f; /* conservative; tune with stationary logs */
	cf->gyroBias += biasAdaptationGain * residual * dt;

	if (cf->gyroBias > 5.0f) cf->gyroBias = 5.0f;
	else if (cf->gyroBias < -5.0f) cf->gyroBias = -5.0f;

	return cf->angle;
}

void Complementary_Reset(Complementary_Filter_t* cp)
{
	cp->angle = 0.0f;
	cp->gyroBias = 0.0f;
}

void Complementary_Set_Alpha(Complementary_Filter_t* cp, float alpha)
{
	cp->alpha = alpha;
}

float Complementary_Get_Angle(Complementary_Filter_t* cp)
{
	return cp->angle;
}