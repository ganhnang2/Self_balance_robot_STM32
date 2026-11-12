/**
 * @file    stm32f4xx_lqr.c
 * @author  Nguyen Hai Nam
 * @brief   Linear Quadratic Regulator (LQR) Controller Implementation.
 */

#include "stm32f4xx_lqr.h"

void LQR_Init(LQR_t* lqr)
{
    if (lqr == (void*)0)
    {
        return;
    }

    /* Nominal gains calculated from Matlab scripts (lqr_controller.m) */
    lqr->K1 = -7.0711f;
    lqr->K2 = -27.9670f;
    lqr->K3 = -40.3837f;
    lqr->K4 = -3.5627f;

    lqr->pos_ref   = 0.0f;
    lqr->vel_ref   = 0.0f;
    lqr->theta_ref = 0.0f;

    lqr->max_voltage  = 12.0f;
    lqr->deadband_pwm = 200.0f; /* ~10% of 2000 ARR */
}

void LQR_Set_Gains(LQR_t* lqr, float k1, float k2, float k3, float k4)
{
    if (lqr == (void*)0)
    {
        return;
    }
    lqr->K1 = k1;
    lqr->K2 = k2;
    lqr->K3 = k3;
    lqr->K4 = k4;
}

void LQR_Set_References(LQR_t* lqr, float pos_ref, float vel_ref, float theta_ref)
{
    if (lqr == (void*)0)
    {
        return;
    }
    lqr->pos_ref   = pos_ref;
    lqr->vel_ref   = vel_ref;
    lqr->theta_ref = theta_ref;
}

float LQR_Compute(LQR_t* lqr, float p, float v, float theta, float omega)
{
    if (lqr == (void*)0)
    {
        return 0.0f;
    }

    /* Compute state error terms */
    float err_p     = p - lqr->pos_ref;
    float err_v     = v - lqr->vel_ref;
    float err_theta = theta - lqr->theta_ref;
    float err_omega = omega; /* Desired angular velocity is 0 */

    /* Control Law: u = - [ K1*err_p + K2*err_v + K3*err_theta + K4*err_omega ] */
    float u = -(lqr->K1 * err_p +
                lqr->K2 * err_v +
                lqr->K3 * err_theta +
                lqr->K4 * err_omega);

    /* Output voltage saturation */
    if (u > lqr->max_voltage)
    {
        u = lqr->max_voltage;
    }
    else if (u < -lqr->max_voltage)
    {
        u = -lqr->max_voltage;
    }

    return u;
}

