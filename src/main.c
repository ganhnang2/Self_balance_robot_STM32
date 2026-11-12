/**
 * @file    main.c
 * @author  Nam Nguyen Hai
 * @brief   Main application for STM32F401CCU6 Two-Wheeled Self-Balancing Robot.
 *
 * @details Implements a Linear Quadratic Regulator (LQR) full state-feedback
 *          controller running in a hard real-time 200 Hz (5 ms) timer interrupt (TIM2).
 *          Sensor fusion (MPU-6050 + Kalman Filter) and chassis odometry (Quadrature Encoders)
 *          feed the 4 continuous states [position, velocity, pitch angle, pitch rate]
 *          into the optimal control law:
 *
 *              Va = - [ K1*(p - p_ref) + K2*(v - v_ref) + K3*(theta - theta_ref) + K4*omega ]
 *
 *          The background loop handles online parameter tuning via Bluetooth (HC-05),
 *          remote directional steering and real-time IMU telemetry.
 *
 * @note    Per user request, OLED display updates are omitted to keep the execution
 *          lean and focused on real-time motor control and telemetry streaming.
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

/* CMSIS & Architecture Headers */
#include "stm32f4xx.h"
#include "core_cm4.h"

/* Board Configuration & Peripheral Drivers */
#include "pin_config.h"
#include "nvic_config.h"
#include "stm32f4xx_utils.h"

/* External Component Libraries */
#include "stm32f4xx_motor.h"
#include "stm32f4xx_mpu6050.h"
#include "stm32f4xx_kalman.h"
#include "stm32f4xx_lqr.h"
#include "stm32f4xx_hc05.h"

/* =========================================================================
 * Physical Constants & Control Tuning Defines
 * ========================================================================= */
#define CONTROL_LOOP_FREQ_HZ        200.0f                          /*!< Control loop frequency: 200 Hz */
#define CONTROL_LOOP_DT             (1.0f / CONTROL_LOOP_FREQ_HZ)   /*!< Sampling time dt: 0.005 s (5 ms) */

#define DEG_TO_RAD                  (3.14159265f / 180.0f)          /*!< Conversion factor degrees to radians */
#define RAD_TO_DEG                  (180.0f / 3.14159265f)          /*!< Conversion factor radians to degrees */

/* Safety Failsafe Limits */
#define FALLEN_ANGLE_LIMIT_DEG      35.0f                           /*!< Emergency cutoff if |pitch| > 35° */
#define RECOVERY_ANGLE_LIMIT_DEG    3.0f                            /*!< Auto re-enable when |pitch| < 3° */

/* Motor & Actuator Parameters */
#define BATTERY_VOLTAGE             12.0f                           /*!< Nominal battery pack voltage (V) */
#define PWM_MAX_ARR                 1999                            /*!< TIM3 ARR for 12.5 kHz PWM */
#define MOTOR_DEADBAND_PWM          200                            	/*!< Deadband compensation count */

/* Motion & Steering Targets */
#define DRIVE_VELOCITY_TARGET       0.25f                           /*!< Target forward/reverse velocity (m/s) */
#define STEERING_VOLTAGE_STEP       1.8f                            /*!< Differential turning voltage (V) */

#define TELEMETRY_DECIMATION        8U                              /*!< Telemetry every 8 ticks (40 ms = 25 Hz) */

/* =========================================================================
 * Global Hardware & Library Handles
 * ========================================================================= */
static MPU6050_t     g_mpu6050;
static Kalman_t      g_kalman;
static LQR_t         g_lqr;
static HC05_Device_t g_hc05;

/* =========================================================================
 * System State Variables
 * ========================================================================= */
static volatile float g_pitch_deg        = 0.0f;    /*!< Current estimated pitch angle (degrees) */
static volatile float g_gyro_y_dps       = 0.0f;    /*!< Calibrated pitch angular velocity (°/s) */
static volatile float g_theta_offset_deg = -1.27f;  /*!< Mechanical balance equilibrium trim angle */
static volatile float g_steering_voltage = 0.0f;    /*!< Turning differential voltage (V) */
static volatile bool  g_robot_enabled    = false;   /*!< Active balance control state flag */
static volatile uint8_t g_telemetry_flag = 0;       /*!< Flag to trigger Bluetooth telemetry in main */

/* =========================================================================
 * Private Function Prototypes
 * ========================================================================= */
static void ControlLoop_Timer_Init(void);
static int32_t Voltage_To_PWM(float voltage, float deadband);

/* =========================================================================
 * Main Entry Point
 * ========================================================================= */
int main(void)
{
    /* 1. Core & FPU Initialization (handled in SystemInit, verified here) */
    enable_FPU();

    /* 2. Configure Hardware GPIO Pins
     *    - PC13: On-board Blue LED
     *    - PA1, PA2, PA3: HC-05 Bluetooth
     *    - PB8, PB9: MPU-6050 I2C1
     *    - PA12, PB12-PB15, PB4-PB5: TB6612 Motor Driver (Direction + TIM3 PWM)
     *    - PA8-PA9, PB6-PB7: Wheel Encoders (TIM1 & TIM4)
     */
    Pin_Config_All();

    /* 3. Configure NVIC Priority Grouping and Interrupts */
    NVIC_Config_All();

    /* 4. Initialize Actuators & Motor Encoders */
    TB6612_Init();
    TB6612_Coast();
    Motor_Encoder_Init();

    /* 5. Initialize HC-05 Bluetooth Interface (USART2 @ 115200 baud) */
    HC05_Init(&g_hc05, NULL);

    /* 6. Initialize LQR Controller Structure */
    LQR_Init(&g_lqr);
    g_lqr.deadband_pwm = MOTOR_DEADBAND_PWM;
    g_lqr.max_voltage  = BATTERY_VOLTAGE;

    /* 7. Initialize Kalman Filter for Pitch Estimation */
    Kalman_Init(&g_kalman);
	
	/* 8. Initialize MPU-6050 6-DoF IMU (Fast Mode 400 kHz on I2C1) */
	uint8_t count = 0;
	while (MPU6050_Init(&g_mpu6050, NULL) != MPU6050_OK)
	{
		if (count >= 500)
			break;
		 count++;
		delay(1);
	}

    /* 9. Visual Notice: Calibration Phase Starting
     *    Keep the robot stationary and upright during this 1-second phase!
     */
    for (uint8_t i = 0; i < 6; i++)
    {
        LED_BUILTIN_TOGGLE();
        delay(50);
    }
    LED_BUILTIN_ON(); // LED active during calibration

    /* Calibrate zero-rate gyro drift offsets */
    MPU6050_CalibrateGyro(&g_mpu6050, 400);

    /* Pre-seed Kalman filter with initial accelerometer tilt angle */
    MPU6050_ReadScaledData(&g_mpu6050);
    float init_pitch = atan2f(-g_mpu6050.ScaledData.Accel_X, g_mpu6050.ScaledData.Accel_Z) * RAD_TO_DEG;
    Kalman_SetAngle(&g_kalman, init_pitch);
    g_pitch_deg = init_pitch;

    /* Check initial orientation before enabling control */
    float init_error = fabsf(init_pitch - g_theta_offset_deg);
    if (init_error < RECOVERY_ANGLE_LIMIT_DEG)
    {
        g_robot_enabled = true;
        LED_BUILTIN_ON();  /* LED BuiltIn is ON when robot operates normally*/
    }
    else
    {
        g_robot_enabled = false;
        LED_BUILTIN_OFF(); /* LED BuiltIn is OFF when the robot falls over. Keep it off until the robot is upright again.*/
    }

    /* 10. Start Hard Real-Time Control Loop Timer (TIM2 @ 200 Hz / 5 ms) */
    ControlLoop_Timer_Init();

    /* =====================================================================
     * Background Superloop
     * Handles non-time-critical asynchronous tasks:
     *   - Bluetooth packet reception & LQR gain updates
     *   - Telemetry frame transmission to PC
     *   - Direction steering commands
     * ===================================================================== */
    uint8_t rx_buffer[HC05_RX_BUFFER_SIZE];

    while (1)
    {
        /* -----------------------------------------------------------------
         * 1. Process Incoming Bluetooth Packets from HC-05
         * ----------------------------------------------------------------- */
        if (HC05_Receive_Frame(&g_hc05, rx_buffer))
        {
            /* Check if new LQR Feedback Gains (K1..K4) were received */
            if (HC05_Is_LQR_Parameter_New(&g_hc05))
            {
                float k1, k2, k3, k4;
                HC05_Get_LQR_Parameters(&g_hc05, &k1, &k2, &k3, &k4);
                LQR_Set_Gains(&g_lqr, k1, k2, k3, k4);
            }

            /* Check if new Static Equilibrium Balance Setpoint (theta_offset) was received */
            if (HC05_Is_Setpoint_New(&g_hc05))
            {
                g_theta_offset_deg = HC05_Get_Setpoint(&g_hc05);
            }

            /* Check if new Motion Direction Command was received */
            if (HC05_Is_Direction_New(&g_hc05))
            {
                HC05_DirectionCode_t dir = HC05_Get_Direction(&g_hc05);
                switch (dir)
                {
                    case DIR_FORWARD:
                        g_lqr.vel_ref = DRIVE_VELOCITY_TARGET;
                        g_steering_voltage = 0.0f;
                        break;

                    case DIR_REVERSED:
                        g_lqr.vel_ref = -DRIVE_VELOCITY_TARGET;
                        g_steering_voltage = 0.0f;
                        break;

                    case DIR_LEFT:
                        g_lqr.vel_ref = 0.0f;
                        g_steering_voltage = -STEERING_VOLTAGE_STEP;
                        break;

                    case DIR_RIGHT:
                        g_lqr.vel_ref = 0.0f;
                        g_steering_voltage = STEERING_VOLTAGE_STEP;
                        break;

                    case DIR_NONE:
                    default:
                        g_lqr.vel_ref = 0.0f;
                        g_steering_voltage = 0.0f;
                        /* Lock current chassis position to eliminate residual drift */
                        g_lqr.pos_ref = Motor_Get_Chassis_Position() * 0.001f;
                        break;
                }
            }
        }

        /* -----------------------------------------------------------------
         * 2. Transmit IMU Telemetry to PC (hc05_fast_plotter.py)
         * ----------------------------------------------------------------- */
        if (g_telemetry_flag)
        {
            g_telemetry_flag = 0;
            HC05_Send_IMU_Frame(&g_hc05,
                                g_theta_offset_deg,
                                g_pitch_deg,
                                g_mpu6050.ScaledData.Accel_X,
                                g_mpu6050.ScaledData.Accel_Y,
                                g_mpu6050.ScaledData.Accel_Z,
                                g_mpu6050.ScaledData.Gyro_X,
                                g_mpu6050.ScaledData.Gyro_Y,
                                g_mpu6050.ScaledData.Gyro_Z);
        }
    }
}

/* =========================================================================
 * Control Loop Timer Setup (TIM2)
 * Configures TIM2 for periodic 200 Hz update interrupts (5 ms interval).
 * ========================================================================= */
static void ControlLoop_Timer_Init(void)
{
    /* Enable TIM2 clock on APB1 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Prescaler: 1 tick = 1 microsecond (1 MHz) */
    TIM2->PSC = (SystemCoreClock / 1000000UL) - 1UL;

    /* Auto-Reload: 5000 ticks = 5000 us = 5 ms (200 Hz) */
    TIM2->ARR = (uint32_t)(CONTROL_LOOP_DT * 1000000.0f) - 1UL;

    /* Clear update interrupt flag */
    TIM2->SR &= ~TIM_SR_UIF;

    /* Enable Update Interrupt (UIE) */
    TIM2->DIER |= TIM_DIER_UIE;

    /* Enable counter */
    TIM2->CR1 |= TIM_CR1_CEN;
}

/* =========================================================================
 * Voltage to PWM Converter with Stiction Deadband Compensation
 * ========================================================================= */
static int32_t Voltage_To_PWM(float voltage, float deadband)
{
    if (fabsf(voltage) < 0.05f)
    {
        return 0;
    }

    /* Linear voltage-to-duty scaling: (V / Vbat) * ARR */
    float duty = (fabsf(voltage) / BATTERY_VOLTAGE) * (float)PWM_MAX_ARR;

    /* Add deadband compensation to overcome mechanical stiction */
    duty += deadband;

    /* Saturation clamping */
    if (duty > (float)PWM_MAX_ARR)
    {
        duty = (float)PWM_MAX_ARR;
    }

    int32_t pwm = (int32_t)duty;
    return (voltage > 0.0f) ? pwm : -pwm;
}

/* =========================================================================
 * HARD REAL-TIME INTERRUPT SERVICE ROUTINES
 * ========================================================================= */

/**
 * @brief  TIM2 Interrupt Handler - Executes LQR Balancing Control Loop at 200 Hz.
 *         Configured with highest NVIC priority (Level 0).
 */
void TIM2_IRQHandler(void)
{
    static uint8_t telemetry_div = 0;

    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;

        /* 1. Read MPU-6050 Scaled Sensor Data */
        if (MPU6050_ReadScaledData(&g_mpu6050) == MPU6050_OK)
        {
            float ax = g_mpu6050.ScaledData.Accel_X;
            float ay = g_mpu6050.ScaledData.Accel_Y;
            float az = g_mpu6050.ScaledData.Accel_Z;
            float gy = g_mpu6050.ScaledData.Gyro_Y;

            /* Compute raw pitch from accelerometer: atan2(-ax, az) */
            float accel_pitch = atan2f(-ax, az) * RAD_TO_DEG;

            /* Fuse accelerometer and gyroscope with Kalman filter */
            g_pitch_deg  = Kalman_GetAngle(&g_kalman, accel_pitch, gy, CONTROL_LOOP_DT);
            g_gyro_y_dps = gy;
        }

        /* 2. Update Wheel Encoders and Read Chassis Linear States */
        Motor_Encoder_Update();
        float pos_m   = Motor_Get_Chassis_Position() * 0.001f;                              // mm -> m
        float vel_m_s = Motor_Get_Chassis_Linear_Velocity(CONTROL_LOOP_DT) * 0.001f;        // mm/s -> m/s

        /* 3. Safety Failsafe Guard (Tilt / Fallen Angle Protection) */
        float pitch_error_deg = g_pitch_deg - g_theta_offset_deg;

        if (fabsf(pitch_error_deg) > FALLEN_ANGLE_LIMIT_DEG)
        {
            /* Robot fell over or was picked up: stop motors and turn off LED immediately */
            if (g_robot_enabled)
            {
                g_robot_enabled = false;
                TB6612_Coast();
                Motor_Encoder_Reset();
                g_lqr.pos_ref = 0.0f;
                LED_BUILTIN_OFF();  /* LED BuiltIn is OFF when the robot falls over. Keep it off until the robot is upright again.*/
            }
        }
        else if (!g_robot_enabled && fabsf(pitch_error_deg) < RECOVERY_ANGLE_LIMIT_DEG)
        {
            /* Robot placed back to upright position: re-engage balancing and turn on LED */
            g_robot_enabled = true;
            Motor_Encoder_Reset();
            g_lqr.pos_ref = 0.0f;
            LED_BUILTIN_ON();       /* LED BuiltIn is ON when robot operates normally*/
        }

        /* 4. Compute and Apply LQR Actuation */
        if (g_robot_enabled)
        {
            /* Convert angle and angular velocity to standard SI units */
            float theta_rad = pitch_error_deg * DEG_TO_RAD;
            float omega_rad = g_gyro_y_dps * DEG_TO_RAD;

            /* Compute baseline balance voltage Va (Volts) */
            float v_bal = LQR_Compute(&g_lqr, pos_m, vel_m_s, theta_rad, omega_rad);

            /* Differential steering mixer */
            float v_left  = v_bal + g_steering_voltage;
            float v_right = v_bal - g_steering_voltage;

            /* Convert output voltage to PWM compare values */
            int32_t pwm_left  = Voltage_To_PWM(v_left,  g_lqr.deadband_pwm);
            int32_t pwm_right = Voltage_To_PWM(v_right, g_lqr.deadband_pwm);

            /* Drive motors via TB6612 H-bridge */
            TB6612_Control_Motors(pwm_left, pwm_right);
        }
        else
        {
            TB6612_Coast();
        }

        /* 5. Decimate Telemetry Flag for Background Loop */
        telemetry_div++;
        if (telemetry_div >= TELEMETRY_DECIMATION)
        {
            telemetry_div = 0;
            g_telemetry_flag = 1;
        }
    }
}

/**
 * @brief  USART2 Interrupt Handler for HC-05 Bluetooth reception (IDLE line detection).
 */
void USART2_IRQHandler(void)
{
    HC05_IRQ_Handler(&g_hc05);
}

/**
 * @brief  DMA1 Stream 6 Interrupt Handler for HC-05 Bluetooth TX DMA.
 */
void DMA1_Stream6_IRQHandler(void)
{
    HC05_TX_DMA_IRQHandler(&g_hc05);
}

/**
 * @brief  DMA1 Stream 5 Interrupt Handler for HC-05 Bluetooth RX DMA.
 *         (Disabled: RX reception is handled entirely by USART2 IDLE interrupt)
 */
/*
void DMA1_Stream5_IRQHandler(void)
{
    HC05_RX_DMA_IRQHandler(&g_hc05);
}
*/