/**
 * @file    mpu6050.h
 * @author  Nguyen Hai Nam
 * @brief   Simplified MPU-6050 driver for Self-Balancing Robot.
 *          1. All hardware settings are hardcoded for optimal balancing performance:
 *            - Clock   : PLL X-Gyro
 *            - DLPF    : 42 Hz  (DLPF_CFG = 3)
 *            - Rate    : 200 Hz (SMPLRT_DIV = 4)
 *            - Gyro FS : ±250 °/s  → 131.0 LSB/(°/s)
 *            - Accel FS: ±2 g      → 16384.0 LSB/g
 *			2. Hardware connection
*			  - GPIO Pin: PB8 (I2C1_SCL), PB9 (I2C1_SDA)
 * @note
 *   - The MPU6050_Init() function takes a pointer to an already-initialized I2C_Handle_t instance:
 *     + If NULL is passed: The driver automatically initializes and uses the default I2C1 peripheral 
 *       (Fast Mode 400 kHz). Note that GPIO PB8/PB9 must still be configured beforehand (e.g. via Pin_Config_MPU6050()).
 *     + If non-NULL is passed: The user MUST configure GPIO pins and initialize the I2C peripheral 
 *       (via I2C_Init()) externally beforehand. The driver will NOT call I2C_Init() internally, 
 *       allowing shared-bus I2C configurations.
 * @reference
 *   - MPU-6000/6050 Register Map and Descriptions (Revision 4.2)
 *   - MPU6050_Registers_and_Formulas.md
 */

#ifndef STM32F4XX_MPU6050_H_
#define STM32F4XX_MPU6050_H_

#include <stdint.h>
#include <math.h>
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/*                         I2C Address & Identity                            */
/*===========================================================================*/

#define MPU6050_I2C_ADDR        0x68U   /* 7-bit address, AD0 pin → GND */
#define MPU6050_WHO_AM_I_VAL    0x70U   /* Expected value from WHO_AM_I register */

/*===========================================================================*/
/*                         Register Addresses                                */
/*===========================================================================*/

#define MPU6050_REG_SMPLRT_DIV  0x19U   /* Sample Rate Divider                */
#define MPU6050_REG_CONFIG      0x1AU   /* DLPF configuration                 */
#define MPU6050_REG_GYRO_CFG    0x1BU   /* Gyro full-scale range              */
#define MPU6050_REG_ACCEL_CFG   0x1CU   /* Accel full-scale range             */
#define MPU6050_REG_ACCEL_XOUT  0x3BU   /* Burst read start: Accel X High     */
#define MPU6050_REG_PWR_MGMT_1  0x6BU   /* Power management                   */
#define MPU6050_REG_WHO_AM_I    0x75U   /* Device identity                    */

/*===========================================================================*/
/*                         Hardcoded Sensitivity Values                      */
/*===========================================================================*/

#define MPU6050_ACCEL_SENS      4096.0f    		/* LSB/g   for ±8g range   */
#define MPU6050_GYRO_SENS       65.5f      		/* LSB/°/s for ±500°/s range */
#define MPU6050_RAD_TO_DEG      (180.0f / 3.14159265f)

/*===========================================================================*/
/*                         Data Types                                        */
/*===========================================================================*/

/** @brief Return status codes */
typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERR_I2C,
    MPU6050_ERR_NOT_FOUND,
} MPU6050_Status_t;

/** @brief Raw 16-bit sensor readings from burst-read buffer */
typedef struct {
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Temp;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
} MPU6050_RawData_t;

/** @brief Scaled physical-unit sensor readings */
typedef struct {
    float Accel_X;      /* g      */
    float Accel_Y;      /* g      */
    float Accel_Z;      /* g      */
    float Temp;         /* °C     */
    float Gyro_X;       /* °/s    */
    float Gyro_Y;       /* °/s    */
    float Gyro_Z;       /* °/s    */
} MPU6050_ScaledData_t;

/** @brief Primary device handle */
typedef struct {
    I2C_Handle_t         *pI2CHandle;   /* Pointer to initialized I2C handle        */
    MPU6050_RawData_t     RawData;      /* Latest raw register readings             */
    MPU6050_ScaledData_t  ScaledData;   /* Latest scaled physical-unit readings     */
    float                 GyroOffset_X; /* Gyro X zero-rate bias (°/s)   			*/
    float                 GyroOffset_Y; /* Gyro Y zero-rate bias (°/s)   			*/
    float                 GyroOffset_Z; /* Gyro Z zero-rate bias (°/s)   			*/
	float 				  AccelOffset_X;/* Accel X zero-rate bias (g)   			*/
	float				  AccelOffset_Y;/* Accel Y zero-rate bias (g)   			*/
	float				  AccelOffset_Z;/* Accel Z zero-rate bias (g)   			*/
} MPU6050_t;

/*===========================================================================*/
/*                         Function Prototypes                               */
/*===========================================================================*/

/**
 * @brief  Write one byte to an MPU6050 register.
 * @param  dev:     Device handle.
 * @param  regAddr: Target register address.
 * @param  data:    Byte to write.
 * @return MPU6050_OK or MPU6050_ERR_I2C.
 */
MPU6050_Status_t MPU6050_WriteReg(MPU6050_t *dev, uint8_t regAddr, uint8_t data);

/**
 * @brief  Read one byte from an MPU6050 register.
 * @param  dev:     Device handle.
 * @param  regAddr: Target register address.
 * @param  pData:   Pointer to store the read byte.
 * @return MPU6050_OK or MPU6050_ERR_I2C.
 */
MPU6050_Status_t MPU6050_ReadReg(MPU6050_t *dev, uint8_t regAddr, uint8_t *pData);

/**
 * @brief  Initialize MPU6050 with hardcoded optimal settings for self-balancing robot.
 *         Settings applied: PLL clock, DLPF=42Hz, 200Hz sample rate, ±250°/s, ±2g.
 * @param  dev:        Device handle to initialize.
 * @param  pI2CHandle: Pointer to ALREADY-INITIALIZED I2C_Handle_t, or NULL for default I2C1.
 * @note   - If NULL is passed: The driver automatically initializes default I2C1 (PB8/PB9 configured beforehand).
 *         - If non-NULL is passed: The user MUST configure GPIO pins and call I2C_Init() externally beforehand.
 *           The driver will NOT re-initialize the I2C peripheral.
 * @return MPU6050_OK on success, MPU6050_ERR_NOT_FOUND if WHO_AM_I check fails.
 */
MPU6050_Status_t MPU6050_Init(MPU6050_t *dev, I2C_Handle_t *pI2CHandle);

/**
 * @brief  Burst-read 14 bytes (Accel + Temp + Gyro) into dev->RawData.
 * @param  dev: Device handle.
 * @return MPU6050_OK or MPU6050_ERR_I2C.
 */
MPU6050_Status_t MPU6050_ReadRawData(MPU6050_t *dev);

/**
 * @brief  Read raw data and convert to physical units into dev->ScaledData.
 *         Gyro output has zero-rate offset subtracted automatically.
 * @param  dev: Device handle.
 * @return MPU6050_OK or MPU6050_ERR_I2C.
 */
MPU6050_Status_t MPU6050_ReadScaledData(MPU6050_t *dev);

/**
 * @brief  Gyroscope zero-rate bias calibration.
 *         Keep the robot STATIONARY during this call.
 *         Averages numSamples readings to compute zero-rate offset GyroOffset_X/Y/Z (°/s).
 * @note   This function should be called once during startup while the sensor is at rest.
 * @param  dev:        Device handle.
 * @param  numSamples: Number of samples to average (e.g. 500 → ~1.0 second).
 * @return MPU6050_OK or error code.
 */
MPU6050_Status_t MPU6050_CalibrateGyro(MPU6050_t *dev, uint16_t numSamples);

/**
 * @brief  Accelerometer zero-offset calibration.
 *         Keep the robot STATIONARY at its balanced upright position (or flat) during this call.
 *         Averages numSamples readings to compute AccelOffset_X/Y/Z in units of 'g'.
 *         Assumes stationary reference: X = 0g, Y = 0g, Z = 1g (Earth gravity).
 * @note   This function should be called once during startup while the sensor is at rest.
 * @param  dev:        Device handle.
 * @param  numSamples: Number of samples to average (e.g. 500 → ~1.0 second).
 * @return MPU6050_OK or error code.
 */
MPU6050_Status_t MPU6050_CalibrateAccel(MPU6050_t *dev, uint16_t numSamples);

/**
 * @brief  Complementary filter for Pitch/Roll sensor fusion.
 *         Blends gyroscope integration (fast response) with
 *         accelerometer angle (drift-free long-term reference).
 *
 *         Formula: angle = alpha*(angle + gyroRate*dt) + (1-alpha)*accelAngle
 *
 * @param  prevAngle:  Previous filtered angle (degrees).
 * @param  gyroRate:   Calibrated gyro angular rate (°/s).
 * @param  accelAngle: Angle computed from accelerometer (degrees).
 * @param  dt:         Sampling period in seconds (e.g. 0.005f for 200Hz).
 * @param  alpha:      Filter coefficient (typically 0.98f).
 * @return Fused angle in degrees.
 */
float MPU6050_ComplementaryFilter(float prevAngle, float gyroRate,
                                   float accelAngle, float dt, float alpha);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H_ */
