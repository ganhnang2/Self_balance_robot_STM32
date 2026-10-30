/**
 * @file    mpu6050.c
 * @author  Nguyen Hai Nam
 * @brief   Simplified MPU-6050 driver implementation for Self-Balancing Robot.
 *          All configuration is hardcoded inside MPU6050_Init():
 *            Clock=PLL X-Gyro, DLPF=42Hz, Rate=200Hz, Gyro=±250°/s, Accel=±2g
 * @note
 *   - The MPU6050_Init() function takes a pointer to an already-initialized I2C_Handle_t instance:
 *     + If NULL is passed: The driver automatically initializes and uses the default I2C1 peripheral 
 *       (Standard Mode 100 kHz). Note that GPIO PB8/PB9 must still be configured beforehand (e.g. via Pin_Config_MPU6050()).
 *     + If non-NULL is passed: The user MUST configure GPIO pins and initialize the I2C peripheral 
 *       (via I2C_Init()) externally beforehand. The driver will NOT call I2C_Init() internally, 
 *       allowing shared-bus I2C configurations.
 */

#include "stm32f4xx_mpu6050.h"
#include <string.h>

/* Internal handle for I2C1 peripheral dedicated to MPU6050 */
static I2C_Handle_t g_mpu6050I2cHandle;

/**
 * @brief  Configure GPIO pins (PB8 - SCL, PB9 - SDA) and initialize I2C1 peripheral for MPU6050.
 * @note   - PB8: I2C1_SCL (Alternate Function AF4)
 *         - PB9: I2C1_SDA (Alternate Function AF4)
 *         - Both pins configured as Open-Drain with no pull-up and pull-down (Users themselves must tie up external pullup resistor).
 */
static void MPU6050_I2C_Init(void)
{
	/* Initialize I2C1 peripheral (GPIO PB8/PB9 configured via Pin_Config_MPU6050) */
	g_mpu6050I2cHandle.pI2Cx                  = I2C1;
	g_mpu6050I2cHandle.I2C_Config.SCLSpeed    = 400000UL ;   		/* 400 kHz fast mode */
	g_mpu6050I2cHandle.I2C_Config.ACKControl  = I2C_ACK_ENABLE;
	g_mpu6050I2cHandle.I2C_Config.FMDutyCycle = I2C_FM_DUTY_16_9;
	I2C_Init(&g_mpu6050I2cHandle);
}

/*===========================================================================*/
/*                         Register / Write Helpers                          */
/*===========================================================================*/

MPU6050_Status_t MPU6050_WriteReg(MPU6050_t *dev, uint8_t regAddr, uint8_t data)
{
    uint8_t txBuf[2] = { regAddr, data };
    if (I2C_Master_Send_Data(dev->pI2CHandle, txBuf, 2U, MPU6050_I2C_ADDR, DISABLE) != I2C_OK)
    {
        return MPU6050_ERR_I2C;
    }
    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ReadReg(MPU6050_t *dev, uint8_t regAddr, uint8_t *pData)
{
    /* Write register address with Repeated Start, then read 1 byte */
    if (I2C_Master_Send_Data(dev->pI2CHandle, &regAddr, 1U, MPU6050_I2C_ADDR, ENABLE) != I2C_OK)
    {
        return MPU6050_ERR_I2C;
    }
    if (I2C_Master_Receive_Data(dev->pI2CHandle, pData, 1U, MPU6050_I2C_ADDR, DISABLE) != I2C_OK)
    {
        return MPU6050_ERR_I2C;
    }
    return MPU6050_OK;
}

/* Internal burst-read helper (not exposed in header) */
static MPU6050_Status_t mpu6050_read_regs(MPU6050_t *dev, uint8_t regAddr,
                                           uint8_t *pBuf, uint32_t len)
{
    if (I2C_Master_Send_Data(dev->pI2CHandle, &regAddr, 1U, MPU6050_I2C_ADDR, ENABLE) != I2C_OK)
    {
        return MPU6050_ERR_I2C;
    }
    if (I2C_Master_Receive_Data(dev->pI2CHandle, pBuf, len, MPU6050_I2C_ADDR, DISABLE) != I2C_OK)
    {
        return MPU6050_ERR_I2C;
    }
    return MPU6050_OK;
}

/*===========================================================================*/
/*                         Initialization                                    */
/*===========================================================================*/

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
MPU6050_Status_t MPU6050_Init(MPU6050_t *dev, I2C_Handle_t *pI2CHandle)
{
    if (pI2CHandle != NULL)
    {
        dev->pI2CHandle = pI2CHandle;
    }
    else
    {
		MPU6050_I2C_Init();
        dev->pI2CHandle = &g_mpu6050I2cHandle;
    }

    dev->GyroOffset_X = 0.0f;
    dev->GyroOffset_Y = 0.0f;
    dev->GyroOffset_Z = 0.0f;
    memset(&dev->RawData,    0, sizeof(dev->RawData));
    memset(&dev->ScaledData, 0, sizeof(dev->ScaledData));

    /* Step 1: Verify communication — WHO_AM_I must return 0x70 */
    uint8_t whoAmI = 0U;
    MPU6050_ReadReg(dev, MPU6050_REG_WHO_AM_I, &whoAmI);
    if (whoAmI != MPU6050_WHO_AM_I_VAL)
    {
        return MPU6050_ERR_NOT_FOUND;
    }
	
    // Step 2: Reset all registers to factory defaults 
    MPU6050_WriteReg(dev, MPU6050_REG_PWR_MGMT_1, 0x80);   /* DEVICE_RESET = 1 */
    delay(100U);

    // Step 3: Wake up + select PLL X-Gyro clock (more stable than 8MHz internal) 
    MPU6050_WriteReg(dev, MPU6050_REG_PWR_MGMT_1, 0x01);   /* SLEEP=0, CLKSEL=001 */
    delay(50U);

    // Step 4: DLPF = 2 → Accel 94Hz / Gyro 98Hz 
    MPU6050_WriteReg(dev, MPU6050_REG_CONFIG, 0x02);
	delay(10U);

    // Step 5: Sample Rate = 1000 / (1 + 1) = 500 Hz 
    MPU6050_WriteReg(dev, MPU6050_REG_SMPLRT_DIV, 0x01);
	delay(10U);
	
    // Step 6: Gyro full-scale = ±500 °/s  (FS_SEL = 0b01 → bits [4:3] = 01)
    MPU6050_WriteReg(dev, MPU6050_REG_GYRO_CFG, 0x08);
	delay(10U);

    // Step 7: Accel full-scale = ±8 g  (AFS_SEL = 0b10 → bits [4:3] = 10)
    MPU6050_WriteReg(dev, MPU6050_REG_ACCEL_CFG, 0x10);
	delay(10U);
    return MPU6050_OK;
}

/*===========================================================================*/
/*                         Raw & Scaled Data Read                            */
/*===========================================================================*/

MPU6050_Status_t MPU6050_ReadRawData(MPU6050_t *dev)
{
    uint8_t buf[14];
    MPU6050_Status_t status = mpu6050_read_regs(dev, MPU6050_REG_ACCEL_XOUT, buf, 14U);
    if (status != MPU6050_OK) return status;

    /* Unpack Big-Endian pairs into signed 16-bit values */
    dev->RawData.Accel_X = (int16_t)((buf[0]  << 8) | buf[1]);
    dev->RawData.Accel_Y = (int16_t)((buf[2]  << 8) | buf[3]);
    dev->RawData.Accel_Z = (int16_t)((buf[4]  << 8) | buf[5]);
    dev->RawData.Temp    = (int16_t)((buf[6]  << 8) | buf[7]);
    dev->RawData.Gyro_X  = (int16_t)((buf[8]  << 8) | buf[9]);
    dev->RawData.Gyro_Y  = (int16_t)((buf[10] << 8) | buf[11]);
    dev->RawData.Gyro_Z  = (int16_t)((buf[12] << 8) | buf[13]);

    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ReadScaledData(MPU6050_t *dev)
{
    MPU6050_Status_t status = MPU6050_ReadRawData(dev);
    if (status != MPU6050_OK) 
		return status;

    /* Accel: divide by 4096 LSB/g, then subtract offset (g)  (±4g range) */
    dev->ScaledData.Accel_X = ((float)dev->RawData.Accel_X - dev->AccelOffset_X) / MPU6050_ACCEL_SENS;
    dev->ScaledData.Accel_Y = ((float)dev->RawData.Accel_Y - dev->AccelOffset_Y) / MPU6050_ACCEL_SENS;
    dev->ScaledData.Accel_Z = ((float)dev->RawData.Accel_Z - dev->AccelOffset_Z) / MPU6050_ACCEL_SENS;

    /* Temperature: TEMP_degC = RAW/340.0 + 36.53 */
    dev->ScaledData.Temp = ((float)dev->RawData.Temp / 340.0f) + 36.53f;

    /* Gyro: divide by 131 LSB/(°/s), then subtract offset (°/s)  (±250°/s range) */
    dev->ScaledData.Gyro_X = (float)(dev->RawData.Gyro_X - dev->GyroOffset_X) / MPU6050_GYRO_SENS;
    dev->ScaledData.Gyro_Y = (float)(dev->RawData.Gyro_Y - dev->GyroOffset_Y) / MPU6050_GYRO_SENS;
    dev->ScaledData.Gyro_Z = (float)(dev->RawData.Gyro_Z - dev->GyroOffset_Z) / MPU6050_GYRO_SENS;

    return MPU6050_OK;
}

/*===========================================================================*/
/*                         Gyro Calibration                                  */
/*===========================================================================*/

MPU6050_Status_t MPU6050_CalibrateGyro(MPU6050_t *dev, uint16_t numSamples)
{
    /* Reset any previous offset so ReadRawData gives unbiased counts */
    dev->GyroOffset_X = 0.0f;
    dev->GyroOffset_Y = 0.0f;
    dev->GyroOffset_Z = 0.0f;

    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;

    for (uint16_t i = 0U; i < numSamples; i++)
    {
        MPU6050_Status_t status = MPU6050_ReadRawData(dev);
        if (status != MPU6050_OK) 
			return status;

        sumX += (float)dev->RawData.Gyro_X;
        sumY += (float)dev->RawData.Gyro_Y;
        sumZ += (float)dev->RawData.Gyro_Z;

        delay(2U);  /* ~2ms per sample → 500 samples ≈ 1s total */
    }

    /* Store mean bias as LSB — subtracted in ReadScaledData	*/
    dev->GyroOffset_X = sumX / (float)numSamples;
    dev->GyroOffset_Y = sumY / (float)numSamples;
    dev->GyroOffset_Z = sumZ / (float)numSamples;

    return MPU6050_OK;
}

/*===========================================================================*/
/*                        	Accel Calibration                                */
/*===========================================================================*/
MPU6050_Status_t MPU6050_CalibrateAccel(MPU6050_t *dev, uint16_t numSamples)
{
    /* Reset any previous offset so ReadRawData gives unbiased counts */
    dev->AccelOffset_X = 0.0f;
    dev->AccelOffset_Y = 0.0f;
    dev->AccelOffset_Z = 0.0f;

    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;

    for (uint16_t i = 0U; i < numSamples; i++)
    {
        MPU6050_Status_t status = MPU6050_ReadRawData(dev);
        if (status != MPU6050_OK) 
			return status;
		
		sumX += (float)dev->RawData.Accel_X;
        sumY += (float)dev->RawData.Accel_Y;
        sumZ += (float)dev->RawData.Accel_Z;

        delay(2U);  /* ~2ms per sample → 500 samples ≈ 1s total */
    }

    /* Store mean bias as LSB/g — subtracted in ReadScaledData	 */
	// Accel of axis Z always be 16384 LSB/g (NOTE: The positive direction of the Z-axis is upward)
	dev->AccelOffset_X = sumX / (float)numSamples;
	dev->AccelOffset_Y = sumY / (float)numSamples;
	dev->AccelOffset_Z = (sumZ / (float)numSamples) - MPU6050_ACCEL_SENS;

    return MPU6050_OK;
}

/*===========================================================================*/
/*                         Complementary Filter                              */
/*===========================================================================*/
float MPU6050_ComplementaryFilter(float prevAngle, float gyroRate,
                                   float accelAngle, float dt, float alpha)
{
    /*
     * Blend two sources:
     *   - Gyro integration  → fast response, but drifts over time
     *   - Accel angle       → slow, noisy under vibration, but drift-free
     *
     * angle = alpha * (prevAngle + gyroRate*dt) + (1-alpha) * accelAngle
     *
     * Typical: alpha = 0.98, dt = 0.005 (200Hz)
     */
    return (alpha * (prevAngle + gyroRate * dt)) + ((1.0f - alpha) * accelAngle);
}