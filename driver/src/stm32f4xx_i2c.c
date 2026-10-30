/**
 * @file   	- stm32f4xx_i2c.c
 * @author  - Nguyen Hai Nam
 * @brief   - Source file for STM32F4xx I2C driver.
 *
 *            Supports:
 *              - Standard Mode (100 kHz) and Fast Mode (400 kHz)
 *              - Master Polling TX/RX
 *              - Master Interrupt TX/RX
 *              - Slave Interrupt TX/RX (byte-by-byte via callback)
 *              - Event and Error IRQ handlers
 * @note	- This driver only supports 7-bit address device.
 * @note	- The STM32F4xx implementation supports master mode only. If slave mode is required,
 *			users are responsible for implementing it themselves.
 */

#include "stm32f4xx_i2c.h"
#include <stddef.h>

/******************************************************************************/
/*                          Private Helper Prototypes                         */
/******************************************************************************/

static uint32_t i2c_get_pclk1_value(void);
static void     i2c_generate_start(I2C_TypeDef *pI2Cx);
static void     i2c_generate_stop(I2C_TypeDef *pI2Cx);
static void     i2c_execute_address_phase(I2C_TypeDef *pI2Cx, uint8_t slaveAddr, uint8_t rw);
static void     i2c_clear_addr_flag(I2C_Handle_t *pI2CHandle);
static void     i2c_master_ev_txe_handle(I2C_Handle_t *pI2CHandle);
static void     i2c_master_ev_rxne_handle(I2C_Handle_t *pI2CHandle);
static void     i2c_close_send_data(I2C_Handle_t *pI2CHandle);
static void     i2c_close_receive_data(I2C_Handle_t *pI2CHandle);

/******************************************************************************/
/*               Prescaler Tables (static const — no global mutable)         */
/******************************************************************************/

static const uint16_t AHB_Prescaler[]  = {1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 8, 16, 64, 128, 256, 512};
static const uint8_t  APB1_Prescaler[] = {1, 1, 1, 1, 2, 4, 8, 16};

/******************************************************************************/
/*                     Private Helper Implementations                         */
/******************************************************************************/

/**
 * @brief  Derive the APB1 peripheral clock (PCLK1) from RCC registers.
 *         Handles HSI, HSE, and PLL clock sources.
 * @note   Declared static — internal use only.
 * @retval PCLK1 frequency in Hz.
 */
static uint32_t i2c_get_pclk1_value(void)
{
    /* Apply AHB prescaler */
    uint8_t ahbp  = (RCC->CFGR >> RCC_CFGR_HPRE_Pos)  & 0x0FU;
    /* Apply APB1 prescaler */
    uint8_t apb1p = (RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x07U;

    return SystemCoreClock / AHB_Prescaler[ahbp] / APB1_Prescaler[apb1p];
}

/**
 * @brief  Generate an I2C START condition.
 */
static void i2c_generate_start(I2C_TypeDef *pI2Cx)
{
    pI2Cx->CR1 |= (1UL << I2C_CR1_START_Pos);
}

/**
 * @brief  Generate an I2C STOP condition.
 */
static void i2c_generate_stop(I2C_TypeDef *pI2Cx)
{
    pI2Cx->CR1 |= (1UL << I2C_CR1_STOP_Pos);
}

/**
 * @brief  Send the address byte on the I2C bus.
 * @param  pI2Cx      I2C peripheral.
 * @param  slaveAddr  7-bit slave address.
 * @param  rw         I2C_WRITE (0) or I2C_READ (1).
 */
static void i2c_execute_address_phase(I2C_TypeDef *pI2Cx, uint8_t slaveAddr, uint8_t rw)
{
    /* Address is placed in bits [7:1], R/W# in bit [0] */
    uint8_t addrByte = (uint8_t)((slaveAddr << 1U) | (rw & 0x01U));
    pI2Cx->DR = addrByte;
}

/**
 * @brief  Clear the ADDR flag (set after address ACK).
 *         Reading SR1 then SR2 clears the flag.
 *         In Master RX with len == 1: disable ACK before clearing ADDR,
 *         then immediately set STOP so the peripheral generates STOP right
 *         after the single byte is received.
 */
static void i2c_clear_addr_flag(I2C_Handle_t *pI2CHandle)
{
    uint32_t dummy;

    /* Master RX with a single byte: must disable ACK before clearing ADDR */
    if ((pI2CHandle->pI2Cx->SR2 & (1UL << I2C_SR2_MSL_Pos)) &&
        (pI2CHandle->RxState == I2C_STATE_BUSY_RX) &&
        (pI2CHandle->RxSize == 1U))
    {
        /* Disable ACK */
        pI2CHandle->pI2Cx->CR1 &= ~(1UL << I2C_CR1_ACK_Pos);
    }

    /* Clear ADDR by reading SR1 then SR2 */
    dummy = pI2CHandle->pI2Cx->SR1;
    dummy = pI2CHandle->pI2Cx->SR2;
    (void)dummy;
}

/**
 * @brief  Release TX interrupt resources and reset TX state to IDLE.
 */
static void i2c_close_send_data(I2C_Handle_t *pI2CHandle)
{
    /* Disable ITBUFEN and ITEVTEN */
    pI2CHandle->pI2Cx->CR2 &= ~(1UL << I2C_CR2_ITBUFEN_Pos);
    pI2CHandle->pI2Cx->CR2 &= ~(1UL << I2C_CR2_ITEVTEN_Pos);

    pI2CHandle->TxState   = I2C_STATE_IDLE;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen     = 0U;
}

/**
 * @brief  Release RX interrupt resources and reset RX state to IDLE.
 *         Re-enables ACK if I2C_Config.ACKControl == I2C_ACK_ENABLE.
 */
static void i2c_close_receive_data(I2C_Handle_t *pI2CHandle)
{
    /* Disable ITBUFEN and ITEVTEN */
    pI2CHandle->pI2Cx->CR2 &= ~(1UL << I2C_CR2_ITBUFEN_Pos);
    pI2CHandle->pI2Cx->CR2 &= ~(1UL << I2C_CR2_ITEVTEN_Pos);

    pI2CHandle->RxState   = I2C_STATE_IDLE;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen     = 0U;
    pI2CHandle->RxSize    = 0U;

    /* Re-enable ACK for next transaction */
    if (pI2CHandle->I2C_Config.ACKControl == I2C_ACK_ENABLE)
    {
        pI2CHandle->pI2Cx->CR1 |= (1UL << I2C_CR1_ACK_Pos);
    }
}

/**
 * @brief  Handle TXE event inside I2C_EV_IRQHandling (Master TX mode).
 */
static void i2c_master_ev_txe_handle(I2C_Handle_t *pI2CHandle)
{
    if (pI2CHandle->TxLen > 0U)
    {
        pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
        pI2CHandle->pTxBuffer++;
        pI2CHandle->TxLen--;
    }
}

/**
 * @brief  Handle RXNE event inside I2C_EV_IRQHandling (Master RX mode).
 */
static void i2c_master_ev_rxne_handle(I2C_Handle_t *pI2CHandle)
{
    if (pI2CHandle->RxSize == 1U)
    {
        /* Only 1 byte total: ACK already disabled in i2c_clear_addr_flag */
        *(pI2CHandle->pRxBuffer) = (uint8_t)pI2CHandle->pI2Cx->DR;
        pI2CHandle->RxLen--;
    }
    else
    {
        /* Multiple bytes remaining */
        if (pI2CHandle->RxLen == 2U)
        {
            /* Penultimate byte: disable ACK so the last byte receives NACK */
            pI2CHandle->pI2Cx->CR1 &= ~(1UL << I2C_CR1_ACK_Pos);
        }

        *(pI2CHandle->pRxBuffer) = (uint8_t)pI2CHandle->pI2Cx->DR;
        pI2CHandle->pRxBuffer++;
        pI2CHandle->RxLen--;
    }
}

/******************************************************************************/
/*                     Public API Implementations                             */
/******************************************************************************/

/**
 * @brief  Enable or disable the I2C peripheral clock.
 * @param  pI2Cx   Pointer to I2Cx peripheral base address.
 * @param  EnorDi  ENABLE or DISABLE.
 */
void I2C_PeripheralClockControl(I2C_TypeDef *pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if      (pI2Cx == I2C1) RCC->APB1ENR |=  (1UL << RCC_APB1ENR_I2C1EN_Pos);
        else if (pI2Cx == I2C2) RCC->APB1ENR |=  (1UL << RCC_APB1ENR_I2C2EN_Pos);
        else if (pI2Cx == I2C3) RCC->APB1ENR |=  (1UL << RCC_APB1ENR_I2C3EN_Pos);
    }
    else
    {
        if      (pI2Cx == I2C1) RCC->APB1ENR &= ~(1UL << RCC_APB1ENR_I2C1EN_Pos);
        else if (pI2Cx == I2C2) RCC->APB1ENR &= ~(1UL << RCC_APB1ENR_I2C2EN_Pos);
        else if (pI2Cx == I2C3) RCC->APB1ENR &= ~(1UL << RCC_APB1ENR_I2C3EN_Pos);
    }
}

/**
 * @brief  Enable or disable the I2C peripheral (PE bit in CR1).
 * @param  pI2Cx   Pointer to I2Cx peripheral base address.
 * @param  EnorDi  ENABLE or DISABLE.
 */
void I2C_PeripheralControl(I2C_TypeDef *pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
        pI2Cx->CR1 |=  (1UL << I2C_CR1_PE_Pos);
    else
        pI2Cx->CR1 &= ~(1UL << I2C_CR1_PE_Pos);
}

/**
 * @brief  Initialize the I2C peripheral according to I2C_Handle_t configuration.
 *
 * @note   CCR calculation:
 *         Standard Mode (SM):
 *           CCR = PCLK1 / (2 * SCLSpeed)
 *         Fast Mode (FM), DUTY = 0 (T_low/T_high = 2):
 *           CCR = PCLK1 / (3 * SCLSpeed)
 *         Fast Mode (FM), DUTY = 1 (T_low/T_high = 16/9):
 *           CCR = PCLK1 / (25 * SCLSpeed)
 *
 *         TRISE calculation:
 *         SM : TRISE = floor(1000 ns / T_pclk1) + 1 = (PCLK1_MHz) + 1
 *         FM : TRISE = floor( 300 ns / T_pclk1) + 1 = (PCLK1 * 3 / 10_000_000) + 1
 *
 * @param  pI2CHandle  Pointer to I2C_Handle_t structure.
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	I2C_PeripheralClockControl(pI2CHandle->pI2Cx, ENABLE);
	
    uint32_t tempReg = 0U;
    uint32_t pclk1   = i2c_get_pclk1_value();

    /* --- CR1: disable peripheral during configuration --- */
    pI2CHandle->pI2Cx->CR1 &= ~(1UL << I2C_CR1_PE_Pos);

    /* --- CR2: set FREQ bits (APB1 clock in MHz, 2–50 range) --- */
    uint32_t freqMHz = pclk1 / 1000000UL;
    tempReg = 0U;
    tempReg |= (freqMHz & 0x3FU) << I2C_CR2_FREQ_Pos;
    pI2CHandle->pI2Cx->CR2 = tempReg;

    /* --- OAR1: own address (7-bit, bit 14 must be kept 1 per RM) --- */
    tempReg  = 0U;
    tempReg |= (uint32_t)(pI2CHandle->I2C_Config.DeviceAddress & 0x7FU) << 1U;
    tempReg |= (1UL << 14U);   /* Bit 14: always 1 (RM0090 §27.6.3) */
    pI2CHandle->pI2Cx->OAR1 = tempReg;

    /* --- CCR: clock control register --- */
    uint32_t ccr = 0U;
    tempReg      = 0U;

    if (pI2CHandle->I2C_Config.SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        /* Standard Mode */
        ccr = pclk1 / (2U * pI2CHandle->I2C_Config.SCLSpeed);
    }
    else
    {
        /* Fast Mode: set FS bit */
        tempReg |= (1UL << I2C_CCR_FS_Pos);

        if (pI2CHandle->I2C_Config.FMDutyCycle == I2C_FM_DUTY_2)
        {
            /* DUTY = 0: T_low = 2*CCR*Tpclk, T_high = CCR*Tpclk */
            tempReg &= ~(1UL << I2C_CCR_DUTY_Pos);

            /* Round down the value */
            uint32_t denominator = 3U * pI2CHandle->I2C_Config.SCLSpeed;
            ccr = (pclk1 + denominator - 1) / denominator;
            if (ccr < 0x04U)
            {
                ccr = 0x04U;
            }
        }
        else
        {
            /* DUTY = 1: T_low = 16*CCR*Tpclk, T_high = 9*CCR*Tpclk */
            tempReg |= (1UL << I2C_CCR_DUTY_Pos);

            /*  Round down the value to ensure that frequency isn't larger than 400kHz */
            uint32_t denominator = 25U * pI2CHandle->I2C_Config.SCLSpeed;
            ccr = (pclk1 + denominator - 1U) / denominator;
            if (ccr < 0x01U)
            {
                ccr = 0x1U;
            }
        }
    }

    tempReg |= (ccr & 0x0FFFU) << I2C_CCR_CCR_Pos;
    pI2CHandle->pI2Cx->CCR = tempReg;

    /* --- TRISE: maximum rise time --- */
    uint32_t trise;
    if (pI2CHandle->I2C_Config.SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        /* SM: T_rise_max = 1000 ns → TRISE = PCLK1_MHz + 1 */
        trise = freqMHz + 1U;
    }
    else
    {
        /* FM: T_rise_max = 300 ns → TRISE = floor(PCLK1 * 300e-9) + 1 */
        trise = (freqMHz * 3UL) / 10UL + 1UL;
    }
    pI2CHandle->pI2Cx->TRISE = (trise & 0x3FU) << I2C_TRISE_TRISE_Pos;
	
	I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);
	
	/* --- CR1: configure ACK --- */
    if (pI2CHandle->I2C_Config.ACKControl == I2C_ACK_ENABLE)
        pI2CHandle->pI2Cx->CR1 |= (1UL << I2C_CR1_ACK_Pos);
    else
        pI2CHandle->pI2Cx->CR1 &= ~(1UL << I2C_CR1_ACK_Pos);
	
	/* --- Reset state --- */
    pI2CHandle->TxState = I2C_STATE_IDLE;
    pI2CHandle->RxState = I2C_STATE_IDLE;
}

/**
 * @brief  De-initialize the I2C peripheral.
 *         Issues a software reset (SWRST), then disables the clock.
 * @param  pI2CHandle  Pointer to I2C_Handle_t structure.
 */
void I2C_DeInit(I2C_Handle_t *pI2CHandle)
{
    /* Software reset — clears all registers and releases the bus */
    pI2CHandle->pI2Cx->CR1 |=  (1UL << I2C_CR1_SWRST_Pos);
    pI2CHandle->pI2Cx->CR1 &= ~(1UL << I2C_CR1_SWRST_Pos);

    /* Disable PE */
    pI2CHandle->pI2Cx->CR1 &= ~(1UL << I2C_CR1_PE_Pos);

    /* Disable clock */
    I2C_PeripheralClockControl(pI2CHandle->pI2Cx, DISABLE);

    /* Reset state */
    pI2CHandle->TxState = I2C_STATE_IDLE;
    pI2CHandle->RxState = I2C_STATE_IDLE;
}

/******************************************************************************/
/*                     Polling (Blocking) — Master                            */
/******************************************************************************/

/**
 * @brief 	Transmit data as I2C Master in blocking/polling mode with timeout & AF check.
 * @param  	pI2CHandle  Pointer to I2C_Handle_t structure.
 * @param  	pTxBuffer   Data to transmit.
 * @param  	len         Number of bytes.
 * @param  	slaveAddr   7-bit slave address.
 * @param  	Sr          ENABLE = Repeated Start, DISABLE = STOP.
 * @return  I2C_OK on success, or error code (I2C_ERR_TIMEOUT, I2C_ERR_AF, etc.).
 */
uint8_t I2C_Master_Send_Data(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
                             uint32_t len, uint8_t slaveAddr, uint8_t Sr)
{
    uint32_t timeout;

    /* 1. Generate START condition */
    i2c_generate_start(pI2CHandle->pI2Cx);

    /* 2. Wait until SB (Start Bit) flag is set */
    timeout = I2C_TIMEOUT_MAX;
    while (!(pI2CHandle->pI2Cx->SR1 & (1UL << I2C_SR1_SB_Pos)))
    {
        if (--timeout == 0U)
        {
            i2c_generate_stop(pI2CHandle->pI2Cx);
            return I2C_ERR_TIMEOUT;
        }
    }

    /* 3. Send address with WRITE bit */
    i2c_execute_address_phase(pI2CHandle->pI2Cx, slaveAddr, I2C_WRITE);

    /* 4. Wait until ADDR flag is set (address sent + ACK received) */
    timeout = I2C_TIMEOUT_MAX;
    while (!(pI2CHandle->pI2Cx->SR1 & (1UL << I2C_SR1_ADDR_Pos)))
    {
        /* Check for Acknowledge Failure (NACK from slave) */
        if (pI2CHandle->pI2Cx->SR1 & (1UL << I2C_SR1_AF_Pos))
        {
            pI2CHandle->pI2Cx->SR1 &= ~(1UL << I2C_SR1_AF_Pos);
            i2c_generate_stop(pI2CHandle->pI2Cx);
            return I2C_ERR_AF;
        }

        if (--timeout == 0U)
        {
            i2c_generate_stop(pI2CHandle->pI2Cx);
            return I2C_ERR_TIMEOUT;
        }
    }

    /* 5. Clear ADDR flag (read SR1 then SR2) */
    uint32_t dummy = pI2CHandle->pI2Cx->SR1;
    dummy          = pI2CHandle->pI2Cx->SR2;
    (void)dummy;

    /* 6. Send data bytes */
    while (len > 0U)
    {
        /* Wait until TXE is set (Data Register empty) */
        timeout = I2C_TIMEOUT_MAX;
        while (!(pI2CHandle->pI2Cx->SR1 & (1UL << I2C_SR1_TXE_Pos)))
        {
            if (pI2CHandle->pI2Cx->SR1 & (1UL << I2C_SR1_AF_Pos))
            {
                pI2CHandle->pI2Cx->SR1 &= ~(1UL << I2C_SR1_AF_Pos);
                i2c_generate_stop(pI2CHandle->pI2Cx);
                return I2C_ERR_AF;
            }

            if (--timeout == 0U)
            {
                i2c_generate_stop(pI2CHandle->pI2Cx);
                return I2C_ERR_TIMEOUT;
            }
        }

        pI2CHandle->pI2Cx->DR = *pTxBuffer;
        pTxBuffer++;
        len--;
    }

    /* 7. Wait until TXE=1 and BTF=1 (byte transfer finished) */
    timeout = I2C_TIMEOUT_MAX;
    while (!(pI2CHandle->pI2Cx->SR1 & (1UL << I2C_SR1_TXE_Pos)))
    {
        if (pI2CHandle->pI2Cx->SR1 & (1UL << I2C_SR1_AF_Pos))
        {
            pI2CHandle->pI2Cx->SR1 &= ~(1UL << I2C_SR1_AF_Pos);
            i2c_generate_stop(pI2CHandle->pI2Cx);
            return I2C_ERR_AF;
        }

        if (--timeout == 0U)
        {
            i2c_generate_stop(pI2CHandle->pI2Cx);
            return I2C_ERR_TIMEOUT;
        }
    }

    timeout = I2C_TIMEOUT_MAX;
    while (!(pI2CHandle->pI2Cx->SR1 & (1UL << I2C_SR1_BTF_Pos)))
    {
        if (pI2CHandle->pI2Cx->SR1 & (1UL << I2C_SR1_AF_Pos))
        {
            pI2CHandle->pI2Cx->SR1 &= ~(1UL << I2C_SR1_AF_Pos);
            i2c_generate_stop(pI2CHandle->pI2Cx);
            return I2C_ERR_AF;
        }

        if (--timeout == 0U)
        {
            i2c_generate_stop(pI2CHandle->pI2Cx);
            return I2C_ERR_TIMEOUT;
        }
    }

    /* 8. Generate STOP or Repeated Start */
    if (Sr == DISABLE)
        i2c_generate_stop(pI2CHandle->pI2Cx);

    return I2C_OK;
}

/**
 * @brief  	Receive data as I2C Master in blocking/polling mode with timeout & AF check.
 * @param  	pI2CHandle  Pointer to I2C_Handle_t structure.
 * @param  	pRxBuffer   Buffer to store received data.
 * @param  	len         Number of bytes to receive.
 * @param  	slaveAddr   7-bit slave address.
 * @param  	Sr          ENABLE = Repeated Start, DISABLE = STOP.
 * @return  I2C_OK on success, or error code (I2C_ERR_TIMEOUT, I2C_ERR_AF, etc.).
 *
 * @note 	The process of receiving the number of byte datas:
 *			1. Generate START condition
 *			2. Wait until bit SB is set (i.e: Start condition generated successfully).
 *			3. Generate address frame including: 7-bit address + 1-bit R/W#.
 *			4. Wait until bit ADDR is set (i.e: A slave on bus line has been responsed)
 *			- Case 1: The length of byte data is 1 byte
 *			5. Disable ACK to generate NACK signal so NACK is sent after the last byte.
 *			6. Clearing ADDR by reading SR1 register followed by reading SR2.
 * 			7. Wait until RXNE is set (i.e: Data is loaded into data register (DR))
 *			8. Generate STOP condition and read the last byte in DR
 *			- Case 2: The length of byte data >= 2 byte
 *			5. Clearing ADDR by reading SR1 register followed by reading SR2.
 *			6. Wait until RXNE is set
 *			7. If current byte is next-to-last byte, we will disable ACK to generate NACK signal
 *			after last byte is loaded into DR
 *			8. Generate STOP condition and read stored byte in DR
 */
uint8_t I2C_Master_Receive_Data(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
                                uint32_t len, uint8_t slaveAddr, uint8_t Sr)
{
    uint32_t timeout;

    /* 1. Generate START */
    i2c_generate_start(pI2CHandle->pI2Cx);

    /* 2. Wait until SB flag is set */
    timeout = I2C_TIMEOUT_MAX;
    while (!(pI2CHandle->pI2Cx->SR1 & (1UL << I2C_SR1_SB_Pos)))
    {
        if (--timeout == 0U)
        {
            i2c_generate_stop(pI2CHandle->pI2Cx);
            return I2C_ERR_TIMEOUT;
        }
    }

    /* 3. Send address with READ bit */
    i2c_execute_address_phase(pI2CHandle->pI2Cx, slaveAddr, I2C_READ);

    /* 4. Wait until ADDR flag is set */
    timeout = I2C_TIMEOUT_MAX;
    while (!(pI2CHandle->pI2Cx->SR1 & (1UL << I2C_SR1_ADDR_Pos)))
    {
        /* Check for Acknowledge Failure (NACK from slave) */
        if (pI2CHandle->pI2Cx->SR1 & (1UL << I2C_SR1_AF_Pos))
        {
            pI2CHandle->pI2Cx->SR1 &= ~(1UL << I2C_SR1_AF_Pos);
            i2c_generate_stop(pI2CHandle->pI2Cx);
            return I2C_ERR_AF;
        }

        if (--timeout == 0U)
        {
            i2c_generate_stop(pI2CHandle->pI2Cx);
            return I2C_ERR_TIMEOUT;
        }
    }

    /* ---- Special case: len == 1 ---- */
    if (len == 1U)
    {
        /* Disable ACK before clearing ADDR so NACK is sent right after the byte */
        pI2CHandle->pI2Cx->CR1 &= ~(1UL << I2C_CR1_ACK_Pos);

        /* Clear ADDR */
        uint32_t dummy = pI2CHandle->pI2Cx->SR1;
        dummy          = pI2CHandle->pI2Cx->SR2;
        (void)dummy;

        /* Wait until RXNE is set */
        timeout = I2C_TIMEOUT_MAX;
        while (!(pI2CHandle->pI2Cx->SR1 & (1UL << I2C_SR1_RXNE_Pos)))
        {
            if (--timeout == 0U)
            {
                i2c_generate_stop(pI2CHandle->pI2Cx);
                if (pI2CHandle->I2C_Config.ACKControl == I2C_ACK_ENABLE)
                    pI2CHandle->pI2Cx->CR1 |= (1UL << I2C_CR1_ACK_Pos);
                return I2C_ERR_TIMEOUT;
            }
        }

        /* Generate STOP before reading DR so peripheral stops clocking */
        if (Sr == DISABLE)
            i2c_generate_stop(pI2CHandle->pI2Cx);

        *pRxBuffer = (uint8_t)pI2CHandle->pI2Cx->DR;
    }
    else
    {
        /* ---- General case: len > 1 ---- */

        /* Clear ADDR (SR1 then SR2) */
        uint32_t dummy = pI2CHandle->pI2Cx->SR1;
        dummy          = pI2CHandle->pI2Cx->SR2;
        (void)dummy;

        for (uint32_t i = len; i > 0U; i--)
        {
            /* Wait for RXNE */
            timeout = I2C_TIMEOUT_MAX;
            while (!(pI2CHandle->pI2Cx->SR1 & (1UL << I2C_SR1_RXNE_Pos)))
            {
                if (--timeout == 0U)
                {
                    i2c_generate_stop(pI2CHandle->pI2Cx);
                    if (pI2CHandle->I2C_Config.ACKControl == I2C_ACK_ENABLE)
                        pI2CHandle->pI2Cx->CR1 |= (1UL << I2C_CR1_ACK_Pos);
                    return I2C_ERR_TIMEOUT;
                }
            }

            if (i == 2U)
            {
                /* Penultimate byte: disable ACK so the last byte gets NACK */
                pI2CHandle->pI2Cx->CR1 &= ~(1UL << I2C_CR1_ACK_Pos);

                /* Generate STOP before reading the last byte */
                if (Sr == DISABLE)
                    i2c_generate_stop(pI2CHandle->pI2Cx);
            }

            *pRxBuffer = (uint8_t)pI2CHandle->pI2Cx->DR;
            pRxBuffer++;
        }
    }

    /* Re-enable ACK for future transactions */
    if (pI2CHandle->I2C_Config.ACKControl == I2C_ACK_ENABLE)
        pI2CHandle->pI2Cx->CR1 |= (1UL << I2C_CR1_ACK_Pos);

    return I2C_OK;
}

/******************************************************************************/
/*                   Interrupt (Non-blocking) — Master                        */
/******************************************************************************/

/**
 * @brief  Transmit data as I2C Master in interrupt/non-blocking mode.
 * @retval TX state at call time.
 */
uint8_t I2C_Master_Send_Data_IT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
                                 uint32_t len, uint8_t slaveAddr, uint8_t Sr)
{
    uint8_t txState = pI2CHandle->TxState;

    if (txState == I2C_STATE_IDLE)
    {
        /* 1. Save transaction context */
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen     = len;
        pI2CHandle->TxState   = I2C_STATE_BUSY_TX;
        pI2CHandle->DevAddr   = slaveAddr;
        pI2CHandle->Sr        = Sr;

        /* 2. Enable peripheral if needed */
        if (!(pI2CHandle->pI2Cx->CR1 & (1UL << I2C_CR1_PE_Pos)))
            I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);

        /* 3. Generate START — the SB event will fire in the EV IRQ */
        i2c_generate_start(pI2CHandle->pI2Cx);

        /* 4. Enable event, buffer, and error interrupts */
        pI2CHandle->pI2Cx->CR2 |= (1UL << I2C_CR2_ITEVTEN_Pos);
        pI2CHandle->pI2Cx->CR2 |= (1UL << I2C_CR2_ITBUFEN_Pos);
        pI2CHandle->pI2Cx->CR2 |= (1UL << I2C_CR2_ITERREN_Pos);
    }

    return txState;
}

/**
 * @brief  Receive data as I2C Master in interrupt/non-blocking mode.
 * @retval RX state at call time.
 */
uint8_t I2C_Master_Receive_Data_IT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
                                    uint32_t len, uint8_t slaveAddr, uint8_t Sr)
{
    uint8_t rxState = pI2CHandle->RxState;

    if (rxState == I2C_STATE_IDLE)
    {
        /* 1. Save transaction context */
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen     = len;
        pI2CHandle->RxSize    = len;
        pI2CHandle->RxState   = I2C_STATE_BUSY_RX;
        pI2CHandle->DevAddr   = slaveAddr;
        pI2CHandle->Sr        = Sr;

        /* 2. Enable peripheral if needed */
        if (!(pI2CHandle->pI2Cx->CR1 & (1UL << I2C_CR1_PE_Pos)))
            I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);

        /* 3. Generate START */
        i2c_generate_start(pI2CHandle->pI2Cx);

        /* 4. Enable event, buffer, and error interrupts */
        pI2CHandle->pI2Cx->CR2 |= (1UL << I2C_CR2_ITEVTEN_Pos);
        pI2CHandle->pI2Cx->CR2 |= (1UL << I2C_CR2_ITBUFEN_Pos);
        pI2CHandle->pI2Cx->CR2 |= (1UL << I2C_CR2_ITERREN_Pos);
    }

    return rxState;
}

/******************************************************************************/
/*                  Slave Byte-level Helpers                                  */
/******************************************************************************/

/**
 * @brief  Send one byte as I2C Slave (call from I2C_ApplicationEventCallback
 *         when event = I2C_EV_DATA_REQ).
 */
void I2C_Slave_Send_Byte(I2C_TypeDef *pI2Cx, uint8_t data)
{
    pI2Cx->DR = data;
}

/**
 * @brief  Receive one byte as I2C Slave (call from I2C_ApplicationEventCallback
 *         when event = I2C_EV_DATA_RCV).
 */
uint8_t I2C_Slave_Receive_Byte(I2C_TypeDef *pI2Cx)
{
    return (uint8_t)pI2Cx->DR;
}

/******************************************************************************/
/*                         IRQ Handlers                                       */
/******************************************************************************/

/**
 * @brief  Handle I2C event interrupt.
 *         Call from I2Cx_EV_IRQHandler().
 *
 *  Events handled:
 *    SB   → Send address (Master TX or RX)
 *    ADDR → Clear ADDR flag
 *    BTF  → Close TX (Master) or handle 3-byte-corner for RX
 *    TXE  → Send next TX byte (Master) or notify Slave
 *    RXNE → Read next RX byte (Master) or notify Slave
 *    STOPF→ Clear STOPF (Slave), notify application
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t ev, buf, sr1;

    ev  = pI2CHandle->pI2Cx->CR2 & (1UL << I2C_CR2_ITEVTEN_Pos);
    buf = pI2CHandle->pI2Cx->CR2 & (1UL << I2C_CR2_ITBUFEN_Pos);
    sr1 = pI2CHandle->pI2Cx->SR1;

    /******* 1. SB — Start Bit sent (Master only) *******/
    if (ev && (sr1 & (1UL << I2C_SR1_SB_Pos)))
    {
        /* SB is cleared by reading SR1 then writing DR */
        if (pI2CHandle->TxState == I2C_STATE_BUSY_TX)
            i2c_execute_address_phase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2C_WRITE);
        else if (pI2CHandle->RxState == I2C_STATE_BUSY_RX)
            i2c_execute_address_phase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2C_READ);
    }

    /******* 2. ADDR — Address sent/matched *******/
    if (ev && (sr1 & (1UL << I2C_SR1_ADDR_Pos)))
    {
        i2c_clear_addr_flag(pI2CHandle);
    }

    /******* 3. BTF — Byte Transfer Finished *******/
    if (ev && (sr1 & (1UL << I2C_SR1_BTF_Pos)))
    {
        if (pI2CHandle->TxState == I2C_STATE_BUSY_TX)
        {
            /* BTF + TXE: all bytes sent — close TX */
            if (sr1 & (1UL << I2C_SR1_TXE_Pos))
            {
                if (pI2CHandle->TxLen == 0U)
                {
                    /* Generate STOP or Repeated Start */
                    if (pI2CHandle->Sr == DISABLE)
                        i2c_generate_stop(pI2CHandle->pI2Cx);

                    i2c_close_send_data(pI2CHandle);
                    I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
                }
            }
        }
        else if (pI2CHandle->RxState == I2C_STATE_BUSY_RX)
        {
            /* BTF during RX — should not happen in normal RXNE-driven flow;
               handle 3-byte corner case if needed */
            (void)0;
        }
    }

    /******* 4. STOPF — Stop condition detected (Slave only) *******/
    if (ev && (sr1 & (1UL << I2C_SR1_STOPF_Pos)))
    {
        /* STOPF cleared by reading SR1 (already done above) then writing CR1 */
        pI2CHandle->pI2Cx->CR1 |= 0x0000U; /* Dummy write to clear STOPF */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    /******* 5. TXE — TX Data Register Empty *******/
    if (ev && buf && (sr1 & (1UL << I2C_SR1_TXE_Pos)))
    {
        /* Only handle in Master TX mode; Slave TX handled via callback */
        if (pI2CHandle->pI2Cx->SR2 & (1UL << I2C_SR2_MSL_Pos))
        {
            if (pI2CHandle->TxState == I2C_STATE_BUSY_TX)
                i2c_master_ev_txe_handle(pI2CHandle);
        }
        else
        {
            /* Slave TXE: notify application to provide next byte */
            if (pI2CHandle->pI2Cx->SR2 & (1UL << I2C_SR2_TRA_Pos))
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
        }
    }

    /******* 6. RXNE — RX Data Register Not Empty *******/
    if (ev && buf && (sr1 & (1UL << I2C_SR1_RXNE_Pos)))
    {
        if (pI2CHandle->pI2Cx->SR2 & (1UL << I2C_SR2_MSL_Pos))
        {
            /* Master RX */
            if (pI2CHandle->RxState == I2C_STATE_BUSY_RX)
            {
                i2c_master_ev_rxne_handle(pI2CHandle);

                if (pI2CHandle->RxLen == 0U)
                {
                    if (pI2CHandle->Sr == DISABLE)
                        i2c_generate_stop(pI2CHandle->pI2Cx);

                    i2c_close_receive_data(pI2CHandle);
                    I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
                }
            }
        }
        else
        {
            /* Slave RXNE: notify application to read received byte */
            if (!(pI2CHandle->pI2Cx->SR2 & (1UL << I2C_SR2_TRA_Pos)))
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
        }
    }
}

/**
 * @brief  Handle I2C error interrupt.
 *         Call from I2Cx_ER_IRQHandler().
 *
 *  Errors handled: BERR, ARLO, AF, OVR, TIMEOUT.
 *  All errors close the current transaction and fire the callback.
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t sr1 = pI2CHandle->pI2Cx->SR1;
    uint32_t cr2 = pI2CHandle->pI2Cx->CR2;

    /******* Bus Error *******/
    if ((sr1 & (1UL << I2C_SR1_BERR_Pos)) &&
        (cr2 & (1UL << I2C_CR2_ITERREN_Pos)))
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1UL << I2C_SR1_BERR_Pos);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERR_BERR);
    }

    /******* Arbitration Lost *******/
    if ((sr1 & (1UL << I2C_SR1_ARLO_Pos)) &&
        (cr2 & (1UL << I2C_CR2_ITERREN_Pos)))
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1UL << I2C_SR1_ARLO_Pos);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERR_ARLO);
    }

    /******* Acknowledge Failure (NACK) *******/
    if ((sr1 & (1UL << I2C_SR1_AF_Pos)) &&
        (cr2 & (1UL << I2C_CR2_ITERREN_Pos)))
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1UL << I2C_SR1_AF_Pos);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERR_AF);
    }

    /******* Overrun / Underrun *******/
    if ((sr1 & (1UL << I2C_SR1_OVR_Pos)) &&
        (cr2 & (1UL << I2C_CR2_ITERREN_Pos)))
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1UL << I2C_SR1_OVR_Pos);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERR_OVR);
    }

    /******* Timeout *******/
    if ((sr1 & (1UL << I2C_SR1_TIMEOUT_Pos)) &&
        (cr2 & (1UL << I2C_CR2_ITERREN_Pos)))
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1UL << I2C_SR1_TIMEOUT_Pos);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERR_TIMEOUT);
    }
}

/******************************************************************************/
/*                      Application Callback (Weak)                           */
/******************************************************************************/

/**
 * @brief  Application event/error callback.
 * @note   Declared __weak — override this function in your application file.
 *         The driver itself never blocks here; all handling is application-side.
 *
 * @param  pI2CHandle  Pointer to I2C_Handle_t.
 * @param  AppEvent    I2C_EV_TX_CMPLT, I2C_EV_RX_CMPLT, I2C_EV_STOP,
 *                     I2C_EV_DATA_REQ, I2C_EV_DATA_RCV,
 *                     I2C_ERR_BERR, I2C_ERR_ARLO, I2C_ERR_AF,
 *                     I2C_ERR_OVR, I2C_ERR_TIMEOUT.
 */
__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
    /* Default: no-op. Override in application. */
    (void)pI2CHandle;
    (void)AppEvent;
}
