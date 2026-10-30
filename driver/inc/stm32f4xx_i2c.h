/*
 * @file                    - stm32f4xx_i2c.h
 * @author                  - Nguyen Hai Nam
 * @brief                   - Header file for STM32F4xx I2C driver.
 *
 * @note
 * 1. This driver supports I2C communication in Standard Mode (100 kHz) and
 *    Fast Mode (400 kHz) using Polling and Interrupt modes.
 * 2. The peripheral clock (PCLK1) is automatically derived from RCC registers,
 *    supporting HSI, HSE, and PLL clock sources.
 * 3. Before calling I2C_Init(), configure the GPIO pins (SCL, SDA) as:
 *    - Alternate Function 4 (AF4)
 *    - Open-Drain output
 *    - Pull-Up enabled
 * 4. DMA mode is not implemented in this driver version.
 *
 * === I2C INITIALIZATION PROCEDURE ===
 * 1. Configure SCL/SDA GPIO pins (AF4, Open-Drain, Pull-Up).
 * 2. Call I2C_PeripheralClockControl() to enable the I2C clock.
 * 3. Fill I2C_Handle_t:
 *    - I2C_Config.SCLSpeed      : I2C_SCL_SPEED_SM (100 kHz) or I2C_SCL_SPEED_FM (400 kHz)
 *    - I2C_Config.DeviceAddress : Own 7-bit slave address (Master: set to 0)
 *    - I2C_Config.ACKControl    : I2C_ACK_ENABLE or I2C_ACK_DISABLE
 *    - I2C_Config.FMDutyCycle   : I2C_FM_DUTY_2 or I2C_FM_DUTY_16_9 (Fast Mode only)
 * 4. Call I2C_Init().
 * 5. Call I2C_PeripheralControl() to enable the peripheral (PE bit).
 * 6. For interrupt mode, enable NVIC interrupts for I2Cx_EV_IRQn and I2Cx_ER_IRQn.
 */

#ifndef STM32F4XX_I2C_H_
#define STM32F4XX_I2C_H_

#include "stm32f401xc.h"
#include "core_cm4.h"

#ifndef ENABLE
#define ENABLE          1
#endif
#ifndef DISABLE
#define DISABLE         0
#endif

/******************************************************************************/
/*                     I2C Configuration Macros                               */
/******************************************************************************/

/** @defgroup I2C_SCL_Speed I2C SCL Clock Speed */
#define I2C_SCL_SPEED_SM        100000UL    /*!< Standard Mode: 100 kHz */
#define I2C_SCL_SPEED_FM        400000UL    /*!< Fast Mode:     400 kHz */

/** @defgroup I2C_ACK_Control I2C ACK Control */
#define I2C_ACK_ENABLE          1   /*!< Acknowledge returned after each byte     */
#define I2C_ACK_DISABLE         0   /*!< No acknowledge returned                  */

/** @defgroup I2C_FM_DutyCycle I2C Fast Mode Duty Cycle (CCR[14]) */
#define I2C_FM_DUTY_2           0   /*!< T_low / T_high = 2   — default FM        */
#define I2C_FM_DUTY_16_9        1   /*!< T_low / T_high = 16/9 — lower jitter    */

/******************************************************************************/
/*                         I2C Application States                             */
/******************************************************************************/

/** @defgroup I2C_State I2C Application State Flags */
#define I2C_STATE_IDLE          0   /*!< Peripheral is free                       */
#define I2C_STATE_BUSY_TX       1   /*!< Transmission in progress                 */
#define I2C_STATE_BUSY_RX       2   /*!< Reception in progress                    */

/******************************************************************************/
/*                       I2C Application Event Codes                          */
/******************************************************************************/

/** @defgroup I2C_Application_Events I2C Application Events (used in callback) */
#define I2C_EV_TX_CMPLT         1   /*!< Transmission complete                    */
#define I2C_EV_RX_CMPLT         2   /*!< Reception complete                       */
#define I2C_EV_STOP             3   /*!< STOP condition detected (Slave mode)      */
#define I2C_EV_DATA_REQ         4   /*!< Slave: master requests data (TXE, TRA=1) */
#define I2C_EV_DATA_RCV         5   /*!< Slave: master sent data (RXNE, TRA=0)    */

/** @defgroup I2C_Application_Errors I2C Return & Error Codes */
#define I2C_OK                  0   /*!< Operation successful / No error          */
#define I2C_ERR_BERR            6   /*!< Bus error (misplaced START or STOP)      */
#define I2C_ERR_ARLO            7   /*!< Arbitration lost                         */
#define I2C_ERR_AF              8   /*!< Acknowledge failure (NACK received)      */
#define I2C_ERR_OVR             9   /*!< Overrun / underrun                       */
#define I2C_ERR_TIMEOUT         10  /*!< Timeout or Tlow error                    */

/** @defgroup I2C_Timeout I2C Timeout Configuration */
#define I2C_TIMEOUT_MAX         10000UL /*!< Polling loop timeout countdown count. */
										/* T = (The number of timeouts * the number of CPU cycles/1 timeout) / f_CPU
										   Assume that the frequency of CPU is 25MHz and the number of timeout is 100000, 
										   timeout will occur after 6.4 milliseconds. */

/******************************************************************************/
/*                    I2C Transfer Direction                                  */
/******************************************************************************/

#define I2C_WRITE               0   /*!< Master → Slave (write bit = 0) */
#define I2C_READ                1   /*!< Slave  → Master (read  bit = 1) */

/******************************************************************************/
/*                    I2C Configuration & Handle Structures                   */
/******************************************************************************/

/**
 * @brief  Configuration structure for I2Cx peripheral.
 */
typedef struct
{
    uint32_t SCLSpeed;      /*!< I2C SCL clock speed in Hz.
                                 Use I2C_SCL_SPEED_SM or I2C_SCL_SPEED_FM    */
    uint8_t  DeviceAddress; /*!< Own 7-bit slave address.
                                 Ignored when operating as Master             */
    uint8_t  ACKControl;    /*!< ACK control: I2C_ACK_ENABLE / I2C_ACK_DISABLE */
    uint8_t  FMDutyCycle;   /*!< Fast Mode duty cycle: I2C_FM_DUTY_2 or
                                 I2C_FM_DUTY_16_9 (relevant only in FM)      */
} I2C_Config_t;

/**
 * @brief  Handle structure for I2Cx peripheral.
 *         Uses CMSIS standard I2C_TypeDef* (consistent with SPI/USART drivers).
 */
typedef struct
{
    I2C_TypeDef   *pI2Cx;      /*!< Base address: I2C1, I2C2, or I2C3          */
    I2C_Config_t   I2C_Config; /*!< I2C peripheral configuration               */

    /* ---- Interrupt TX context ---- */
    uint8_t       *pTxBuffer;  /*!< Pointer to TX data buffer                  */
    uint32_t       TxLen;      /*!< Remaining bytes to transmit                */
    uint8_t        TxState;    /*!< TX state: I2C_STATE_IDLE / BUSY_TX         */

    /* ---- Interrupt RX context ---- */
    uint8_t       *pRxBuffer;  /*!< Pointer to RX data buffer                  */
    uint32_t       RxLen;      /*!< Remaining bytes to receive                 */
    uint32_t       RxSize;     /*!< Total requested receive count (NACK logic) */
    uint8_t        RxState;    /*!< RX state: I2C_STATE_IDLE / BUSY_RX         */

    /* ---- Common IT context ---- */
    uint8_t        DevAddr;    /*!< Target slave 7-bit address for transaction  */
    uint8_t        Sr;         /*!< Repeated Start: ENABLE = Sr, DISABLE = Stop */
} I2C_Handle_t;

/******************************************************************************/
/*                         I2C Driver API Prototypes                          */
/******************************************************************************/

/**
 * @defgroup I2C_APIs I2C Driver APIs
 * @{
 */

/** @name Peripheral Clock and Control */
/** @{ */

/**
 * @brief  Enable or disable the I2C peripheral clock (RCC APB1ENR).
 * @param  pI2Cx   Pointer to I2Cx peripheral base address (I2C1, I2C2, I2C3).
 * @param  EnorDi  ENABLE or DISABLE.
 */
void I2C_PeripheralClockControl(I2C_TypeDef *pI2Cx, uint8_t EnorDi);

/**
 * @brief  Enable or disable the I2C peripheral (PE bit in CR1).
 * @note   Must be called after I2C_Init() to start communication.
 * @param  pI2Cx   Pointer to I2Cx peripheral base address.
 * @param  EnorDi  ENABLE or DISABLE.
 */
void I2C_PeripheralControl(I2C_TypeDef *pI2Cx, uint8_t EnorDi);
/** @} */

/** @name Initialization and De-initialization */
/** @{ */

/**
 * @brief  Initialize the I2C peripheral according to I2C_Handle_t configuration.
 * @note   - I2C_PeripheralClockControl() must be called before this function.
 *         - I2C_PeripheralControl() must be called after this function.
 *         - CCR and TRISE are calculated automatically from PCLK1 and SCLSpeed.
 * @param  pI2CHandle  Pointer to I2C_Handle_t structure.
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);

/**
 * @brief  De-initialize the I2C peripheral.
 *         Resets the peripheral via SWRST, clears registers, disables clock.
 * @param  pI2CHandle  Pointer to I2C_Handle_t structure.
 */
void I2C_DeInit(I2C_Handle_t *pI2CHandle);
/** @} */

/** @name Polling (blocking) — Master */
/** @{ */

/**
 * @brief  Transmit data as I2C Master in blocking/polling mode with timeout & AF check.
 * @param  pI2CHandle  Pointer to I2C_Handle_t structure.
 * @param  pTxBuffer   Pointer to data buffer to be transmitted.
 * @param  len         Number of bytes to transmit.
 * @param  slaveAddr   7-bit target slave address (without R/W bit).
 * @param  Sr          ENABLE to issue Repeated Start at end; DISABLE to issue STOP.
 * @return I2C_OK (0) on success, or error code (I2C_ERR_TIMEOUT, I2C_ERR_AF, etc.).
 * @note   Guarded with I2C_TIMEOUT_MAX loop counts on each phase.
 *         Automatically clears AF flag and generates STOP if NACK received.
 */
uint8_t I2C_Master_Send_Data(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
                             uint32_t len, uint8_t slaveAddr, uint8_t Sr);

/**
 * @brief  Receive data as I2C Master in blocking/polling mode with timeout & AF check.
 * @param  pI2CHandle  Pointer to I2C_Handle_t structure.
 * @param  pRxBuffer   Pointer to buffer to store received data.
 * @param  len         Number of bytes to receive.
 * @param  slaveAddr   7-bit target slave address (without R/W bit).
 * @param  Sr          ENABLE to issue Repeated Start at end; DISABLE to issue STOP.
 * @return I2C_OK (0) on success, or error code (I2C_ERR_TIMEOUT, I2C_ERR_AF, etc.).
 * @note   Guarded with I2C_TIMEOUT_MAX loop counts on each phase.
 *         Automatically clears AF flag and generates STOP if NACK received.
 */
uint8_t I2C_Master_Receive_Data(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
                                uint32_t len, uint8_t slaveAddr, uint8_t Sr);
/** @} */

/** @name Interrupt (non-blocking) — Master */
/** @{ */

/**
 * @brief  Transmit data as I2C Master in interrupt/non-blocking mode.
 * @param  pI2CHandle  Pointer to I2C_Handle_t structure.
 * @param  pTxBuffer   Pointer to data buffer to be transmitted.
 * @param  len         Number of bytes to transmit.
 * @param  slaveAddr   7-bit target slave address.
 * @param  Sr          ENABLE for Repeated Start; DISABLE for STOP.
 * @retval TX state at the time of call (I2C_STATE_IDLE or I2C_STATE_BUSY_TX).
 */
uint8_t I2C_Master_Send_Data_IT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
                                 uint32_t len, uint8_t slaveAddr, uint8_t Sr);

/**
 * @brief  Receive data as I2C Master in interrupt/non-blocking mode.
 * @param  pI2CHandle  Pointer to I2C_Handle_t structure.
 * @param  pRxBuffer   Pointer to buffer to store received data.
 * @param  len         Number of bytes to receive.
 * @param  slaveAddr   7-bit target slave address.
 * @param  Sr          ENABLE for Repeated Start; DISABLE for STOP.
 * @retval RX state at the time of call (I2C_STATE_IDLE or I2C_STATE_BUSY_RX).
 */
uint8_t I2C_Master_Receive_Data_IT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
                                    uint32_t len, uint8_t slaveAddr, uint8_t Sr);
/** @} */

/** @name Slave byte-level helpers (call from callback) */
/** @{ */

/**
 * @brief  Send one byte as I2C Slave.
 *         Call this inside I2C_ApplicationEventCallback() when event = I2C_EV_DATA_REQ.
 * @param  pI2Cx  Pointer to I2Cx peripheral base address.
 * @param  data   Byte to send.
 */
void I2C_Slave_Send_Byte(I2C_TypeDef *pI2Cx, uint8_t data);

/**
 * @brief  Receive one byte as I2C Slave.
 *         Call this inside I2C_ApplicationEventCallback() when event = I2C_EV_DATA_RCV.
 * @param  pI2Cx  Pointer to I2Cx peripheral base address.
 * @retval Received byte.
 */
uint8_t I2C_Slave_Receive_Byte(I2C_TypeDef *pI2Cx);
/** @} */

/** @name IRQ Handling */
/** @{ */

/**
 * @brief  Handle I2C event interrupt (SB, ADDR, BTF, TXE, RXNE, STOPF).
 *         Call from I2Cx_EV_IRQHandler().
 * @param  pI2CHandle  Pointer to I2C_Handle_t structure.
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);

/**
 * @brief  Handle I2C error interrupt (BERR, ARLO, AF, OVR, TIMEOUT).
 *         Call from I2Cx_ER_IRQHandler().
 * @param  pI2CHandle  Pointer to I2C_Handle_t structure.
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
/** @} */

/** @name Application Callback */
/** @{ */

/**
 * @brief  Application event/error callback — invoked at end of TX/RX and on errors.
 * @note   This function is declared __weak in the source file.
 *         Override it in your application to handle I2C events.
 * @param  pI2CHandle  Pointer to I2C_Handle_t structure.
 * @param  AppEvent    One of I2C_EV_* or I2C_ERR_* event codes.
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);
/** @} */

/** @} */ /* End of I2C_APIs */

#endif /* STM32F4XX_I2C_H_ */
