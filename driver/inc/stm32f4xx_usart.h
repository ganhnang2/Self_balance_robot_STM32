/**
 * @file 			- stm32f4xx_usart.h
 * @author 			- Nguyen Hai Nam
 * @brief 			- USART Driver for STM32F4xx microcontrollers.
 *        			This file contains data structure definitions, configurations, and API prototypes
 *        			for interfacing with the USART (Universal Synchronous Asynchronous Receiver Transmitter) peripheral.
 * 
 * @details
 * 1. Initialization & Configuration:
 *    - USART_Init(): Configures the USART(8N1) with 8 data bits, 1 stop bit, no parity, 
 *                    and supports optional Oversampling (8 or 16) and Baudrate.
 *    - USART_Set_Baud_Rate(): Automatically calculates and configures the BRR register based on the clock frequency.
 * 
 * 2. Polling Mode (Blocking Data Transfer):
 *    - USART_Write(): Transmits a data buffer by polling the TXE flag.
 *    - USART_Read(): Receives data until a terminating character ('\r', '\n', '\0') is found by polling the RXNE flag.
 * 
 * 3. Interrupt Mode (Non-blocking Data Transfer):
 *    - USART_Write_IT(): Initiates non-blocking transmission using TXE interrupts.
 *    - USART_Read_IT(): Initiates non-blocking reception using RXNE interrupts.
 *    - USART_IRQHandling(): Central Interrupt Service Routine (ISR) to handle TX/RX interrupts.
 * 
 * 4. DMA Mode (Direct Memory Access) - Only implemented on USART1:
 *    - USART_Write_DMA() & USART_Read_DMA(): APIs to support DMA-based transmission and reception.
 *    - USART_Write_Float_String(): Formats a floating-point number into an ASCII string ("%.4f\r\n") 
 *                                  and transmits it via polling (ideal for human-readable Serial Monitor/Plotter).
 *    - USART_Write_Float_Bin(): Directly transmits the 4 raw bytes of a float in IEEE-754 Little-Endian format 
 *                               without conversion overhead (ideal for high-frequency telemetry/logging).
 */
#ifndef STM32F4XX_USART_H_
#define STM32F4XX_USART_H_

#include "stm32f401xc.h"
#include "core_cm4.h"

/**
 * @defgroup USART_State_Flags USART State Flags
 * @brief    The flag state of USART peripheral
 * @{
 */
#define USART_STATE_IDLE        		0  					/*!< USART is free */
#define USART_STATE_BUSY        		1  					/*!< USART is busy */
/**
 * @}
 */
 
#define USART_TX_MODE					0
#define USART_RX_MODE					1

/**
 * @brief  						- USART Handle Structure definition
 */
typedef struct {
	USART_TypeDef* pUSARTx;					/*!< Pointer to USART registers base address */
	uint8_t* pTxBuffer;						/*!< Pointer to TX buffer */
	uint8_t* pRxBuffer;						/*!< Pointer to RX buffer */
	uint32_t TxLen;							/*!< Length of data to be transmitted */
	uint32_t RxLen;							/*!< Length of data to be received */
	uint8_t TxState;						/*!< Current TX state */
	uint8_t RxState;						/*!< Current RX state */
	struct {
		uint32_t baudrate;					/*!< Baud rate configuration */
		uint8_t oversampling8;				/*!< Oversampling mode (8 or 16) */
	} config;								/*!< USART configuration structure */
}USART_Handle_t;

/**********************************Inits*****************************************/

/**
 * @brief  						- Configure the Baud Rate register.
 *
 * @note   						- None.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  baudrate 			- Baud rate value.
 *
 * @return 						- None
 */
void USART_Set_Baud_Rate(USART_Handle_t* pUSARTHandle, uint32_t baudrate);

/**
 * @brief  						- Initialize the USART peripheral.
 *
 * @note   						- None.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 *
 * @return 						- None
 */
void USART_Init(USART_Handle_t* pUSARTHandle);

/**
 * @brief  						- De-initialize the USART peripheral.
 *
 * @note   						- Disables the USART peripheral and turns off its RCC clock.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 *
 * @return 						- None
 */
void USART_DeInit(USART_Handle_t* pUSARTHandle);

/**
 * @brief  						- Configure DMA mode only for USART1.
 *
 * @note   						- None.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 *
 * @return 						- None
 */
void USART1_DMA_Init(USART_Handle_t* pUSARTHandle);

void USART2_DMA_Init(USART_Handle_t* pUSARTHandle);

/****************************Transfer data**************************************/

/**
 * @brief  						- Write data by using Polling method.
 *
 * @note   						- Blocks until all bytes are sent.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  buffer 				- Pointer to the data buffer to be transmitted.
 * @param  nBytes 				- Number of bytes to be transmitted.
 *
 * @return 						- None
 */
void USART_Write(USART_Handle_t* pUSARTHandle, uint8_t *buffer, uint32_t nBytes);

/**
 * @brief  						- Read data by using Polling method.
 *
 * @note   						- Blocks until bytes are received or termination character is found.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  buffer 				- Pointer to the data buffer where received data will be stored.
 * @param  nBytes 				- Maximum number of bytes to receive.
 *
 * @return 						- None
 */
void USART_Read(USART_Handle_t* pUSARTHandle, uint8_t *buffer, uint32_t nBytes);

/**
 * @brief  						- Read data by using Polling method.
 *
 * @note   						- Only receive the number of fixed bytes (nBytes) and the transaction has finished completely when 
 *								nBytes bytes are loaded into buffer
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  buffer 				- Pointer to the data buffer where received data will be stored.
 * @param  nBytes 				- Maximum number of bytes to receive.
 *
 * @return 						- None
 */
void USART_Read_Until_Idle(USART_Handle_t* pUSARTHandle, uint8_t *buffer, uint32_t nBytes);

/**
 * @brief  						- Write data by using Interrupt-driven method.
 *
 * @note   						- None.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  buffer 				- Pointer to the data buffer to be transmitted.
 * @param  nBytes 				- Number of bytes to be transmitted.
 *
 * @return 						- None
 */
void USART_Write_IT(USART_Handle_t *pUSARTHandle, uint8_t *buffer, uint32_t nBytes);

/**
 * @brief  						- Read data by using Interrupt-driven method.
 *
 * @note   						- Only receive the number of fixed bytes (nBytes) and the transaction has finished completely when 
 *								nBytes bytes are loaded into buffer
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  buffer 				- Pointer to the data buffer where received data will be stored.
 * @param  nBytes 				- Number of bytes to receive.
 *
 * @return 						- None
 */
void USART_Read_IT(USART_Handle_t *pUSARTHandle, uint8_t *buffer, uint32_t nBytes);

/**
 * @brief  						- Send a float value formatted as an ASCII string.
 *
 * @note   						- Format: "%.4f\r\n". Requires '-u _printf_float' in LDFLAGS when using nano.specs.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  val 					- Float value to be transmitted.
 *
 * @return 						- None
 */
void USART_Write_Float_String(USART_Handle_t *pUSARTHandle, float val);

/**
 * @brief  						- Send a float value as raw binary data (IEEE-754 32-bit single precision).
 *
 * @note   						- Sends 4 bytes directly in Little-Endian format (suitable for high-speed logging, MATLAB, Python, Vofa+).
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  val 					- Float value to be transmitted.
 *
 * @return 						- None
 */
void USART_Write_Float_Bin(USART_Handle_t *pUSARTHandle, float val);

/**
 * @brief  						- Read data by using Interrupt-driven method.
 *
 * @note   						- None.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  buffer 				- Pointer to the data buffer where received data will be stored.
 * @param  nBytes 				- Maximum number of bytes to receive.
 *
 * @return 						- None
 */
void USART_Read_Until_Idle_IT(USART_Handle_t *pUSARTHandle, uint8_t *buffer, uint32_t nBytes);

/**
 * @brief  						- Write data by using DMA (Direct Memory Access).
 *
 * @note   						- Only USART1 is supported for DMA in this driver.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  buffer 				- Pointer to the data buffer to be transmitted.
 * @param  nBytes 				- Number of bytes to be transmitted.
 *
 * @return 						- None
 */
void USART_Write_DMA(USART_Handle_t *pUSARTHandle, uint8_t *buffer, uint32_t nBytes);

/**
 * @brief  						- Read data by using DMA (Direct Memory Access).
 *
 * @note   						- Only USART1 is supported for DMA in this driver.
 * @note						- Only receive the number of fixed bytes (nBytes) and the transaction has finished completely when 
 *								nBytes bytes are loaded into buffer
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  buffer 				- Pointer to the data buffer where received data will be stored.
 * @param  nBytes 				- Number of bytes to be received.
 *
 * @return 						- None
 */
void USART_Read_DMA(USART_Handle_t *pUSARTHandle, uint8_t *buffer, uint32_t nBytes);

/**
 * @brief  						- Read data by using DMA (Direct Memory Access).
 *
 * @note   						- Only USART1 is supported for DMA in this driver.
 * @note						- This funtion allow user to receive data which has any number of bytes. It's the advanced function when comparing
 *								to USART_Read_DMA() function.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  buffer 				- Pointer to the data buffer where received data will be stored.
 * @param  nBytes 				- Number of bytes to be received.
 *
 * @return 						- None
 */
void USART_Read_Until_Idle_DMA(USART_Handle_t *pUSARTHandle, uint8_t *buffer, uint32_t nBytes);

/**
 * @brief  						- Sending floating-point values according to the IEEE 754 standard 
 *
 * @note   						- This function is only supported after user enabled DMA Controller.
 * @note						- Floating-point values are represented according to the IEEE 754 (4-byte) standard
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  value				- Floating-point value
 *
 * @return 						- None
 */
void USART_Send_Float_Bin(USART_Handle_t *pUSARTHandle, float val);

/**
 * @brief  						- Sending floating-point values according to string
 *
 * @note   						- This function is only supported after user enabled DMA Controller.
 * @note						- Floating-point values are represented according to string
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  value				- Floating-point value
 *
 * @return 						- None
 */
void USART_Send_Float_String(USART_Handle_t *pUSARTHandle, float val);

/**
 * @brief  						- Get the state of USART conversion.
 *
 * @note   						- None.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  mode					- Tx/Rx mode (e.g., USART_TX_MODE, USART_RX_MODE)
 *
 * @return 						- The current state of the USART (e.g., USART_STATE_IDLE, USART_STATE_BUSY)
 */
uint8_t USART_Get_State(USART_Handle_t *pUSARTHandle, uint8_t mode);

/******************************Interrupt Handling******************************/

/**
 * @brief  						- Interrupt handling for USART TX/RX.
 *
 * @note   						- None.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 *
 * @return 						- None
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/**
 * @brief  						- Interrupt handling for USART1 DMA.
 *
 * @note   						- None.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 *
 * @return 						- None
 */
void USART1_DMA_IRQHandling(USART_Handle_t *pUSARTHandle);

/**
 * @brief  						- Interrupt handling for USART2 DMA.
 *
 * @note   						- None.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 *
 * @return 						- None
 */
void USART2_DMA_IRQHandling(USART_Handle_t *pUSARTHandle);

#endif