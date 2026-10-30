#include "stm32f4xx_usart.h"
#include <stdio.h>
#include <string.h>

static const uint8_t  APBx_Prescaler[] = {1, 1, 1, 1, 2, 4, 8, 16};

/********************Internal helper to handle receiving data in Interrupt-driven mode**************************/
static void __USART_Send(USART_Handle_t *pUSARTHandle);
static void __USART_Receive(USART_Handle_t *pUSARTHandle);

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
void USART_Set_Baud_Rate(USART_Handle_t* pUSARTHandle, uint32_t baudrate)
{
	uint32_t mantissa, fraction;
	uint32_t usartdiv;
	uint32_t PCLKx = SystemCoreClock;
	uint8_t apbxPre;
	
	// USART1, USART6 (APB2)
	if (pUSARTHandle->pUSARTx == USART1 || pUSARTHandle->pUSARTx == USART6)
	{	
		// APB high-speed prescaler (APB2)
		apbxPre = (RCC->CFGR >> RCC_CFGR_PPRE2_Pos) & 0x07U;
		PCLKx = SystemCoreClock / APBx_Prescaler[apbxPre];
	}
	// USART2 (APB1)
	else
	{
		// APB high-speed prescaler (APB1)
		apbxPre = (RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x07U;
		PCLKx = SystemCoreClock / APBx_Prescaler[apbxPre];
	}
	
	pUSARTHandle->config.baudrate = baudrate;
	// OVER8 = 0
	if (pUSARTHandle->config.oversampling8 == 0)
	{
		// Multiply to 100 to take the fraction part
		// usartdiv = ((100 * PCLKx) / (16 * baudrate));
		usartdiv = ((25 * PCLKx) / (4 * baudrate));
	}
	// OVER8 = 1
	else
	{
		// Multiply to 100 to take the fraction part
		// usartdiv = ((100 * PCLKx) / (8 * baudrate));
		usartdiv = ((25 * PCLKx) / (2 * baudrate));
	}
	
	// Calculate the mantissa and fraction part
	mantissa = usartdiv / 100;
	fraction = (usartdiv - (mantissa * 100));
	
	// OVER8 = 0
	if (pUSARTHandle->config.oversampling8 == 0)
	{
		fraction = (((fraction * 16) + 50) / 100);
		
		// Processing carry during rounding
		if (fraction > 15)
		{
			mantissa += 1;
			fraction = 0;
		}
	}
	// OVER8 = 1
	else
	{
		fraction = (((fraction * 8) + 50) / 100);
		
		// Processing carry during rounding
		if (fraction > 7)
		{
			mantissa += 1;
			fraction = 0;
		}
	}
	
	// Set the value to BRR register
	pUSARTHandle->pUSARTx->BRR = 0;
	pUSARTHandle->pUSARTx->BRR |= (mantissa << USART_BRR_DIV_Mantissa_Pos);
	pUSARTHandle->pUSARTx->BRR |= (fraction << USART_BRR_DIV_Fraction_Pos);
}

/**
 * @brief  						- Initialize the USART peripheral.
 *
 * @note   						- None.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 *
 * @return 						- None
 */

/**
 * === USART INITIALIZATION PROCEDURE ===
 * 1. Disable USART (UE = 0 in CR1) before configuration.
 * 2. Set data length: 8-bit (M bit = 0 in CR1).
 * 3. Select Stop bits: 1 Stop bit (STOP bits = 00 in CR2).
 * 4. Configure Parity control: No parity (PCE = 0 in CR1).
 * 5. Configure Oversampling: 8 or 16 (OVER8 bit in CR1).
 * 6. Calculate Baud rate to program into the USART_BRR register:
 *    
 *                  fCK (Peripheral clock frequency)
 *    Baud rate = ----------------------------------
 *                 8 * (2 - OVER8) * USARTDIV
 * 
 *    - fCK: Typically external crystal (HSE) or system clock (e.g. 8MHz, APB Prescaler = 1).
 *    - USARTDIV: Unsigned fixed-point number coded in the USART_BRR register:
 *      + When OVER8 = 0: Fractional part is coded on 4 bits (DIV_Fraction[3:0]).
 *      + When OVER8 = 1: Fractional part is coded on 3 bits (DIV_Fraction[2:0]).
 * 
 *    Example: Configuring Baud rate = 9600 with fCK = 8MHz:
 *    - OVER8 = 0:
 *      USARTDIV = 8MHz / (16 * 9600) = 52.0833
 *      DIV_Mantissa = 52 = 0x034
 *      DIV_Fraction = 0.0833 * 16 = 1.33 ≈ 1 = 0x1
 *    - OVER8 = 1:
 *      USARTDIV = 8MHz / (8 * 9600) = 104.1667
 *      DIV_Mantissa = 104 = 0x068
 *      DIV_Fraction = 0.1667 * 8 = 1.33 ≈ 1 = 0x1
 * 
 * 7. Configure DMA (if used): Enable DMA request in CR3 (only implemented on USART1).
 * 8. Enable Transmitter (TE) and Receiver (RE) in CR1.
 * 9. Enable USART (UE = 1 in CR1).
 */
void USART_Init(USART_Handle_t* pUSARTHandle)
{
	/******************************USART*********************************/
	// USART clock enable
	if (pUSARTHandle->pUSARTx == USART1)
	{
		RCC->APB2ENR |= (1 << RCC_APB2ENR_USART1EN_Pos);
	}
	else if (pUSARTHandle->pUSARTx == USART2)
	{
		RCC->APB1ENR |= (1 << RCC_APB1ENR_USART2EN_Pos);
	}
#ifdef USART3
	else if (pUSARTHandle->pUSARTx == USART3)
	{
		RCC->APB1ENR |= (1 << RCC_APB1ENR_USART3EN_Pos);
	}
#endif
#ifdef UART4
	else if (pUSARTHandle->pUSARTx == UART4)
	{
		RCC->APB1ENR |= (1 << RCC_APB1ENR_UART4EN_Pos);
	}
#endif
#ifdef UART5
	else if (pUSARTHandle->pUSARTx == UART5)
	{
		RCC->APB1ENR |= (1 << RCC_APB1ENR_UART5EN_Pos);
	}
#endif
#ifdef USART6
	else if (pUSARTHandle->pUSARTx == USART6)
	{
		RCC->APB2ENR |= (1 << RCC_APB2ENR_USART6EN_Pos);
	}
#endif
#ifdef UART7
	else if (pUSARTHandle->pUSARTx == UART7)
	{
		RCC->APB1ENR |= (1 << RCC_APB1ENR_UART7EN_Pos);
	}
#endif
#ifdef UART8
	else if (pUSARTHandle->pUSARTx == UART8)
	{
		RCC->APB1ENR |= (1 << RCC_APB1ENR_UART8EN_Pos);
	}
#endif
	
	// Disable USART
	pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_UE_Pos);
	
	// Set data Length to 8 bits 
	// 0: 1 Start bit, 8 Data bits, 1: 1 Start bit, 9 Data Bits
	pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_M_Pos);
	
	// Select 1 stop bit
	// 00: 1 bit, 01: 0.5 bit, 10: 2 bits, 11: 1.5 bit
	pUSARTHandle->pUSARTx->CR2 &= ~(0x03 << USART_CR2_STOP_Pos);
	
	// Set parity control as no parity 
	// 0 = no parity, 
	// 1 = parity enabled (then, program PS bit to select Even or Odd parity)
	pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_PCE_Pos);
	
	// Oversampling8
	// 0 = oversampling by 16, 1 = oversampling by 8 
	if (pUSARTHandle->config.oversampling8 == 1)
	{
		pUSARTHandle->pUSARTx->CR1 |= 1 << USART_CR1_OVER8_Pos;
	}
	else
	{
		pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_OVER8_Pos);
	}
	
	// USART is free
	pUSARTHandle->TxState = USART_STATE_IDLE;
	pUSARTHandle->RxState = USART_STATE_IDLE;
	
	// Set baud rate value to BRR register
	USART_Set_Baud_Rate(pUSARTHandle, pUSARTHandle->config.baudrate);
	
	// Enable transmission and reception 
	pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TE_Pos) | (1 << USART_CR1_RE_Pos);
	
	// Enable USART 
	pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_UE_Pos);
}

/**
 * @brief  						- De-initialize the USART peripheral.
 *
 * @note   						- Disables the USART peripheral and turns off its RCC clock.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 *
 * @return 						- None
 */
void USART_DeInit(USART_Handle_t* pUSARTHandle)
{
	// Disable USART
	pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_UE_Pos);

	// Disable RCC clock for the corresponding USART instance
	if (pUSARTHandle->pUSARTx == USART1)
	{
		RCC->APB2ENR &= ~(1 << RCC_APB2ENR_USART1EN_Pos);
	}
	else if (pUSARTHandle->pUSARTx == USART2)
	{
		RCC->APB1ENR &= ~(1 << RCC_APB1ENR_USART2EN_Pos);
	}
#ifdef USART3
	else if (pUSARTHandle->pUSARTx == USART3)
	{
		RCC->APB1ENR &= ~(1 << RCC_APB1ENR_USART3EN_Pos);
	}
#endif
#ifdef UART4
	else if (pUSARTHandle->pUSARTx == UART4)
	{
		RCC->APB1ENR &= ~(1 << RCC_APB1ENR_UART4EN_Pos);
	}
#endif
#ifdef UART5
	else if (pUSARTHandle->pUSARTx == UART5)
	{
		RCC->APB1ENR &= ~(1 << RCC_APB1ENR_UART5EN_Pos);
	}
#endif
#ifdef USART6
	else if (pUSARTHandle->pUSARTx == USART6)
	{
		RCC->APB2ENR &= ~(1 << RCC_APB2ENR_USART6EN_Pos);
	}
#endif
#ifdef UART7
	else if (pUSARTHandle->pUSARTx == UART7)
	{
		RCC->APB1ENR &= ~(1 << RCC_APB1ENR_UART7EN_Pos);
	}
#endif
#ifdef UART8
	else if (pUSARTHandle->pUSARTx == UART8)
	{
		RCC->APB1ENR &= ~(1 << RCC_APB1ENR_UART8EN_Pos);
	}
#endif

	// Reset state
	pUSARTHandle->TxState = USART_STATE_IDLE;
	pUSARTHandle->RxState = USART_STATE_IDLE;
}

/**
 * @brief  						- Configure DMA mode only for USART1.
 *
 * @note   						- Only use DMA for USART1.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 *
 * @return 						- None
 */
void USART1_DMA_Init(USART_Handle_t* pUSARTHandle)
{
	// NOTE: Only use DMA for USART1
	if (pUSARTHandle->pUSARTx == USART1)
	{
		/********************************DMA USART*********************************/
		pUSARTHandle->pUSARTx->CR3 |= (1 << USART_CR3_DMAT_Pos);  // Enable DMA Transmit request
		pUSARTHandle->pUSARTx->CR3 |= (1 << USART_CR3_DMAR_Pos);  // Enable DMA Receive request
		
		/*******************************DMA Peripheral****************************/
		// Enable DMA2 clock
		RCC->AHB1ENR |= (1 << RCC_AHB1ENR_DMA2EN_Pos);
		
		//======USART1_RX========//
		// Disable DMA channel
		// NOTE: Only configure the DMA peripheral when DMA is disabled
		DMA2_Stream5->CR &= ~(1 << DMA_SxCR_EN_Pos);
		while (DMA2_Stream5->CR & (1 << DMA_SxCR_EN_Pos));
		
		// Peripheral data size 00 = 8bits
		DMA2_Stream5->CR &= ~(0x03 << DMA_SxCR_PSIZE_Pos);
		
		// Memory data size: 00 = 8 bits
		DMA2_Stream5->CR &= ~(0x03 << DMA_SxCR_MSIZE_Pos);
		
		// Disable peripheral increment mode
		DMA2_Stream5->CR &= ~(1 << DMA_SxCR_PINC_Pos);
		
		// Enable memory increment mode
		DMA2_Stream5->CR |= (1 << DMA_SxCR_MINC_Pos);
		
		// Choose the channel selection (Steam 5 & Channel 4 - USART1_RX)
		DMA2_Stream5->CR &= ~(0x07 << DMA_SxCR_CHSEL_Pos);
		DMA2_Stream5->CR |= (0x04 << DMA_SxCR_CHSEL_Pos);
		
		//======USART1_TX========//
		DMA2_Stream7->CR &= ~(1 << DMA_SxCR_EN_Pos);
		while (DMA2_Stream7->CR & (1 << DMA_SxCR_EN_Pos));
		
		// Peripheral data size 00 = 8bits
		DMA2_Stream7->CR &= ~(0x03 << DMA_SxCR_PSIZE_Pos);
		
		// Memory data size: 00 = 8 bits
		DMA2_Stream7->CR &= ~(0x03 << DMA_SxCR_MSIZE_Pos);
		
		// Disable peripheral increment mode
		DMA2_Stream7->CR &= ~(1 << DMA_SxCR_PINC_Pos);
		
		// Enable memory increment mode
		DMA2_Stream7->CR |= (1 << DMA_SxCR_MINC_Pos);
		
		// Choose the channel selection (Steam 7 & Channel 4 - USART1_TX)
		DMA2_Stream7->CR &= ~(0x07 << DMA_SxCR_CHSEL_Pos);
		DMA2_Stream7->CR |= (0x04 << DMA_SxCR_CHSEL_Pos);
	}
}

/**
 * @brief  						- Configure DMA mode only for USART2.
 *
 * @note   						- Only use DMA for USART2.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 *
 * @return 						- None
 */
void USART2_DMA_Init(USART_Handle_t* pUSARTHandle)
{
	// NOTE: Only use DMA for USART1
	if (pUSARTHandle->pUSARTx == USART2)
	{
		/********************************DMA USART*********************************/
		pUSARTHandle->pUSARTx->CR3 |= (1 << USART_CR3_DMAT_Pos);  // Enable DMA Transmit request
		// pUSARTHandle->pUSARTx->CR3 |= (1 << USART_CR3_DMAR_Pos);  // Enable DMA Receive request
		
		/*******************************DMA Peripheral****************************/
		// Enable DMA2 clock
		RCC->AHB1ENR |= (1 << RCC_AHB1ENR_DMA1EN_Pos);
		
		//======USART2_RX========//
		// DMA1_Stream6_Channel_4
		// Disable DMA channel
		// NOTE: Only configure the DMA peripheral when DMA is disabled
		DMA1_Stream6->CR &= ~(1 << DMA_SxCR_EN_Pos);
		while (DMA1_Stream6->CR & (1 << DMA_SxCR_EN_Pos));
		
		// Peripheral data size 00 = 8bits
		DMA1_Stream6->CR &= ~(0x03 << DMA_SxCR_PSIZE_Pos);
		
		// Memory data size: 00 = 8 bits
		DMA1_Stream6->CR &= ~(0x03 << DMA_SxCR_MSIZE_Pos);
		
		// Disable peripheral increment mode
		DMA1_Stream6->CR &= ~(1 << DMA_SxCR_PINC_Pos);
		
		// Enable memory increment mode
		DMA1_Stream6->CR |= (1 << DMA_SxCR_MINC_Pos);
		
		// Choose the channel selection (Steam 6 & Channel 4 - USART1_RX)
		DMA1_Stream6->CR &= ~(0x07 << DMA_SxCR_CHSEL_Pos);
		DMA1_Stream6->CR |= (0x04 << DMA_SxCR_CHSEL_Pos);
		
		//======USART2_TX========//
		// DMA1_Stream5_Channel_4
		DMA1_Stream5->CR &= ~(1 << DMA_SxCR_EN_Pos);
		while (DMA1_Stream5->CR & (1 << DMA_SxCR_EN_Pos));
		
		// Peripheral data size 00 = 8bits
		DMA1_Stream5->CR &= ~(0x03 << DMA_SxCR_PSIZE_Pos);
		
		// Memory data size: 00 = 8 bits
		DMA1_Stream5->CR &= ~(0x03 << DMA_SxCR_MSIZE_Pos);
		
		// Disable peripheral increment mode
		DMA1_Stream5->CR &= ~(1 << DMA_SxCR_PINC_Pos);
		
		// Enable memory increment mode
		DMA1_Stream5->CR |= (1 << DMA_SxCR_MINC_Pos);
		
		// Choose the channel selection (Steam 5 & Channel 4 - USART1_TX)
		DMA1_Stream5->CR &= ~(0x07 << DMA_SxCR_CHSEL_Pos);
		DMA1_Stream5->CR |= (0x04 << DMA_SxCR_CHSEL_Pos);
	}
}

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
void USART_Write(USART_Handle_t* pUSARTHandle, uint8_t *buffer, uint32_t nBytes)
{
	if (nBytes == 0)
	{
		return;
	}
	
	uint32_t i;
	for (i = 0; i < nBytes; i++)
	{
		while (!(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE_Pos))); 		// Wait until hardware sets TXE 
		pUSARTHandle->pUSARTx->DR = buffer[i] & 0xFF; 							// Writing to DR clears TXE flag 
	}
	
	// Wait until TC bit is set. TC is set by hardware and cleared by software.
	while (!(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC_Pos))); 		// TC: Transmission complete flag
	
	// Clear the flag TC by software
	pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC_Pos);
}

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
void USART_Read(USART_Handle_t* pUSARTHandle, uint8_t *buffer, uint32_t nBytes)
{
	if (nBytes == 0)
	{
		return;
	}
	
	uint32_t i;
	
	for (i = 0; i < nBytes; i++)
	{
		while (!(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE_Pos))); 		// Wait until hardware sets RXNE 
		buffer[i] = pUSARTHandle->pUSARTx->DR; 									// Reading DR clear RXNE flags
	}
	
	// Add the NULL character
	buffer[i] = '\0';
}

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
void USART_Write_Float_String(USART_Handle_t *pUSARTHandle, float val)
{
	char buffer[32];
	
	// Convert a float value into string following the specified format
	// Ex: val = 12.345678f -> buffer = "12.3457\r\n"
	int len = snprintf(buffer, sizeof(buffer), "%.4f\r\n", val);
	if (len > 0)
	{
		USART_Write_DMA(pUSARTHandle, (uint8_t *)buffer, (uint32_t)len);
	}
}

/**
 * @brief  						- Send a float value as raw binary data (IEEE-754 32-bit single precision).
 *
 * @note   						- Sends 4 bytes directly in Little-Endian format.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  val 					- Float value to be transmitted.
 *
 * @return 						- None
 */
void USART_Write_Float_Bin(USART_Handle_t *pUSARTHandle, float val)
{
	USART_Write_DMA(pUSARTHandle, (uint8_t *)&val, (uint32_t)sizeof(float));
}

/**
 * @brief  						- Read data by using Polling method.
 *
 * @note   						- This funtion allow user to receive data which has any number of bytes
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  buffer 				- Pointer to the data buffer where received data will be stored.
 * @param  nBytes 				- Maximum number of bytes to receive.
 *
 * @return 						- None
 */
void USART_Read_Until_Idle(USART_Handle_t* pUSARTHandle, uint8_t *buffer, uint32_t nBytes)
{
	if (nBytes == 0)
	{
		return;
	}
	
	uint32_t i = 0;
	
	// Wait until the first byte is read
	while (!(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE_Pos)));
	buffer[i++] = pUSARTHandle->pUSARTx->DR;
		
	// Clear the IDLE flag if exist.
	volatile uint32_t dummy = pUSARTHandle->pUSARTx->SR;
    dummy = pUSARTHandle->pUSARTx->DR;
    (void)dummy;

	while(i < nBytes)
	{
		if (pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE_Pos))
		{
			buffer[i++] = pUSARTHandle->pUSARTx->DR;
		}
		
		// Check whether the line is in idle state
		if (pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE_Pos))
		{
			// Clear the IDLE flag
            dummy = pUSARTHandle->pUSARTx->SR;
            dummy = pUSARTHandle->pUSARTx->DR;
			break;		// Stop receving data
		}
	}
	
	// Add the NULL character
	buffer[i] = '\0';
}

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
void USART_Write_IT(USART_Handle_t *pUSARTHandle, uint8_t *buffer, uint32_t nBytes)
{
	if (nBytes == 0)
	{
		return;
	}
	
	uint8_t txState = pUSARTHandle->TxState;
	if (txState != USART_STATE_BUSY)
	{
		pUSARTHandle->TxState = USART_STATE_BUSY;						// Set state to busy
		pUSARTHandle->pTxBuffer = buffer;
		pUSARTHandle->TxLen = nBytes;
	
		// Transmit register empty interrupt 
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE_Pos);
		
		// Write to Transmit Data Register (DR) to start transmission 
		// An interrupt will be initiated after data in DR has been sent. 
		pUSARTHandle->pUSARTx->DR = *buffer;
	}
}

/**
 * @brief  						- Read data by using Interrupt-driven method.
 *
 * @note   						- Blocks until bytes are received or termination character is found.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  buffer 				- Pointer to the data buffer where received data will be stored.
 * @param  nBytes 				- Maximum number of bytes to receive.
 *
 * @return 						- None
 */
void USART_Read_IT(USART_Handle_t *pUSARTHandle, uint8_t *buffer, uint32_t nBytes)
{
	if (nBytes == 0)
	{
		return;
	}
	
	uint8_t rxState = pUSARTHandle->RxState;
	
	if (rxState != USART_STATE_BUSY)
	{
		pUSARTHandle->RxState = USART_STATE_BUSY;						// Set state to busy
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE_Pos);		// Receive Register not empty interrupt
		pUSARTHandle->pRxBuffer = buffer;
		pUSARTHandle->RxLen = nBytes;
	}
}

/**
 * @brief  						- Read data by using Interrupt-driven method.
 *
 * @note   						- This funtion allow user to receive data which has any number of bytes
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  buffer 				- Pointer to the data buffer where received data will be stored.
 * @param  nBytes 				- Maximum number of bytes to receive.
 *
 * @return 						- None
 */
void USART_Read_Until_Idle_IT(USART_Handle_t *pUSARTHandle, uint8_t *buffer, uint32_t nBytes)
{
	if (nBytes == 0)
	{
		return;
	}
	
	uint8_t rxState = pUSARTHandle->RxState;
	
	if (rxState != USART_STATE_BUSY)
	{
		// Set state to busy
		pUSARTHandle->RxState = USART_STATE_BUSY;			

		// Update the variable
		pUSARTHandle->pRxBuffer = buffer;
		pUSARTHandle->RxLen = nBytes;	
		
		// Clear IDLE flag by software sequence: read SR then DR
		volatile uint32_t dummy = pUSARTHandle->pUSARTx->SR;
		dummy = pUSARTHandle->pUSARTx->DR;
		(void)dummy;
		
		// Enable USART IDLE and RXNE interrupt
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_IDLEIE_Pos) | (1 << USART_CR1_RXNEIE_Pos);	

	}
}

/**
 * @brief  						- Write data by using DMA (Direct Memory Access).
 *
 * @note   						- Only USART1, USART2 is supported for DMA in this driver.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  buffer 				- Pointer to the data buffer to be transmitted.
 * @param  nBytes 				- Number of bytes to be transmitted.
 *
 * @return 						- None
 */
void USART_Write_DMA(USART_Handle_t *pUSARTHandle, uint8_t *buffer, uint32_t nBytes)
{
	if (nBytes == 0)
	{
		return;
	}
	
	uint8_t txState = pUSARTHandle->TxState;
	
	// NOTE: Only use DMA for USART1, USART2
	if (txState != USART_STATE_BUSY)
	{
		pUSARTHandle->TxState = USART_STATE_BUSY;	
		pUSARTHandle->TxLen = nBytes;
		pUSARTHandle->pTxBuffer = buffer;
		
		// DMA2_Stream7_Channel_4
		if (pUSARTHandle->pUSARTx == USART1)
		{
			// Number of data to transfer 
			DMA2_Stream7->NDTR = nBytes;
			
			// Peripheral address 
			DMA2_Stream7->PAR = (uint32_t)&(pUSARTHandle->pUSARTx->DR);
			
			// Transmit buffer address 
			DMA2_Stream7->M0AR = (uint32_t)pUSARTHandle->pTxBuffer;
			
			// Transfer direction: to peripheral
			DMA2_Stream7->CR &= ~(0x03 << DMA_SxCR_DIR_Pos);
			DMA2_Stream7->CR |= (0x01 << DMA_SxCR_DIR_Pos);
			
			// Enable interrupt 
			DMA2_Stream7->CR |= (1 << DMA_SxCR_TEIE_Pos) | (1 << DMA_SxCR_HTIE_Pos) | (1 << DMA_SxCR_TCIE_Pos);
			
			// Enable DMA2 Stream 7
			DMA2_Stream7->CR |= (1 << DMA_SxCR_EN_Pos);
		}
		// DMA1_Stream6_Channel_4
		else if (pUSARTHandle->pUSARTx == USART2)
		{
			// Number of data to transfer 
			DMA1_Stream6->NDTR = nBytes;
			
			// Peripheral address 
			DMA1_Stream6->PAR = (uint32_t)&(pUSARTHandle->pUSARTx->DR);
			
			// Transmit buffer address 
			DMA1_Stream6->M0AR = (uint32_t)pUSARTHandle->pTxBuffer;
			
			// Transfer direction: to peripheral
			DMA1_Stream6->CR &= ~(0x03 << DMA_SxCR_DIR_Pos);
			DMA1_Stream6->CR |= (0x01 << DMA_SxCR_DIR_Pos);
			
			// Enable interrupt 
			DMA1_Stream6->CR |= (1 << DMA_SxCR_TEIE_Pos) | (1 << DMA_SxCR_HTIE_Pos) | (1 << DMA_SxCR_TCIE_Pos);
			
			// Enable DMA2 Stream 7
			DMA1_Stream6->CR |= (1 << DMA_SxCR_EN_Pos);
		}
	}
}

/**
 * @brief  						- Read data by using DMA (Direct Memory Access).
 *
 * @note   						- Only USART1, USART2 is supported for DMA in this driver.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 * @param  buffer 				- Pointer to the data buffer where received data will be stored.
 * @param  nBytes 				- Maximum number of bytes to receive.
 *
 * @return 						- None
 */
void USART_Read_DMA(USART_Handle_t *pUSARTHandle, uint8_t *buffer, uint32_t nBytes)
{
	if (nBytes == 0)
	{
		return;
	}
	
	uint8_t rxState = pUSARTHandle->RxState;
	
	// NOTE: Only use DMA for USART1
	if (rxState != USART_STATE_BUSY)
	{	
		pUSARTHandle->RxState = USART_STATE_BUSY;	
		pUSARTHandle->RxLen = nBytes;
		pUSARTHandle->pRxBuffer = buffer;
		
		// DMA2_Stream5_Channel_4
		if (pUSARTHandle->pUSARTx == USART1)
		{
			// Transfer direction: to memory
			DMA2_Stream5->CR &= ~(0x03 << DMA_SxCR_DIR_Pos);
			
			// Number of data to transfer 
			DMA2_Stream5->NDTR = nBytes;
			
			// Peripheral address 
			DMA2_Stream5->PAR = (uint32_t)&(pUSARTHandle->pUSARTx->DR);
			
			// Receive buffer address 
			DMA2_Stream5->M0AR = (uint32_t)pUSARTHandle->pRxBuffer;
			
			// Enable interrupt
			DMA2_Stream5->CR |= (1 << DMA_SxCR_TEIE_Pos) | (1 << DMA_SxCR_HTIE_Pos) | (1 << DMA_SxCR_TCIE_Pos);
			
			// Enable DMA2 Stream 5
			DMA2_Stream5->CR |= (1 << DMA_SxCR_EN_Pos);
		}
		// DMA1_Stream5_Channel_4
		else if (pUSARTHandle->pUSARTx == USART2)
		{
			// Transfer direction: to memory
			DMA1_Stream5->CR &= ~(0x03 << DMA_SxCR_DIR_Pos);
			
			// Number of data to transfer 
			DMA1_Stream5->NDTR = nBytes;
			
			// Peripheral address 
			DMA1_Stream5->PAR = (uint32_t)&(pUSARTHandle->pUSARTx->DR);
			
			// Receive buffer address 
			DMA1_Stream5->M0AR = (uint32_t)pUSARTHandle->pRxBuffer;
			
			// Enable interrupt
			DMA1_Stream5->CR |= (1 << DMA_SxCR_TEIE_Pos) | (1 << DMA_SxCR_HTIE_Pos) | (1 << DMA_SxCR_TCIE_Pos);
			
			// Enable DMA2 Stream 5
			DMA1_Stream5->CR |= (1 << DMA_SxCR_EN_Pos);
		}
	}
}

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
void USART_Read_Until_Idle_DMA(USART_Handle_t *pUSARTHandle, uint8_t *buffer, uint32_t nBytes)
{
	if (nBytes == 0)
	{
		return;
	}
	
	uint8_t rxState = pUSARTHandle->RxState;
	
	// NOTE: Only use DMA for USART1, USART2
	if (rxState != USART_STATE_BUSY)
	{	
		pUSARTHandle->RxState = USART_STATE_BUSY;	
		pUSARTHandle->RxLen = nBytes;
		pUSARTHandle->pRxBuffer = buffer;
		
		// DMA2_Stream5_Channel_4
		if (pUSARTHandle->pUSARTx == USART1)
		{
			// 1. Clear IDLE flag by software sequence: read SR then DR
			volatile uint32_t dummy = pUSARTHandle->pUSARTx->SR;
			dummy = pUSARTHandle->pUSARTx->DR;
			(void)dummy;

			// 2. Disable DMA stream before configuring NDTR
			DMA2_Stream5->CR &= ~(1 << DMA_SxCR_EN_Pos);
			while (DMA2_Stream5->CR & (1 << DMA_SxCR_EN_Pos));

			// Transfer direction: to memory
			DMA2_Stream5->CR &= ~(0x03 << DMA_SxCR_DIR_Pos);
			
			// Number of data to transfer (maximum buffer size)
			DMA2_Stream5->NDTR = nBytes;
			
			// Peripheral address 
			DMA2_Stream5->PAR = (uint32_t)&(pUSARTHandle->pUSARTx->DR);
			
			// Receive buffer address 
			DMA2_Stream5->M0AR = (uint32_t)pUSARTHandle->pRxBuffer;
			
			// Clear leftover DMA interrupt flags
			DMA2->HIFCR |= (1 << DMA_HIFCR_CTCIF5_Pos) | (1 << DMA_HIFCR_CHTIF5_Pos) | (1 << DMA_HIFCR_CTEIF5_Pos);

			// Enable DMA interrupts
			DMA2_Stream5->CR |= (1 << DMA_SxCR_TEIE_Pos) | (1 << DMA_SxCR_HTIE_Pos) | (1 << DMA_SxCR_TCIE_Pos);
			
			// Enable DMA2 Stream 5
			DMA2_Stream5->CR |= (1 << DMA_SxCR_EN_Pos);
		}
		// DMA1_Stream5_Channel_4
		else if (pUSARTHandle->pUSARTx == USART2)
		{
			// 1. Clear IDLE flag by software sequence: read SR then DR
			volatile uint32_t dummy = pUSARTHandle->pUSARTx->SR;
			dummy = pUSARTHandle->pUSARTx->DR;
			(void)dummy;

			// 2. Disable DMA stream before configuring NDTR
			DMA1_Stream5->CR &= ~(1 << DMA_SxCR_EN_Pos);
			while (DMA1_Stream5->CR & (1 << DMA_SxCR_EN_Pos));

			// Transfer direction: to memory
			DMA1_Stream5->CR &= ~(0x03 << DMA_SxCR_DIR_Pos);
			
			// Number of data to transfer (maximum buffer size)
			DMA1_Stream5->NDTR = nBytes;
			
			// Peripheral address 
			DMA1_Stream5->PAR = (uint32_t)&(pUSARTHandle->pUSARTx->DR);
			
			// Receive buffer address 
			DMA1_Stream5->M0AR = (uint32_t)pUSARTHandle->pRxBuffer;
			
			// Clear leftover DMA interrupt flags
			DMA1->HIFCR |= (1 << DMA_HIFCR_CTCIF5_Pos) | (1 << DMA_HIFCR_CHTIF5_Pos) | (1 << DMA_HIFCR_CTEIF5_Pos);

			// Enable DMA interrupts
			DMA1_Stream5->CR |= (1 << DMA_SxCR_TEIE_Pos) | (1 << DMA_SxCR_HTIE_Pos) | (1 << DMA_SxCR_TCIE_Pos);
			
			// Enable DMA1 Stream 5
			DMA1_Stream5->CR |= (1 << DMA_SxCR_EN_Pos);
		}
		
		// 3. Enable USART IDLE interrupt
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_IDLEIE_Pos);
	}
}

/**
 * @brief  						- Get the state of USART conversion.
 *
 * @note   						- None.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 *
 * @return 						- The current state of the USART (e.g., ADC_STATE_IDLE, ADC_STATE_BUSY)
 */
uint8_t USART_Get_State(USART_Handle_t *pUSARTHandle, uint8_t mode)
{	
	if (mode == USART_TX_MODE)
	{
		return pUSARTHandle->TxState;
	}
	else
	{
		return pUSARTHandle->RxState;
	}
}

/**
 * @brief  						- Internal helper to handle receiving data in Interrupt-driven mode.
 *
 * @note   						- None.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure.
 *
 * @return 						- None
 */
static void __USART_Receive(USART_Handle_t *pUSARTHandle)
{
	if (pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE_Pos))						// Check RXNE flag 	
	{
		*pUSARTHandle->pRxBuffer = pUSARTHandle->pUSARTx->DR;						// Reading DR clears the RXNE flag 
		(pUSARTHandle->pRxBuffer)++;
		pUSARTHandle->RxLen--;
		
		if (pUSARTHandle->RxLen == 0)
		{
			*pUSARTHandle->pRxBuffer = '\0';										// Add the NULL character
			pUSARTHandle->RxState = USART_STATE_IDLE;								// Clear the flag RxState to indicate that RX is complete
			pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE_Pos);				// Disable RXNEIE interrupt
		}
	}
}

/**
 * @brief  						- Internal helper to handle transmitting data in Interrupt-driven mode.
 *
 * @note   						- None.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure.
 *
 * @return 						- None
 */
static void __USART_Send(USART_Handle_t *pUSARTHandle)
{
	if (pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE_Pos))					// Check TXE flag 	
	{
		(pUSARTHandle->pTxBuffer)++;			// Bytes that have been sent 
		if (pUSARTHandle->TxLen > 0)			// Transmit the next byte
		{
			pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer) & 0xFF;	// Writing to DR clears TXE 
		}
		else				// Transmission completes
		{
			pUSARTHandle->TxState = USART_STATE_IDLE;						// Clear the flag TxState to indicate that TX is free
			pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE_Pos);		// Disable TXE interrupt
		}
		
		pUSARTHandle->TxLen--;
	}
}

/**
 * @brief  						- Interrupt handling for USART TX/RX.
 *
 * @note   						- None.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 *
 * @return 						- None
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1, temp2;
	
	/**********************************Interrupt-driven******************************/
	// Let's check TXE flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE_Pos);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE_Pos);
	if (temp1 && temp2)
	{
		__USART_Send(pUSARTHandle);
	}
	
	// Let's check RXNE flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE_Pos);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE_Pos);
	if (temp1 && temp2)
	{
		__USART_Receive(pUSARTHandle);
	}
	
	/*******************************IDLE Handling (DMA & Interrupt)********************************/
	// Let's check IDLE flag 
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE_Pos);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE_Pos);
	if (temp1 && temp2)
	{
		// 1. Clear IDLE flag by software sequence: read SR then DR
		volatile uint32_t dummy = pUSARTHandle->pUSARTx->SR;
		dummy = pUSARTHandle->pUSARTx->DR;
		(void)dummy;
		
		// 2. Disable IDLEIE interrupt
		pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_IDLEIE_Pos);
		
		/************************************DMA*****************************/
		if (pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_DMAR_Pos))
		{
			if (pUSARTHandle->pUSARTx == USART1)
			{
				// 3. Process DMA receive completion
				if (pUSARTHandle->RxState == USART_STATE_BUSY)
				{
					// Disable DMA stream interrupts and disable stream
					DMA2_Stream5->CR &= ~((1 << DMA_SxCR_TEIE_Pos) | (1 << DMA_SxCR_HTIE_Pos) | (1 << DMA_SxCR_TCIE_Pos));
					DMA2_Stream5->CR &= ~(1 << DMA_SxCR_EN_Pos);

					// Calculate actual number of bytes received
					uint32_t remaining = DMA2_Stream5->NDTR;
					uint32_t received = pUSARTHandle->RxLen - remaining;

					// Store actual received length back into RxLen
					pUSARTHandle->RxLen = received;

					// Null-terminate the string in buffer if within limits
					pUSARTHandle->pRxBuffer[received] = '\0';

					// Clear DMA flags
					DMA2->HIFCR |= (1 << DMA_HIFCR_CTCIF5_Pos) | (1 << DMA_HIFCR_CHTIF5_Pos) | (1 << DMA_HIFCR_CTEIF5_Pos);
				}
			}
			else if (pUSARTHandle->pUSARTx == USART2)
			{
				// 3. Process DMA receive completion
				if (pUSARTHandle->RxState == USART_STATE_BUSY)
				{
					// Disable DMA stream interrupts and disable stream
					DMA1_Stream5->CR &= ~((1 << DMA_SxCR_TEIE_Pos) | (1 << DMA_SxCR_HTIE_Pos) | (1 << DMA_SxCR_TCIE_Pos));
					DMA1_Stream5->CR &= ~(1 << DMA_SxCR_EN_Pos);

					// Calculate actual number of bytes received
					uint32_t remaining = DMA1_Stream5->NDTR;
					uint32_t received = pUSARTHandle->RxLen - remaining;

					// Store actual received length back into RxLen
					pUSARTHandle->RxLen = received;

					// Null-terminate the string in buffer if within limits
					pUSARTHandle->pRxBuffer[received] = '\0';

					// Clear DMA flags
					DMA1->HIFCR |= (1 << DMA_HIFCR_CTCIF5_Pos) | (1 << DMA_HIFCR_CHTIF5_Pos) | (1 << DMA_HIFCR_CTEIF5_Pos);
				}
			}
		}
		/*******************************Interrupt******************************/
		else
		{
			(pUSARTHandle->pRxBuffer)++;
			*pUSARTHandle->pRxBuffer = '\0'; 								// Add the NULL character
			pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE_Pos); 	// Disable RXNE interrupt
		}
		
		// Mark RX as IDLE (complete)
		pUSARTHandle->RxState = USART_STATE_IDLE;
	}
}

/**
 * @brief  						- Interrupt handling for USART1 DMA.
 *
 * @note   						- None.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 *
 * @return 						- None
 */
void USART1_DMA_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1, temp2;
	/**************************************USART1_TX***************************************/
	// Let's check Transfer complete (TCIE)
	temp1 = DMA2->HISR & (1 << DMA_HISR_TCIF7_Pos);
	temp2 = DMA2_Stream7->CR & (1 << DMA_SxCR_TCIE_Pos); 
	if (temp1 && temp2)
	{
		pUSARTHandle->TxState = USART_STATE_IDLE;		// Clear the flag TxState to indicate that TX is free
		DMA2->HIFCR |= 1 << DMA_HIFCR_CTCIF7_Pos;		// writing 1 to the corresponding bit in the DMA_HIFCR register to clear the corresponding TCIF flag 
		DMA2_Stream7->CR &= ~((1 << DMA_SxCR_TEIE_Pos) | (1 << DMA_SxCR_HTIE_Pos) | (1 << DMA_SxCR_TCIE_Pos));		// Disable interrupt
		DMA2_Stream7->CR &= ~(1 << DMA_SxCR_EN_Pos);	// Disable DMA2_Stream7
	}
	
	// Let's check Transfer complete (HTIIE)
	temp1 = DMA2->HISR & (1 << DMA_HISR_HTIF7_Pos);
	temp2 = DMA2_Stream7->CR & (1 << DMA_SxCR_HTIE_Pos); 
	if (temp1 && temp2)
	{
		DMA2->HIFCR |= 1 << DMA_HIFCR_CHTIF7_Pos; 		// writing 1 to the corresponding bit in the DMA_HIFCR register to clear the corresponding TCIF flag 
	}
	
	// Let's check Transfer Error (HTIIE)
	temp1 = DMA2->HISR & (1 << DMA_HISR_TEIF7_Pos);
	temp2 = DMA2_Stream7->CR & (1 << DMA_SxCR_TEIE_Pos); 
	if (temp1 && temp2)
	{
		DMA2->HIFCR |= 1 << DMA_HIFCR_CTEIF7_Pos;		// writing 1 to the corresponding bit in the DMA_HIFCR register to clear the corresponding TCIF flag 
		
		// Transfer data again
		if (pUSARTHandle->TxState == USART_STATE_BUSY)
		{
			pUSARTHandle->TxState = USART_STATE_IDLE;	// Clear the flag TxState to indicate that TX is free
			USART_Write_DMA(pUSARTHandle, pUSARTHandle->pTxBuffer, pUSARTHandle->TxLen);
		}
	}
	
	/**************************************USART1_RX***************************************/
	// Let's check Transfer complete (TEIE)
	temp1 = DMA2->HISR & (1 << DMA_HISR_TCIF5_Pos);
	temp2 = DMA2_Stream5->CR & (1 << DMA_SxCR_TCIE_Pos); 
	if (temp1 && temp2)
	{
		pUSARTHandle->RxState = USART_STATE_IDLE;		// Clear the flag RxState to indicate that RX is free
		DMA2->HIFCR |= 1 << DMA_HIFCR_CTCIF5_Pos;		// writing 1 to the corresponding bit in the DMA_HIFCR register to clear the corresponding TCIF flag 
		DMA2_Stream5->CR &= ~((1 << DMA_SxCR_TEIE_Pos) | (1 << DMA_SxCR_HTIE_Pos) | (1 << DMA_SxCR_TCIE_Pos));		// Disable interrupt
		DMA2_Stream5->CR &= ~(1 << DMA_SxCR_EN_Pos);	// Disable DMA2_Stream5

		// Disable IDLE interrupt if it was enabled
		if (pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE_Pos))
		{
			pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_IDLEIE_Pos);
			// Clear IDLE flag by software sequence: read SR then DR
			volatile uint32_t dummy = pUSARTHandle->pUSARTx->SR;
			dummy = pUSARTHandle->pUSARTx->DR;
			(void)dummy;
		}
	}
	
	/****************************************DMA*********************************************/
	// Let's check Half Transfer complete (HTIIE)
	temp1 = DMA2->HISR & (1 << DMA_HISR_HTIF5_Pos);
	temp2 = DMA2_Stream5->CR & (1 << DMA_SxCR_HTIE_Pos); 
	if (temp1 && temp2)
	{
		DMA2->HIFCR |= 1 << DMA_HIFCR_CHTIF5_Pos; 		// writing 1 to the corresponding bit in the DMA_HIFCR register to clear the corresponding TCIF flag 
	}
	
	// Let's check Transfer Error (TIIE)
	temp1 = DMA2->HISR & (1 << DMA_HISR_TEIF5_Pos);
	temp2 = DMA2_Stream5->CR & (1 << DMA_SxCR_TEIE_Pos); 
	if (temp1 && temp2)
	{
		DMA2->HIFCR |= 1 << DMA_HIFCR_CTEIF5_Pos;		// writing 1 to the corresponding bit in the DMA_HIFCR register to clear the corresponding TCIF flag 
		
		// Transfer data again
		if (pUSARTHandle->RxState == USART_STATE_BUSY)
		{
			pUSARTHandle->RxState = USART_STATE_IDLE;	// Clear the flag TxState to indicate that TX is free
			USART_Read_DMA(pUSARTHandle, pUSARTHandle->pRxBuffer, pUSARTHandle->RxLen);
		}
	}
}

/**
 * @brief  						- Interrupt handling for USART2 DMA.
 *
 * @note   						- None.
 *
 * @param  pUSARTHandle 		- Pointer to a USART_Handle_t structure that contains the configuration information for the specified USART.
 *
 * @return 						- None
 */
void USART2_DMA_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1, temp2;
	/**************************************USART2_TX***************************************/
	// DMA1_Stream6_Channel4
	// Let's check Transfer complete (TCIE)
	temp1 = DMA1->HISR & (1 << DMA_HISR_TCIF6_Pos);
	temp2 = DMA1_Stream6->CR & (1 << DMA_SxCR_TCIE_Pos); 
	if (temp1 && temp2)
	{
		pUSARTHandle->TxState = USART_STATE_IDLE;		// Clear the flag TxState to indicate that TX is free
		DMA1->HIFCR |= 1 << DMA_HIFCR_CTCIF6_Pos;		// writing 1 to the corresponding bit in the DMA_HIFCR register to clear the corresponding TCIF flag 
		DMA1_Stream6->CR &= ~((1 << DMA_SxCR_TEIE_Pos) | (1 << DMA_SxCR_HTIE_Pos) | (1 << DMA_SxCR_TCIE_Pos));		// Disable interrupt
		DMA1_Stream6->CR &= ~(1 << DMA_SxCR_EN_Pos);	// Disable DMA2_Stream7
	}
	
	// Let's check Transfer complete (HTIIE)
	temp1 = DMA1->HISR & (1 << DMA_HISR_HTIF6_Pos);
	temp2 = DMA1_Stream6->CR & (1 << DMA_SxCR_HTIE_Pos); 
	if (temp1 && temp2)
	{
		DMA1->HIFCR |= 1 << DMA_HIFCR_CHTIF6_Pos; 		// writing 1 to the corresponding bit in the DMA_HIFCR register to clear the corresponding TCIF flag 
	}
	
	// Let's check Transfer Error (HTIIE)
	temp1 = DMA1->HISR & (1 << DMA_HISR_TEIF6_Pos);
	temp2 = DMA1_Stream6->CR & (1 << DMA_SxCR_TEIE_Pos); 
	if (temp1 && temp2)
	{
		DMA1->HIFCR |= 1 << DMA_HIFCR_CTEIF6_Pos;		// writing 1 to the corresponding bit in the DMA_HIFCR register to clear the corresponding TCIF flag 
		
		// Transfer data again
		if (pUSARTHandle->TxState == USART_STATE_BUSY)
		{
			pUSARTHandle->TxState = USART_STATE_IDLE;	// Clear the flag TxState to indicate that TX is free
			USART_Write_DMA(pUSARTHandle, pUSARTHandle->pTxBuffer, pUSARTHandle->TxLen);
		}
	}
	
	/**************************************USART2_RX***************************************/
	// DMA1_Stream5_Channel4
	// Let's check Transfer complete (TEIE)
	temp1 = DMA1->HISR & (1 << DMA_HISR_TCIF5_Pos);
	temp2 = DMA1_Stream5->CR & (1 << DMA_SxCR_TCIE_Pos); 
	if (temp1 && temp2)
	{
		pUSARTHandle->RxState = USART_STATE_IDLE;		// Clear the flag RxState to indicate that RX is free
		DMA1->HIFCR |= 1 << DMA_HIFCR_CTCIF5_Pos;		// writing 1 to the corresponding bit in the DMA_HIFCR register to clear the corresponding TCIF flag 
		DMA1_Stream5->CR &= ~((1 << DMA_SxCR_TEIE_Pos) | (1 << DMA_SxCR_HTIE_Pos) | (1 << DMA_SxCR_TCIE_Pos));		// Disable interrupt
		DMA1_Stream5->CR &= ~(1 << DMA_SxCR_EN_Pos);	// Disable DMA2_Stream5

		// Disable IDLE interrupt if it was enabled
		if (pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE_Pos))
		{
			pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_IDLEIE_Pos);
			// Clear IDLE flag by software sequence: read SR then DR
			volatile uint32_t dummy = pUSARTHandle->pUSARTx->SR;
			dummy = pUSARTHandle->pUSARTx->DR;
			(void)dummy;
		}
	}
	
	/****************************************DMA*********************************************/
	// Let's check Half Transfer complete (HTIIE)
	temp1 = DMA1->HISR & (1 << DMA_HISR_HTIF5_Pos);
	temp2 = DMA1_Stream5->CR & (1 << DMA_SxCR_HTIE_Pos); 
	if (temp1 && temp2)
	{
		DMA1->HIFCR |= 1 << DMA_HIFCR_CHTIF5_Pos; 		// writing 1 to the corresponding bit in the DMA_HIFCR register to clear the corresponding TCIF flag 
	}
	
	// Let's check Transfer Error (TIIE)
	temp1 = DMA1->HISR & (1 << DMA_HISR_TEIF5_Pos);
	temp2 = DMA1_Stream5->CR & (1 << DMA_SxCR_TEIE_Pos); 
	if (temp1 && temp2)
	{
		DMA1->HIFCR |= 1 << DMA_HIFCR_CTEIF5_Pos;		// writing 1 to the corresponding bit in the DMA_HIFCR register to clear the corresponding TCIF flag 
		
		// Transfer data again
		if (pUSARTHandle->RxState == USART_STATE_BUSY)
		{
			pUSARTHandle->RxState = USART_STATE_IDLE;	// Clear the flag TxState to indicate that TX is free
			USART_Read_DMA(pUSARTHandle, pUSARTHandle->pRxBuffer, pUSARTHandle->RxLen);
		}
	}
}