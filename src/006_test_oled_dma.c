/*
	This program tests the OLED SSD1306 128x64 display using I2C with DMA transfer.
	Received UART data (via USART1 interrupt mode) is displayed on the OLED screen.
	Programming method: OLED via I2C DMA, USART RX via Interrupt (Idle detection)
	
	Source: Nguyen Hai Nam
*/

#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_lib.h"
#include "pin_config.h"

#define MAX_BUFFER 		100UL

volatile uint8_t rcvBuffer[MAX_BUFFER];
USART_Handle_t USARTHandle;

void GPIOUSART_Init()
{
	// PB.6 = AF7 (USART1_TX), PB.7 = AF7 (USART1_RX)
	// Enable peripheral clock
	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOBEN_Pos);
	
	// Choose alternate function for GPIO
	GPIOB->MODER &= ~(0x03 << 12);
	GPIOB->MODER |= (0x02 << 12);
	GPIOB->MODER &= ~(0x03 << 14);
	GPIOB->MODER |= (0x02 << 14);
	
	// Select alternate function
	GPIOB->AFR[0] &= ~(0x0F << 24);
	GPIOB->AFR[0] |= (0x07 << 24);		// AF7 for PB6
	
	GPIOB->AFR[0] &= ~(0x0F << 28);
	GPIOB->AFR[0] |= (0x07 << 28);    	// AF7 for PB7
	
	// Configure the speed, output type, pullup-pulldown	
	// Output push-pull
	GPIOB->OTYPER &= ~((1 << 6) | (1 << 7));
	
	// Speed: Very High
	GPIOB->OSPEEDR &= ~((0x03 << 12) | (0x03 << 14));
	GPIOB->OSPEEDR |= (0x03 << 12) | (0x03 << 14);
	
	// No pull-up, pull-down
	GPIOB->PUPDR &= ~((0x03 << 12) | (0x03 << 14));	
}

void USART1_Init()
{
	GPIOUSART_Init();
	
	// Enable UART 1 clock
	RCC->APB2ENR |= (1 << RCC_APB2ENR_USART1EN_Pos);
	
	USARTHandle.pUSARTx = USART1;
	USARTHandle.config.baudrate = 115200;
	USARTHandle.config.oversampling8 = 0;
	USART_Init(&USARTHandle);
	USART1_DMA_Init(&USARTHandle);
}

void USART1_IRQHandler(void)
{
	USART_IRQHandling(&USARTHandle);
}

// Interrupt for I2C2_Tx
void DMA1_Stream7_IRQHandler(void)
{
	OLED_DMA_IRQ_Handler();
}

int main()
{
	/******************************Inits*************************/
	// Initialize Pins
	Pin_Config_OLED();

	// Initialize USART
	USART1_Init();
	
	// Initialize OLED Display
	OLED_Init(NULL);
	OLED_DMA_Init(NULL);
	OLED_Clear_Buffer();
	OLED_Print_String("SYSTEM READY", 25, 0, OLED_COLOR_WHITE);
	OLED_Print_String("Received: ", 0, 20, OLED_COLOR_WHITE);
	OLED_Update_Display_DMA(); 
	
	// Enable the interrupt for USART
	NVIC_SetPriority(USART1_IRQn, 0);
	NVIC_EnableIRQ(USART1_IRQn);

	// Enable the interrupt for DMA1_Stream7 (I2C2_TX - OLED)
	NVIC_SetPriority(DMA1_Stream7_IRQn, 1);
	NVIC_EnableIRQ(DMA1_Stream7_IRQn);
	
	/****************************Loop*************************/
	USART_Read_Until_Idle_IT(&USARTHandle, (uint8_t*)rcvBuffer, MAX_BUFFER);
	while(1)
	{
		uint8_t rxState = USART_Get_State(&USARTHandle, USART_RX_MODE);
		if (rxState == USART_STATE_IDLE)
		{
			USART_Write(&USARTHandle, (uint8_t*)rcvBuffer, strlen((char*)rcvBuffer));
			if (OLED_Get_State_DMA() == OLED_DMA_STATE_IDLE)
			{
				OLED_Clear_Buffer();
				OLED_Print_String("SYSTEM READY", 25, 0, OLED_COLOR_WHITE);
				OLED_Print_String("Received: ", 0, 20, OLED_COLOR_WHITE);
				OLED_Print_String((char*)rcvBuffer, 0, 36, OLED_COLOR_WHITE);
				OLED_Update_Display_DMA(); 
			}
			
			USART_Read_Until_Idle_IT(&USARTHandle, (uint8_t*)rcvBuffer, MAX_BUFFER);
		}
	}
}