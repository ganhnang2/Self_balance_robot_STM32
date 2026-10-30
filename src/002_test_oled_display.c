/*
	This program has instructed to configure UART protocol to interface with computer through CH340
	Programming method: DMA
	
	Source: Nguyen Hai Nam
*/

#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_lib.h"
#include "pin_config.h"

#define MAX_BUFFER 		100UL

volatile uint8_t buffer[] = "Hello. This is a message from STM32!\r\n";
volatile uint8_t rcvBuffer[MAX_BUFFER];
USART_Handle_t USARTHandle;
I2C_Handle_t I2CHandle;

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

// Interrupt for USART1_DMA2_Tx
void DMA2_Stream7_IRQHandler(void)
{
	USART1_DMA_IRQHandling(&USARTHandle);
}

// Interrupt for USART1_DMA2_Rx
void DMA2_Stream5_IRQHandler(void)
{
	USART1_DMA_IRQHandling(&USARTHandle);
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
	OLED_Clear_Buffer();
	OLED_Print_String("SYSTEM READY", 25, 0, OLED_COLOR_WHITE);
	OLED_Print_String("Received: ", 0, 20, OLED_COLOR_WHITE);
	OLED_Update_Display(); 
	
	// Enable the interrupt for USART
	NVIC_SetPriority(USART1_IRQn, 0);
	NVIC_EnableIRQ(USART1_IRQn);

	// Enable the interrupt for USART1_DMA2
	NVIC_SetPriority(DMA2_Stream7_IRQn, 2);			// Tx
	NVIC_SetPriority(DMA2_Stream5_IRQn, 1);			// Rx
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);
	NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	
	/****************************Loop*************************/
	uint8_t rxState;
	USART_Read_Until_Idle_DMA(&USARTHandle, (uint8_t*)rcvBuffer, MAX_BUFFER);
	while(1)
	{
		OLED_Print_String("Received: ", 0, 0, OLED_COLOR_WHITE);
		rxState = USART_Get_State(&USARTHandle, USART_RX_MODE);
		if (rxState == USART_STATE_IDLE)
		{
			OLED_Clear_Buffer();
			OLED_Print_String("SYSTEM READY", 25, 0, OLED_COLOR_WHITE);
			OLED_Print_String("Received: ", 0, 20, OLED_COLOR_WHITE);
			OLED_Print_String((char*)rcvBuffer, 0, 36, OLED_COLOR_WHITE);
			OLED_Update_Display(); 	
			printf("Received: %s\r\n", (char*)rcvBuffer);
			USART_Read_Until_Idle_DMA(&USARTHandle, (uint8_t*)rcvBuffer, MAX_BUFFER);
		}
	}
}