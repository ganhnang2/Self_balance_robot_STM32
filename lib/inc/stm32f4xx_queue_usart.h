/**
 * @file		- stm32f4xx_queue_usart.h
 * @author		- Nam Nguyen Hai
 * @brief		- Implementing queue data structure to store commands received from computer 
 * @note		-
 */
#ifndef STM32F4XX_QUEUE_USART_H_
#define STM32F4XX_QUEUE_USART_H_

#include "stm32f401xc.h"
#include "core_cm4.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define CMD_QUEUE_SIZE   8   	// Maximum size: 8 
#define CMD_MAX_LEN      64 	// Maximum length of cmd: 64 bytes (63 data bytes + 1 null byte)

typedef struct {
    char data[CMD_QUEUE_SIZE][CMD_MAX_LEN];
    volatile uint8_t head;  	// Head (Producer)
    volatile uint8_t tail;  	// Tail (Consumer)
    volatile uint8_t count;  	// The number of command need to be handled
} CommandQueue_t;

uint8_t Queue_Push_Cmd(CommandQueue_t* queue, const char *cmd);
uint8_t Queue_Pop_Cmd(CommandQueue_t* queue, char *outCmd);

#endif