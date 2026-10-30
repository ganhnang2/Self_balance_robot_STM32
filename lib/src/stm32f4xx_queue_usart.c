#include "stm32f4xx_queue_usart.h"

uint8_t Queue_Push_Cmd(CommandQueue_t* cmdQueue, const char *cmd)
{
	// Queue is full (Overflow), so ignore current command
	if (cmdQueue->count >= CMD_QUEUE_SIZE) 
	{
        return false; 
    }
	
	// Copy command to head position
    strncpy((char *)cmdQueue->data[cmdQueue->head], cmd, CMD_MAX_LEN - 1);
	
	// Always ensure that the last character is null-terminator ('\0')
    cmdQueue->data[cmdQueue->head][CMD_MAX_LEN - 1] = '\0'; 
	
	// Update head pointer and the number of commands
    cmdQueue->head = (cmdQueue->head + 1) % CMD_QUEUE_SIZE;
    cmdQueue->count++;
    return true;
}

uint8_t Queue_Pop_Cmd(CommandQueue_t* cmdQueue, char *outCmd)
{
	// Queue is empty, no command
	if (cmdQueue->count == 0) {
        return false; 
    }
	
	// Pop command from tail pointer 
    strncpy(outCmd, (char *)cmdQueue->data[cmdQueue->tail], CMD_MAX_LEN);

	// Disable interrupt or atomic operation when decrease count variable
    __disable_irq();
    cmdQueue->tail = (cmdQueue->tail + 1) % CMD_QUEUE_SIZE;
    cmdQueue->count--;
    __enable_irq();
	
    return true;
}