/**
 * @file    stm32f4xx_hc05.c
 * @author  Nguyen Hai Nam
 * @brief   Source file for HC-05 Bluetooth Module Driver on STM32F4xx.
 * 
 * @note
 *   - The HC05_Init() function takes a pointer to an already-initialized USART_Handle_t instance:
 *     + If NULL is passed: The driver automatically initializes and uses the default USART2 peripheral 
 *       (9600 baud) and executes AT commands for configuration. GPIO pins (PA1, PA2, PA3) must still 
 *       be configured beforehand (e.g. via Pin_Config_HC05()).
 *     + If non-NULL is passed: The user MUST configure GPIO pins and initialize the USART peripheral 
 *       (via USART_Init()) externally beforehand. The driver will NOT call USART_Init() internally, 
 *       maintaining consistent initialization behavior across all lib modules (MPU6050, OLED, HC05).
 */

#include <stdlib.h>
#include "stm32f4xx_hc05.h"


static void HC05_Parse_Frame(HC05_Device_t* dev, uint8_t* buffer)
{
    if (dev == NULL || buffer == NULL)
    {
        return;
    }

    // Case 1: Receive message as Kp, Ki, Kd parameters for 3 PID loops
    // Formats:
    // - Balance:  "BAL:Kp=35.0\r\n", "BAL:Ki=0.0\r\n", "BAL:Kd=1.2\r\n"
    // - Velocity: "VEL:Kp=0.15\r\n", "VEL:Ki=0.05\r\n", "VEL:Kd=0.0\r\n"
    // - Steering: "STR:Kp=1.50\r\n", "STR:Ki=0.0\r\n", "STR:Kd=0.1\r\n"
    // - Legacy:   "PID:Kp=35.0\r\n" (default to Balance)
    HC05_PID_Target_t target = PID_TARGET_NONE;
    char* paramStr = NULL;

    if (strncmp((char*)buffer, "BAL:K", 5) == 0)
    {
        target = PID_TARGET_BALANCE;
        paramStr = (char*)buffer + 4;
    }
    else if (strncmp((char*)buffer, "VEL:K", 5) == 0)
    {
        target = PID_TARGET_VELOCITY;
        paramStr = (char*)buffer + 4;
    }
    else if (strncmp((char*)buffer, "STR:K", 5) == 0)
    {
        target = PID_TARGET_STEERING;
        paramStr = (char*)buffer + 4;
    }
    else if (strncmp((char*)buffer, "PID:K", 5) == 0)
    {
        target = PID_TARGET_BALANCE;
        paramStr = (char*)buffer + 4;
    }

    if (target != PID_TARGET_NONE && paramStr != NULL)
    {
        uint8_t parseSuccess = 0;
        if (paramStr[2] == '=')
        {
            float val = (float)atof(paramStr + 3);
            char type = paramStr[1];
            if (type == 'p' || type == 'P')
            {
                dev->pid.Kp = val;
                dev->pid.target = target;
                dev->pid.isNew = 1;
                parseSuccess = 1;
            }
            else if (type == 'i' || type == 'I')
            {
                dev->pid.Ki = val;
                dev->pid.target = target;
                dev->pid.isNew = 1;
                parseSuccess = 1;
            }
            else if (type == 'd' || type == 'D')
            {
                dev->pid.Kd = val;
                dev->pid.target = target;
                dev->pid.isNew = 1;
                parseSuccess = 1;
            }
        }

        // Send feedback to computer 
        HC05_Send_ACK_Frame(dev, parseSuccess);
    }
    // Case 2: Computer sends direction for robot
    // Format: "DIR:FORWARD", "DIR:REVERSED", "DIR:LEFT", "DIR:RIGHT", or "DIR:F", "DIR:1", etc.
    else if (strncmp((char*)buffer, "DIR:", 4) == 0)
    {
        char* d = (char*)buffer + 4;
        if (strncmp(d, "FORWARD", 7) == 0 || d[0] == 'F' || d[0] == 'f' || d[0] == '1')
        {
            dev->direction.current = DIR_FORWARD;
            dev->direction.isNew = 1;
        }
        else if (strncmp(d, "REVERSED", 8) == 0 || strncmp(d, "BACKWARD", 8) == 0 || d[0] == 'B' || d[0] == 'b' || d[0] == '2')
        {
            dev->direction.current = DIR_REVERSED;
            dev->direction.isNew = 1;
        }
        else if (strncmp(d, "LEFT", 4) == 0 || d[0] == 'L' || d[0] == 'l' || d[0] == '3')
        {
            dev->direction.current = DIR_LEFT;
            dev->direction.isNew = 1;
        }
        else if (strncmp(d, "RIGHT", 5) == 0 || d[0] == 'R' || d[0] == 'r' || d[0] == '4')
        {
            dev->direction.current = DIR_RIGHT;
            dev->direction.isNew = 1;
        }
        else
        {
            dev->direction.current = DIR_NONE;
            dev->direction.isNew = 1;
        }
    }
}

void HC05_Send_AT_Command(HC05_Device_t* dev, const char* cmd)
{
    if (dev == NULL || cmd == NULL)
    {
        return;
    }
    USART_Write(&dev->usart, (uint8_t*)cmd, strlen(cmd));
}

/**
 * @brief   Initialize the HC-05 module device structure and receive queue.
 * @param   dev         Pointer to HC05_Device_t instance.
 * @param   usartConfig Pointer to an ALREADY-INITIALIZED USART_Handle_t, or NULL.
 * @note    - If usartConfig is NULL: The driver automatically initializes default USART2 
 *            (9600 baud) and sends AT commands. GPIO pins (PA1, PA2, PA3) must be configured 
 *            beforehand (e.g., via Pin_Config_HC05()).
 *          - If usartConfig is provided: The user MUST configure GPIO pins and call USART_Init() 
 *            externally beforehand. The driver will NOT call USART_Init() internally.
 * @retval  None
 */
void HC05_Init(HC05_Device_t* dev, USART_Handle_t* usartConfig)
{
    if (dev == NULL)
    {
        return;
    }

    if (usartConfig == NULL)
    {
        // Configure default USART2 (GPIO pins PA1, PA2, PA3 configured via Pin_Config_HC05)
        RCC->APB1ENR |= (1 << RCC_APB1ENR_USART2EN_Pos);
        dev->usart.pUSARTx = USART2;
        dev->usart.config.baudrate = 115200;
        dev->usart.config.oversampling8 = 0;
        USART_Init(&dev->usart);
		USART2_DMA_Init(&dev->usart);
        
        // 4. Configure HC-05 module via AT commands
        // Pull KEY pin HIGH to enter AT command mode
        GPIOA->ODR |= (1 << GPIO_ODR_OD1_Pos);

        HC05_Send_AT_Command(dev, "AT\r\n");
        HC05_Send_AT_Command(dev, "AT+ROLE=0\r\n");
        HC05_Send_AT_Command(dev, "AT+NAME=HC-05\r\n");
        HC05_Send_AT_Command(dev, "AT+PSWD=123456\r\n");	
        HC05_Send_AT_Command(dev, "AT+UART=115200,0,0\r\n");
        HC05_Send_AT_Command(dev, "AT+RESET\r\n");

        // Pull KEY pin LOW to return to transparent data mode
        GPIOA->ODR &= ~(1 << GPIO_ODR_OD1_Pos);
    }
    else
    {
        // Use external USART configuration (GPIO and USART_Init() must be configured externally)
        dev->usart = *usartConfig;
    }
    
    // 5. Initialize internal queue and enable interrupt reception
    dev->queue.head = 0;
    dev->queue.tail = 0;
    dev->queue.count = 0;
    dev->direction.current = DIR_NONE;
    dev->direction.isNew = 0;
    dev->pid.isNew = 0;
    dev->pid.target = PID_TARGET_NONE;
    memset(dev->rxBuffer, 0, sizeof(dev->rxBuffer));
    USART_Read_Until_Idle_IT(&dev->usart, (uint8_t*)dev->rxBuffer, HC05_RX_BUFFER_SIZE);
}

uint8_t HC05_Send_ACK_Frame(HC05_Device_t* dev, uint8_t isOk)
{
    if (dev == NULL)
    {
        return 0;
    }

    const char *msg = isOk ? "ACK:OK\r\n" : "ACK:FAIL\r\n";
    uint32_t len    = isOk ? 8U : 10U;
    USART_Write(&dev->usart, (uint8_t*)msg, len);
    return (uint8_t)len;
}

uint8_t HC05_Send_IMU_Frame(HC05_Device_t* dev, float targetAngle, float pitch, float ax, float ay, float az, float gx, float gy, float gz)
{
    if (dev == NULL)
    {
        return 0;
    }

    uint8_t state = 0;
    int len = snprintf(dev->txBuffer, (size_t)HC05_TX_BUFFER_SIZE, "IMU:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", targetAngle, pitch, ax, ay, az, gx, gy, gz);
    
    if (len > 0 && (uint32_t)len < HC05_TX_BUFFER_SIZE)
    {
        USART_Write(&dev->usart, (uint8_t*)dev->txBuffer, (uint32_t)len);
        state = (uint8_t)len;
    }
    
    return state;
}

uint8_t HC05_Receive_Frame(HC05_Device_t* dev, uint8_t* buffer)
{
    if (dev == NULL)
    {
        return false;
    }

    char temp_buf[CMD_MAX_LEN];
    char* target = (buffer != NULL) ? (char*)buffer : temp_buf;

    if (Queue_Pop_Cmd(&dev->queue, target) == true)
    {
        dev->pid.isNew = 0;
        dev->pid.target = PID_TARGET_NONE;
        dev->direction.isNew = 0;

        HC05_Parse_Frame(dev, (uint8_t*)target);
        return true;
    }

    return false;
}

void HC05_Get_Tuning_Parameters(HC05_Device_t* dev, float* Kp, float* Ki, float* Kd)
{
    if (dev == NULL)
    {
        return;
    }

    if (Kp != NULL)
    {
        *Kp = dev->pid.Kp;
    }
    if (Ki != NULL)
    {
        *Ki = dev->pid.Ki;
    }
    if (Kd != NULL)
    {
        *Kd = dev->pid.Kd;
    }
    dev->pid.isNew = 0;
    dev->pid.target = PID_TARGET_NONE;
}

HC05_PID_Target_t HC05_Get_PID_Target(HC05_Device_t* dev)
{
    if (dev == NULL)
    {
        return PID_TARGET_NONE;
    }
    return dev->pid.target;
}

HC05_DirectionCode_t HC05_Get_Direction(HC05_Device_t* dev)
{
    if (dev == NULL)
    {
        return DIR_NONE;
    }

    dev->direction.isNew = 0;
    return dev->direction.current;
}

uint8_t HC05_Is_Parameter_New(HC05_Device_t* dev)
{
    if (dev == NULL)
    {
        return 0;
    }
    return dev->pid.isNew;
}

uint8_t HC05_Is_Direction_New(HC05_Device_t* dev)
{
    if (dev == NULL)
    {
        return 0;
    }
    return dev->direction.isNew;
}

// This handler must be invoked inside the corresponding USART IRQHandler by the application
void HC05_IRQ_Handler(HC05_Device_t* dev)
{
    if (dev == NULL)
    {
        return;
    }

    uint8_t prevState = (USART_Get_State(&dev->usart, USART_RX_MODE) == USART_STATE_BUSY);
    
    USART_IRQHandling(&dev->usart);
    
    uint8_t currState = (USART_Get_State(&dev->usart, USART_RX_MODE) == USART_STATE_IDLE);
    
    // When a frame transfer completes (IDLE line detected), state transitions from BUSY to IDLE
    if (prevState && currState)
    {
        // 1. Push received string into queue if not empty
        if (dev->rxBuffer[0] != '\0')
        {
            Queue_Push_Cmd(&dev->queue, dev->rxBuffer);
        }
        
        // 2. Clear buffer and re-arm reception for the next incoming frame
        memset(dev->rxBuffer, 0, sizeof(dev->rxBuffer));
        USART_Read_Until_Idle_IT(&dev->usart, (uint8_t*)dev->rxBuffer, HC05_RX_BUFFER_SIZE);
    }
}

// This handler must be invoked inside the corresponding DMA_Stream Handler by the application
void HC05_DMA_IRQHandler(HC05_Device_t* dev)
{
	if (dev == NULL)
	{
		return;
	}
	
	USART2_DMA_IRQHandling(&dev->usart);
}