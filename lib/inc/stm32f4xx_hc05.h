/**
 * @file        stm32f4xx_hc05.h
 * @author      Nguyen Hai Nam
 * @brief       HC-05 Bluetooth Module Driver (UART Bridge) for STM32F4xx
 * 
 * @details     This library provides an interface to communicate with an HC-05 
 *              Bluetooth module via USART2.
 * 
 *              Hardware Connections:
 *              - PA2 : USART2_TX (Connects to HC-05 RXD)
 *              - PA3 : USART2_RX (Connects to HC-05 TXD)
 *              - PA1 : HC05_EN   (KEY pin for AT mode / Data mode switching)
 * 
 *              Frame Structures:
 *              Format: `{Header}:{Data}\r\n`
 *              
 *              1. PC -> STM32 (Commands received):
 *                 - PID values: `PID:Kp,Ki,Kd` (e.g., "PID:12.67,1.05,7.56")
 *				   - Kp value: "PID:Kp=5.670"
 *                 - Ki value: "PID:Ki=5.670"
 *				   - Kd value: "PID:Kd=5.670"
 *              2. STM32 -> PC (Data transmitted):
 *                 - IMU data: `IMU:Pitch,Roll,Ax,Ay,Az,Gx,Gy,Gz`
 *                   (e.g., "IMU:67.800,20.560,0.680,0.340,0.000,6.780,-9.560,-4.340\r\n")
 *                 - Acknowledgments: `ACK:OK\r\n` or `ACK:FAIL\r\n`
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
#ifndef STM32F4XX_HC05_H_
#define STM32F4XX_HC05_H_

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "stm32f4xx_queue_usart.h"

/** 
 * @brief  Maximum size of the RX buffer (bytes)
 * @note   Used for receiving data from HC-05
 */
#define HC05_RX_BUFFER_SIZE     20U

/** 
 * @brief  Maximum size of the TX buffer (bytes)
 * @note   Used for transmitting data to HC-05
 */
#define HC05_TX_BUFFER_SIZE     128U

/* ==================== HC-05 Direction Enumeration ==================== */
typedef enum {
    DIR_NONE        = 0,
    DIR_FORWARD     = 1,
    DIR_REVERSED    = 2,
    DIR_LEFT        = 3,
    DIR_RIGHT       = 4
} HC05_DirectionCode_t;

/* ==================== PID Target Loop Enumeration ==================== */
typedef enum {
    PID_TARGET_NONE     = 0,
    PID_TARGET_BALANCE  = 1,
    PID_TARGET_VELOCITY = 2,
    PID_TARGET_STEERING = 3
} HC05_PID_Target_t;

/* ==================== HC-05 Structure Definitions ==================== */
typedef struct {
    float             Kp;
    float             Ki;
    float             Kd;
    HC05_PID_Target_t target;     /*!< Target loop: PID_TARGET_NONE, PID_TARGET_BALANCE, etc. */
    uint8_t           isNew;
} HC05_PID_t;

typedef struct {
    HC05_DirectionCode_t current;
    uint8_t              isNew;
} HC05_Direction_t;

typedef struct {
    USART_Handle_t   usart;
    CommandQueue_t   queue;
    char             txBuffer[HC05_TX_BUFFER_SIZE];
    char             rxBuffer[HC05_RX_BUFFER_SIZE];
    HC05_PID_t       pid;
    HC05_Direction_t direction;
} HC05_Device_t;

/**
 * @brief   Initialize the HC-05 module device structure and receive queue.
 * @param   dev         Pointer to HC05_Device_t instance.
 * @param   usartConfig Pointer to an ALREADY-INITIALIZED USART_Handle_t, or NULL.
 * @note    - If usartConfig is NULL: The driver automatically initializes the default 
 *            USART2 peripheral (9600 baud) and configures AT commands. GPIO pins (PA1, PA2, PA3) 
 *            must be configured beforehand (e.g. via Pin_Config_HC05()).
 *          - If usartConfig is provided: The user MUST configure GPIO pins and call USART_Init() 
 *            externally beforehand. The library will NOT re-initialize the USART peripheral.
 * @retval  None
 */
void HC05_Init(HC05_Device_t* dev, USART_Handle_t* usartConfig);

/**
 * @brief   Receive a frame from the HC-05 queue.
 * @details Pops a received message (if any) from the internal command queue.
 * @param   dev    Pointer to HC05_Device_t instance.
 * @param   buffer Pointer to the character array where the received string will be stored.
 * @note    User can pass NULL to buffer if no need to retrieve raw string.
 * @retval  uint8_t 1 (true) if a frame was successfully received, 0 (false) if queue is empty.
 */
uint8_t HC05_Receive_Frame(HC05_Device_t* dev, uint8_t* buffer);

/**
 * @brief   Send an acknowledgment (ACK) frame.
 * @details Sends "ACK:OK\r\n" or "ACK:FAIL\r\n" depending on the input parameter.
 * @param   dev  Pointer to HC05_Device_t instance.
 * @param   isOk 1 to send OK, 0 to send FAIL.
 * @retval  uint8_t The number of bytes successfully transmitted.
 */
uint8_t HC05_Send_ACK_Frame(HC05_Device_t* dev, uint8_t isOk);

/**
 * @brief   Send an IMU data frame.
 * @details Formats and sends Pitch, Roll, Accelerometer, and Gyroscope data.
 *          The frame format is: "IMU:pitch,roll,ax,ay,az,gx,gy,gz\r\n"
 * @param   dev   Pointer to HC05_Device_t instance.
 * @param   pitch Pitch angle (float).
 * @param   roll  Roll angle (float).
 * @param   ax    Accelerometer X-axis value (float).
 * @param   ay    Accelerometer Y-axis value (float).
 * @param   az    Accelerometer Z-axis value (float).
 * @param   gx    Gyroscope X-axis value (float).
 * @param   gy    Gyroscope Y-axis value (float).
 * @param   gz    Gyroscope Z-axis value (float).
 * @retval  uint8_t The number of bytes successfully transmitted.
 */
uint8_t HC05_Send_IMU_Frame(HC05_Device_t* dev, float targetAngle, float pitch, float ax, float ay, float az, float gx, float gy, float gz);

/**
 * @brief   Send a raw AT command to the HC-05 module.
 * @details Transmits the provided string via USART without additional formatting.
 *          The command string should usually end with "\r\n".
 * @param   dev Pointer to HC05_Device_t instance.
 * @param   cmd Pointer to the null-terminated AT command string.
 * @retval  None
 */
void HC05_Send_AT_Command(HC05_Device_t* dev, const char* cmd);

/**
 * @brief   Get the latest received PID tuning parameters.
 * @details Copies the stored Kp, Ki, and Kd values into the provided pointers
 *          and clears the new parameter flag.
 * @param   dev Pointer to HC05_Device_t instance.
 * @param   Kp  Pointer to store the Kp value (can be NULL).
 * @param   Ki  Pointer to store the Ki value (can be NULL).
 * @param   Kd  Pointer to store the Kd value (can be NULL).
 * @retval  None
 */
void HC05_Get_Tuning_Parameters(HC05_Device_t* dev, float* Kp, float* Ki, float* Kd);

/**
 * @brief   Get the target PID loop of the latest received parameters.
 * @param   dev Pointer to HC05_Device_t instance.
 * @retval  HC05_PID_Target_t The target loop (PID_TARGET_BALANCE, PID_TARGET_VELOCITY, PID_TARGET_STEERING, or PID_TARGET_NONE).
 */
HC05_PID_Target_t HC05_Get_PID_Target(HC05_Device_t* dev);

/**
 * @brief   Get the latest robot movement direction command.
 * @details Returns the last decoded direction (DIR_FORWARD, DIR_REVERSED, etc.)
 *          and clears the new direction flag.
 * @param   dev Pointer to HC05_Device_t instance.
 * @retval  HC05_DirectionCode_t The direction code (DIR_FORWARD, DIR_REVERSED, DIR_LEFT, DIR_RIGHT, DIR_NONE).
 */
HC05_DirectionCode_t HC05_Get_Direction(HC05_Device_t* dev);

/**
 * @brief   Check if new PID tuning parameters have been received.
 * @details Returns 1 if a PID parameter frame was parsed in the last received frame.
 * @param   dev Pointer to HC05_Device_t instance.
 * @retval  uint8_t 1 (true) if new parameters are available, 0 (false) otherwise.
 */
uint8_t HC05_Is_Parameter_New(HC05_Device_t* dev);

/**
 * @brief   Check if a new direction command has been received.
 * @details Returns 1 if a direction command frame was parsed in the last received frame.
 * @param   dev Pointer to HC05_Device_t instance.
 * @retval  uint8_t 1 (true) if a new direction command is available, 0 (false) otherwise.
 */
uint8_t HC05_Is_Direction_New(HC05_Device_t* dev);

/**
 * @brief   Interrupt service routine handler for HC-05 USART.
 * @details Handles receiving data and pushes complete frames into the queue on IDLE line detection.
 *          Call this function inside the USARTx_IRQHandler of the USART connected to HC-05.
 * @param   dev Pointer to HC05_Device_t instance.
 * @retval  None
 */
void HC05_IRQ_Handler(HC05_Device_t* dev);

void HC05_DMA_IRQHandler(HC05_Device_t* dev);

#endif /* STM32F4XX_HC05_H_ */