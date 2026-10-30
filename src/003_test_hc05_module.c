/*
 * Test HC-05 Bluetooth Module with STM32F401
 * - USART2 (PA2-TX, PA3-RX): Giao tiếp với HC-05
 * - USART1 (PB6-TX, PB7-RX): Dùng in printf() lên máy tính qua CH340
 * - PA1: Chân KEY của HC-05
 */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hc05.h"
#include "pin_config.h"

#define MAX_RECEIVED_BYTES      64

USART_Handle_t usartPrintf;         // USART1 để debug printf
char rcvBuffer[MAX_RECEIVED_BYTES]; // Mảng nhận dữ liệu từ HC-05

// Retarget printf() qua USART1
int _write(int file, char *ptr, int len)
{
    if (file == 1 || file == 2)
    {
        USART_Write(&usartPrintf, (uint8_t *)ptr, (uint32_t)len);
        return len;
    }
    return -1;
}

void USART1_Init()
{
	/*********************** USART1 (PB6 - TX, PB7 - RX) *********************/
	// Bật clock cho GPIOB (QUAN TRỌNG)
    RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOBEN_Pos);
	
    GPIOB->MODER &= ~((0x03 << 12) | (0x03 << 14));
    GPIOB->MODER |=  ((0x02 << 12) | (0x02 << 14)); // Alternate Function
    
    GPIOB->AFR[0] &= ~((0x0F << 24) | (0x0F << 28));
    GPIOB->AFR[0] |=  ((0x07 << 24) | (0x07 << 28)); // AF7 cho PB6, PB7

    GPIOB->OSPEEDR |= ((0x03 << 12) | (0x03 << 14));
    GPIOB->PUPDR   |= ((0x01 << 12) | (0x01 << 14));
	
	// Cấu hình USART1 để dùng printf() lên Terminal máy tính (115200 baud)
    RCC->APB2ENR |= (1 << RCC_APB2ENR_USART1EN_Pos);
    usartPrintf.pUSARTx = USART1;
    usartPrintf.config.baudrate = 115200;
    usartPrintf.config.oversampling8 = 0;
    USART_Init(&usartPrintf);
	
	NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void)
{
    USART_IRQHandling(&usartPrintf);
}

// In menu lựa chọn chức năng ra cửa sổ Serial USART1
void Print_Menu(void)
{
    printf("\r\n=========================================\r\n");
    printf("        HC-05 BLUETOOTH TEST MENU        \r\n");
    printf("=========================================\r\n");
    printf("  1. Truyen du lieu (GUI 1 goi IMU)\r\n");
    printf("  2. Nhan du lieu  (NHAN 1 goi tin)\r\n");
    printf("-----------------------------------------\r\n");
    printf(">> Nhap lua chon (1 hoac 2): ");
}

int main(void)
{
    char cmd[10];
    float ax = -0.67f, ay = 0.32f, az = 0.76f;
    float gx = 6.78f, gy = 4.532f, gz = -2.23f;
	float pitch = 86.78, target = 0.67;
    
    HC05_Device_t hc05Dev;

	Pin_Config_HC05();
	USART1_Init();
    
	HC05_Init(&hc05Dev, NULL);
    printf("Bat dau chuong trinh test HC-05!\r\n");

	while(1)
	{
        memset(cmd, 0, sizeof(cmd));
        Print_Menu();
        // Blocking: Hàm này chờ đến khi nhận được ký tự và đường truyền IDLE
        // mới thoát ra 
        USART_Read_Until_Idle(&usartPrintf, (uint8_t*)cmd, sizeof(cmd));
		
		// --- CHỨC NĂNG 1: GỬI 1 GÓI IMU QUA BLUETOOTH ---
		if (cmd[0] == '1')
		{
			printf("\r\n[TX] Dang gui goi tin IMU...\r\n");
			// Giả lập IMU thay đổi nhẹ mỗi lần gửi
			ax += 0.01f; if (ax >  1.0f) ax = -1.0f;
			gx += 0.10f; if (gx > 20.0f) gx = -20.0f;
			// Đóng gói và gửi ĐÚNG 1 gói tin IMU qua HC-05
			uint8_t sentBytes = HC05_Send_IMU_Frame(&hc05Dev, target, pitch, ax, ay, az, gx, gy, gz);
			
			// Đóng gói và gửi ĐÚNG 1 gói tin IMU qua HC-05
			if(sentBytes)
			{
				printf("[TX DONE] Da gui thanh cong %d bytes!\r\n", sentBytes);
			}
			else
			{
				printf("[TX ERROR] Truyen du lieu khong thanh cong!\r\n");
			}
		}
		// --- CHỨC NĂNG 2: NHẬN ĐÚNG 1 GÓI TIN TỪ BLUETOOTH ---
		else if (cmd[0] == '2')
		{
			// Có dữ liệu mới nhận được
			if (HC05_Receive_Frame(&hc05Dev, (uint8_t*)rcvBuffer))
			{
				printf("[RX DONE] Nhan duoc: %s\r\n", rcvBuffer);
			}
			else
			{
				printf("[RX ERROR] Khong co du lieu!\r\n");
			}
		}
		// Phím không hợp lệ
		else
		{
			printf("\r\n[LOI] Phim '%c' khong hop le! Vui long nhap 1 hoac 2.\r\n", cmd[0]);
		}
    }
}
