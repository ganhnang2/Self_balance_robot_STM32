/**
 * @file    004_test_encoder.c
 * @brief   Kiểm tra chức năng đọc Encoder 2 bánh xe JGA25-370.
 *
 * === Phần cứng ===
 *  Encoder Bánh Trái : PA8 (TIM1_CH1, AF1), PA9 (TIM1_CH2, AF1)
 *  Encoder Bánh Phải : PB7 (TIM4_CH1, AF2), PB6 (TIM4_CH2, AF2)
 *  UART Debug (printf): PA11 (TX), PA12 (RX) -> USART6
 */

#include <stdio.h>
#include "stm32f4xx_motor.h"
#include "stm32f4xx.h"
#include "pin_config.h"

USART_Handle_t usartPrintf;

/* =========================================================================
 * Biến toàn cục chia sẻ giữa ISR và main()
 * ========================================================================= */

volatile uint8_t s_DataReady  = 0;  	// Flag: ISR báo hiệu có dữ liệu mới 

// Hai biến lưu trữ motor bên trái
volatile int32_t s_LeftCounter  = 0;	// Giá trị hiện tại của bộ đếm Counter
volatile int32_t s_LeftCount = 0;		// Số xung đếm được sau 10ms

// Hai biến lưu trữ motor bên phải
volatile uint32_t s_RightCounter = 0;	// Giá trị hiện tại của bộ đếm Counter
volatile uint32_t s_RightCount = 0;		// Số xung đếm được sau 10ms

// Retarget printf() qua USART6
int _write(int file, char *ptr, int len)
{
    if (file == 1 || file == 2)
    {
        USART_Write(&usartPrintf, (uint8_t *)ptr, (uint32_t)len);
        return len;
    }
    return -1;
}

void GPIO_Config(void) {
	/*********************** USART6 (PA11 - TX, PA12 - RX, AF08) *********************/
	// Bật clock cho GPIOA
    RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN_Pos);

	// Pin mode: Alternate Function
    GPIOA->MODER &= ~((0x03 << GPIO_MODER_MODER11_Pos) | (0x03 << GPIO_MODER_MODER12_Pos));
    GPIOA->MODER |=  ((0x02 << GPIO_MODER_MODER11_Pos) | (0x02 << GPIO_MODER_MODER12_Pos));

	// Output type: Push-pull
    GPIOA->OTYPER &= ~((1 << GPIO_OTYPER_OT11_Pos) | (1 << GPIO_OTYPER_OT12_Pos));

	// Speed: Fast
    GPIOA->OSPEEDR &= ~((0x03 << GPIO_OSPEEDR_OSPEED11_Pos) | (0x03 << GPIO_OSPEEDR_OSPEED12_Pos));
    GPIOA->OSPEEDR |=  ((0x02 << GPIO_OSPEEDR_OSPEED11_Pos) | (0x02 << GPIO_OSPEEDR_OSPEED12_Pos));

	// Pull-up
    GPIOA->PUPDR &= ~((0x03 << GPIO_PUPDR_PUPD11_Pos) | (0x03 << GPIO_PUPDR_PUPD12_Pos));
    GPIOA->PUPDR |=  ((0x01 << GPIO_PUPDR_PUPD11_Pos) | (0x01 << GPIO_PUPDR_PUPD12_Pos));

	// Alternate Function 8 (USART6) — PA11, PA12 ở AFRH (AFR[1])
    GPIOA->AFR[1] &= ~((0x0F << GPIO_AFRH_AFSEL11_Pos) | (0x0F << GPIO_AFRH_AFSEL12_Pos));
    GPIOA->AFR[1] |=  ((0x08 << GPIO_AFRH_AFSEL11_Pos) | (0x08 << GPIO_AFRH_AFSEL12_Pos));
}

void USART6_Init(void)
{
	/*********************** USART6 (PA11 - TX, PA12 - RX, AF08) *********************/
	// Cấu hình USART6 để dùng printf() lên Terminal máy tính (115200 baud)
    RCC->APB2ENR |= (1 << RCC_APB2ENR_USART6EN_Pos);
    usartPrintf.pUSARTx = USART6;
    usartPrintf.config.baudrate = 115200;
    usartPrintf.config.oversampling8 = 0;
    USART_Init(&usartPrintf);
}

/* =========================================================================
 * TIM5 Control Loop Init — 100Hz interrupt
 * TIM5 là 32-bit timer trên APB1 (84MHz trên F401)
 * fTIM5 = 25MHz / (PSC+1) = 1MHz
 * fISR  = 1MHz / (ARR+1)  = 100Hz → ARR = 9999
 * ========================================================================= */
void TIM5_ControlLoop_Init(void)
{
    /* Bật clock TIM5 (APB1) */
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    TIM5->PSC  = 24;     	/* 25 - 1 = 24 → fTIM5 = 1MHz  */
    TIM5->ARR  = 9999;     	/* 9999         → fISR  = 100Hz */
    TIM5->CNT  = 0;

    /* Auto-reload preload enable */
	// Nếu ARPE = 0: CNT có thể đang ở giữa chu kỳ, ARR thay đổi ngay
	//               → Chu kỳ PWM hiện tại bị cắt ngắn/kéo dài bất thường
	// Nếu ARPE = 1: ARR mới chỉ có hiệu lực ở chu kỳ tiếp theo → Sóng PWM sạch
    TIM5->CR1 |= TIM_CR1_ARPE;

    /* Bật ngắt Update */
    TIM5->DIER |= TIM_DIER_UIE;

    /* Bật NVIC cho TIM5 */
    NVIC_SetPriority(TIM5_IRQn, 1);
    NVIC_EnableIRQ(TIM5_IRQn);

    /* Khởi chạy Timer */
    TIM5->CR1 |= TIM_CR1_CEN;
}


/* =========================================================================
 * TIM5 IRQ Handler — Ngắt 100Hz, đọc Encoder tại đây
 *
 * Lý do đặt Motor_Encoder_Update() TRONG ISR:
 *   Để đảm bảo dt (khoảng thời gian giữa 2 lần đọc) là CHÍNH XÁC TUYỆT ĐỐI.
 *   Nếu đọc trong main() thì dt bị ảnh hưởng bởi thời gian printf() -> không chính xác.
 * ========================================================================= */
void TIM5_IRQHandler(void)
{
    if (TIM5->SR & TIM_SR_UIF)
    {
        TIM5->SR &= ~TIM_SR_UIF;    /* Xóa cờ ngắt */

        /* Cập nhật trạng thái Encoder (wrap-around subtraction tự động xử lý tràn số) */
        Motor_Encoder_Update();

        /* Copy kết quả sang biến buffer để main() đọc an toàn */
        s_LeftCount  = Motor_Get_Encoder_Left();
        s_RightCount = Motor_Get_Encoder_Right();
		
		/* Nhận counter mà Encoder đếm được */
		s_LeftCounter = Motor_Get_Encoder_Counter_Left();
		s_RightCounter = Motor_Get_Encoder_Counter_Right();
		
        s_DataReady = 1;    /* Báo main() có dữ liệu mới sẵn sàng */
    }
}

/* =========================================================================
 * Main
 * ========================================================================= */

int main(void)
{
    /* 1. Khởi tạo GPIO cho Encoder và Motor */
    Pin_Config_Encoder();
    Pin_Config_Motor();

    /* 2. Khởi tạo USART6 (PA11 TX / PA12 RX, AF08) để debug printf */
    GPIO_Config();
    USART6_Init();

    /* 3. Khởi tạo Encoder (TIM1 + TIM4) */
    Motor_Encoder_Init();

    /* 4. Khởi chạy TIM5 ngắt 100Hz (vòng lặp điều khiển) */
    TIM5_ControlLoop_Init();
	
	TB6612_Init();

    printf("=== ENCODER TEST START ===\r\n");
    printf("Quay banh xe va quan sat gia tri ben duoi:\r\n\r\n");

    /* 4. Vòng lặp chính: chỉ in dữ liệu, không xử lý logic */
    while (1)
    {
        if (s_DataReady)
        {
			// Thực hiện in dữ liệu lên Serial để thực hiện debug chiều quay và xung đọc được của Encoder
			printf("-----------------------------------------------\r\n");
			printf("Left: Delta count [pulse/100ms] = %ld, Counter = %ld\r\n", (int32_t)s_LeftCount, (int32_t)s_LeftCounter);
			printf("Right: Delta count [pulse/100ms] = %ld, Counter = %ld\r\n", (int32_t)s_RightCount, (int32_t)s_RightCounter);
			
			// Reset cờ trạng thái
			s_DataReady = 0;
        }
    }
}