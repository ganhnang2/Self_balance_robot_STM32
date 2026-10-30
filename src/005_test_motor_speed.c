/**
 * @file    005_test_motor_speed.c
 * @brief   Chương trình kiểm tra và điều khiển tốc độ 2 động cơ, hiển thị tốc độ và Encoder qua Bluetooth HC-05.
 *
 * === Mục đích ===
 * - CHỈ SỬ DỤNG DUY NHẤT BLUETOOTH HC-05 (USART2):
 *   + Người dùng nhập trực tiếp giá trị PWM từ điện thoại hoặc máy tính qua HC-05.
 *   + Trạng thái tốc độ (RPM, xung Encoder, PWM, cm/s) của cả 2 bánh được hiển thị trực tiếp lên máy tính.
 * - Không cần bất kỳ kết nối UART có dây nào khác (printf tự động xuất qua HC-05).
 *
 * === Hướng dẫn nhập lệnh qua Bluetooth Terminal ===
 * 1. Nhập 1 số để điều khiển cả 2 bánh:
 *    - Ví dụ: "500"   -> Cả 2 bánh tiến với PWM = 500
 *    - Ví dụ: "-400"  -> Cả 2 bánh lùi với PWM = 400
 *    - Ví dụ: "0"     -> Dừng cả 2 bánh
 * 2. Nhập 2 số để điều khiển độc lập từng bánh (cách nhau bởi dấu phẩy hoặc khoảng trắng):
 *    - Ví dụ: "300 600"  (hoặc "300,600") -> Bánh trái 300, bánh phải 600
 *    - Ví dụ: "400 -400" (hoặc "400,-400")-> Quay tại chỗ (trái tiến, phải lùi)
 * 3. Phím tắt nhanh:
 *    - 's' hoặc '0'   -> Dừng khẩn cấp động cơ
 *    - '+'            -> Tăng tốc độ cả 2 bánh thêm 50 PWM
 *    - '-'            -> Giảm tốc độ cả 2 bánh đi 50 PWM
 *    - 'r' hoặc 'R'   -> Reset bộ đếm Encoder về 0
 *    - 'p' hoặc 'P'   -> Tạm dừng / Tiếp tục in dữ liệu tốc độ
 *    - 'h' hoặc '?'   -> In lại bảng hướng dẫn
 *
 * === Cấu hình phần cứng ===
 * - Động cơ & Cầu H TB6612FNG:
 *   + STBY: PA12 (Standby = 1)
 *   + AIN1: PB13, AIN2: PB12 (Chiều quay Bánh Phải)
 *   + BIN1: PB14, BIN2: PB15 (Chiều quay Bánh Trái)
 *   + PWMA: PB4 (TIM3_CH1), PWMB: PB5 (TIM3_CH2) - PWM 12.5kHz
 * - Encoder 2 bánh:
 *   + Bánh Trái: PA8 (TIM1_CH1), PA9 (TIM1_CH2)
 *   + Bánh Phải: PB6 (TIM4_CH1), PB7 (TIM4_CH2)
 * - Module Bluetooth HC-05 (USART2):
 *   + PA1: KEY, PA2: TX, PA3: RX (115200 baud, ngắt USART2)
 * - Timer đo chu kỳ:
 *   + TIM5: Chu kỳ 10ms (100Hz) để đọc Encoder chính xác
 * - LED Báo hiệu:
 *   + PC13: Built-in LED nhấp nháy khi đang hoạt động
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_lib.h"
#include "pin_config.h"

/* =========================================================================
 * Định nghĩa hằng số
 * ========================================================================= */
#define MOTOR_MAX_PWM           1250L       // PWM tối đa (100% công suất)
#define MOTOR_MIN_PWM          -1250L       // PWM tối đa chiều ngược lại

#define CONTROL_FREQ_HZ         100.0f      // Tần số ngắt đo Encoder 100Hz
#define CONTROL_PERIOD_US       10000UL     // Chu kỳ 100Hz = 10ms = 10000 us
#define CONTROL_DT_S            0.010f      // dt = 10ms = 0.010 giây
#define DISPLAY_DIVIDER         10          // In dữ liệu mỗi 10 chu kỳ = 100ms (10Hz)

/* Thông số cơ khí động cơ JGA25-370 và bánh xe */
#define ENCODER_PULSES_PER_REV  913.2f      // Số xung / 1 vòng quay bánh xe
#define WHEEL_RADIUS_M          0.0325f     // Bán kính bánh xe: R = 32.5mm = 0.0325m
#define PI                      3.14159265f

/* =========================================================================
 * Biến toàn cục
 * ========================================================================= */
HC05_Device_t hc05Dev;                      // Đối tượng giao tiếp module HC-05

volatile uint8_t s_DataReady = 0;           // Cờ báo ngắt TIM5 đo xong mẫu mới

// Xung Encoder đo được trong mỗi chu kỳ 10ms
volatile int32_t s_LeftDelta  = 0;
volatile int32_t s_RightDelta = 0;

// Giá trị thanh ghi CNT tức thời của Timer Encoder
volatile uint32_t s_LeftCounter  = 0;
volatile uint32_t s_RightCounter = 0;

// Tổng số xung Encoder tích lũy
volatile int32_t s_TotalLeftPulses  = 0;
volatile int32_t s_TotalRightPulses = 0;

// Giá trị PWM hiện tại đang cấp cho 2 động cơ [-1250, 1250]
volatile int32_t s_CurrentLeftPWM  = 0;
volatile int32_t s_CurrentRightPWM = 0;

// Cờ bật/tắt hiển thị lên màn hình
volatile uint8_t s_PrintEnabled = 1;

/* =========================================================================
 * Retarget printf() sang trực tiếp Bluetooth HC-05 (USART2)
 * Giúp mọi lệnh printf() tự động truyền thẳng lên điện thoại / máy tính qua Bluetooth.
 * ========================================================================= */
int _write(int file, char *ptr, int len)
{
    if (file == 1 || file == 2)
    {
        USART_Write(&hc05Dev.usart, (uint8_t *)ptr, (uint32_t)len);
        return len;
    }
    return -1;
}

/* =========================================================================
 * Khởi tạo TIM5 ngắt chu kỳ 10ms (100Hz) để đo xung Encoder
 * ========================================================================= */
void TIM5_ControlLoop_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    uint32_t timerClock = (SystemCoreClock > 1000000UL) ? SystemCoreClock : 25000000UL;
    TIM5->PSC = (timerClock / 1000000UL) - 1; // 1 us / tick
    TIM5->ARR = CONTROL_PERIOD_US - 1;        // 10ms = 10000 us
    TIM5->CNT = 0;

    TIM5->CR1 |= TIM_CR1_ARPE;
    TIM5->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM5_IRQn, 2);
    NVIC_EnableIRQ(TIM5_IRQn);

    TIM5->CR1 |= TIM_CR1_CEN;
}

/* =========================================================================
 * Trình phục vụ ngắt TIM5 (100Hz / 10ms)
 * ========================================================================= */
void TIM5_IRQHandler(void)
{
    if (TIM5->SR & TIM_SR_UIF)
    {
        TIM5->SR &= ~TIM_SR_UIF; // Xóa cờ ngắt

        // 1. Cập nhật trạng thái Encoder của 2 bánh xe
        Motor_Encoder_Update();

        // 2. Lấy số xung delta trong chu kỳ 10ms vừa qua
        s_LeftDelta   = Motor_Get_Encoder_Left();
        s_RightDelta  = Motor_Get_Encoder_Right();

        // 3. Lấy giá trị thanh ghi counter CNT
        s_LeftCounter  = Motor_Get_Encoder_Counter_Left();
        s_RightCounter = Motor_Get_Encoder_Counter_Right();

        // 4. Cộng dồn vào tổng số xung tích lũy
        s_TotalLeftPulses  += s_LeftDelta;
        s_TotalRightPulses += s_RightDelta;

        s_DataReady = 1; // Báo hiệu cho main() có dữ liệu mới
    }
}

/* =========================================================================
 * Trình phục vụ ngắt USART2 (Bluetooth HC-05)
 * ========================================================================= */
void USART2_IRQHandler(void)
{
    HC05_IRQ_Handler(&hc05Dev);
}

/* =========================================================================
 * Giới hạn PWM trong khoảng an toàn [-1250, 1250]
 * ========================================================================= */
static inline int32_t Clamp_PWM(int32_t pwm)
{
    if (pwm > MOTOR_MAX_PWM) return MOTOR_MAX_PWM;
    if (pwm < MOTOR_MIN_PWM) return MOTOR_MIN_PWM;
    return pwm;
}

/* In hướng dẫn sử dụng */
void Print_Help_Menu(void)
{
    printf("\r\n");
    printf("======================================================================\r\n");
    printf("       HE THONG DIEU KHIEN PWM & HIEN THI ENCODER 2 BANH XE           \r\n");
    printf("                   (GIAO TIEP QUA BLUETOOTH HC-05)                    \r\n");
    printf("======================================================================\r\n");
    printf("Huong dan nhap lenh PWM tu ban phim:\r\n");
    printf("  1. Nhap 1 so        : vd '500'      -> Chay ca 2 banh PWM = 500\r\n");
    printf("  2. Nhap so am       : vd '-400'     -> Chay lui ca 2 banh PWM = -400\r\n");
    printf("  3. Nhap 2 so        : vd '300 600'  -> Banh Trai 300, Phai 600\r\n");
    printf("  4. Dung dong co     : nhap '0' hoac 's'\r\n");
    printf("  5. Phim tang/giam   : '+' (tang 50), '-' (giam 50)\r\n");
    printf("  6. Reset Encoder    : nhap 'r' (ve 0 xung)\r\n");
    printf("  7. Bat/Tat hien thi : nhap 'p'\r\n");
    printf("======================================================================\r\n\r\n");
}

/* =========================================================================
 * Hàm xử lý chuỗi nhập điều khiển PWM từ người dùng qua HC-05
 * ========================================================================= */
void Process_User_Command(char* rawStr)
{
    if (rawStr == NULL || rawStr[0] == '\0')
    {
        return;
    }

    // Xóa ký tự xuống dòng (\r, \n) và khoảng trắng thừa
    size_t len = strlen(rawStr);
    while (len > 0 && (rawStr[len - 1] == '\r' || rawStr[len - 1] == '\n' || rawStr[len - 1] == ' '))
    {
        rawStr[--len] = '\0';
    }

    char* str = rawStr;
    while (*str == ' ') str++; // Bỏ qua khoảng trắng đầu
    if (*str == '\0') return;

    // Bỏ qua tiền tố "SPD:" hoặc "PWM:" nếu người dùng có gõ
    if (strncmp(str, "SPD:", 4) == 0 || strncmp(str, "PWM:", 4) == 0)
    {
        str += 4;
        while (*str == ' ') str++;
    }

    // --- Trường hợp 1: Phím tắt một ký tự ---
    if ((str[0] == 's' || str[0] == 'S' || str[0] == '0') && str[1] == '\0')
    {
        s_CurrentLeftPWM  = 0;
        s_CurrentRightPWM = 0;
        TB6612_Control_Motors(0, 0);
        printf("\r\n>> [DUNG DONG CO] PWM Trai = 0, Phai = 0\r\n\r\n");
        return;
    }
    else if (str[0] == '+' && str[1] == '\0')
    {
        s_CurrentLeftPWM  = Clamp_PWM(s_CurrentLeftPWM + 50);
        s_CurrentRightPWM = Clamp_PWM(s_CurrentRightPWM + 50);
        TB6612_Control_Motors(s_CurrentLeftPWM, s_CurrentRightPWM);
        printf("\r\n>> [TANG TOC +50] PWM Trai = %ld, Phai = %ld\r\n\r\n",
               (long)s_CurrentLeftPWM, (long)s_CurrentRightPWM);
        return;
    }
    else if (str[0] == '-' && str[1] == '\0')
    {
        s_CurrentLeftPWM  = Clamp_PWM(s_CurrentLeftPWM - 50);
        s_CurrentRightPWM = Clamp_PWM(s_CurrentRightPWM - 50);
        TB6612_Control_Motors(s_CurrentLeftPWM, s_CurrentRightPWM);
        printf("\r\n>> [GIAM TOC -50] PWM Trai = %ld, Phai = %ld\r\n\r\n",
               (long)s_CurrentLeftPWM, (long)s_CurrentRightPWM);
        return;
    }
    else if ((str[0] == 'r' || str[0] == 'R') && str[1] == '\0')
    {
        __disable_irq();
        Motor_Encoder_Reset();
        s_TotalLeftPulses  = 0;
        s_TotalRightPulses = 0;
        __enable_irq();
        printf("\r\n>> [RESET] Da reset bo dem Encoder ca 2 banh ve 0!\r\n\r\n");
        return;
    }
    else if ((str[0] == 'p' || str[0] == 'P') && str[1] == '\0')
    {
        s_PrintEnabled = !s_PrintEnabled;
        printf("\r\n>> [PRINT] %s hien thi toc do.\r\n\r\n", s_PrintEnabled ? "BAT" : "TAT");
        return;
    }
    else if (str[0] == 'h' || str[0] == 'H' || str[0] == '?')
    {
        Print_Help_Menu();
        return;
    }

    // --- Trường hợp 2: Nhập số PWM ---
    // Kiểm tra xem người dùng nhập 2 số riêng biệt ("500 600" hoặc "500,600") hay 1 số ("500")
    long val1 = 0, val2 = 0;
    char delimiter = 0;

    // Thử đọc 2 số có dấu phân cách (dấu phẩy, khoảng trắng, chấm phẩy)
    if (sscanf(str, "%ld%c%ld", &val1, &delimiter, &val2) >= 3 &&
        (delimiter == ',' || delimiter == ' ' || delimiter == ';'))
    {
        s_CurrentLeftPWM  = Clamp_PWM(val1);
        s_CurrentRightPWM = Clamp_PWM(val2);
        TB6612_Control_Motors(s_CurrentLeftPWM, s_CurrentRightPWM);
        printf("\r\n>> [OK] Cai dat PWM 2 banh: Trai = %ld | Phai = %ld\r\n\r\n",
               (long)s_CurrentLeftPWM, (long)s_CurrentRightPWM);
    }
    // Thử đọc 1 số duy nhất cho cả 2 bánh
    else if (sscanf(str, "%ld", &val1) == 1)
    {
        s_CurrentLeftPWM  = Clamp_PWM(val1);
        s_CurrentRightPWM = Clamp_PWM(val1);
        TB6612_Control_Motors(s_CurrentLeftPWM, s_CurrentRightPWM);
        printf("\r\n>> [OK] Cai dat PWM ca 2 banh: %ld\r\n\r\n", (long)val1);
    }
    else
    {
        printf("\r\n>> [LOI] Khong hieu: \"%s\". Vui long nhap so PWM (vd: 500 hoac 400 600)\r\n\r\n", str);
    }
}

/* =========================================================================
 * MAIN FUNCTION
 * ========================================================================= */
int main(void)
{
    char rxCmd[64];
    uint8_t displayCounter = 0;

    // Các biến tích lũy xung trong chu kỳ hiển thị (100ms)
    int32_t accumLeftPulses  = 0;
    int32_t accumRightPulses = 0;

    /* 1. Khởi tạo toàn bộ chân GPIO: LED, Encoder, Motor TB6612, Bluetooth HC-05 */
    Pin_Config_All();

    /* 2. Khởi tạo Driver TB6612 và Encoder 2 bánh (TIM1 + TIM4) */
    Motor_Encoder_Init();
    TB6612_Init();
    TB6612_Control_Motors(0, 0); // Dừng động cơ lúc khởi động

    // Bật chân STBY của TB6612 lên mức HIGH để kích hoạt cầu H
    GPIO_Write_Pin(PIN_MOTOR_STBY_PORT, PIN_MOTOR_STBY_PIN, GPIO_PIN_SET);

    /* 3. Khởi tạo Bluetooth HC-05 (USART2 - 115200 baud) */
    NVIC_SetPriority(USART2_IRQn, 1);
    NVIC_EnableIRQ(USART2_IRQn);
    HC05_Init(&hc05Dev, NULL);

    /* 4. Khởi chạy Timer TIM5 ngắt chu kỳ 10ms (100Hz) để đo tốc độ Encoder */
    TIM5_ControlLoop_Init();

    /* 5. Gửi bảng hướng dẫn sử dụng lên máy tính */
    Print_Help_Menu();

    // Chu vi bánh xe (cm): C = 2 * PI * R
    const float wheelCircumferenceCm = 2.0f * PI * WHEEL_RADIUS_M * 100.0f; // ~20.42 cm

    /* 6. Vòng lặp chính */
    while (1)
    {
        // -------------------------------------------------------------
        // [A] Nhận dữ liệu điều khiển PWM từ người dùng qua Bluetooth
        // -------------------------------------------------------------
        if (HC05_Receive_Frame(&hc05Dev, (uint8_t*)rxCmd) != 0)
        {
            Process_User_Command(rxCmd);
        }

        // -------------------------------------------------------------
        // [B] Cập nhật và tính toán thông số 2 bánh mỗi 10ms (100Hz)
        // -------------------------------------------------------------
        if (s_DataReady)
        {
            // Đọc an toàn dữ liệu từ ngắt
            __disable_irq();
            int32_t leftDelta   = s_LeftDelta;
            int32_t rightDelta  = s_RightDelta;
            int32_t totalLeft   = s_TotalLeftPulses;
            int32_t totalRight  = s_TotalRightPulses;
            s_DataReady = 0;
            __enable_irq();

            // Cộng dồn số xung trong chu kỳ hiển thị (10 mẫu = 100ms)
            accumLeftPulses  += leftDelta;
            accumRightPulses += rightDelta;
            displayCounter++;

            // Mỗi 10 chu kỳ 10ms = 100ms (10Hz) xuất dữ liệu 1 lần
            if (displayCounter >= DISPLAY_DIVIDER)
            {
                displayCounter = 0;

                if (s_PrintEnabled)
                {
                    // Khoảng thời gian đo thực tế của mẫu hiển thị (100ms = 0.1s)
                    const float dtDisplay = (float)DISPLAY_DIVIDER * CONTROL_DT_S;

                    // 1. Tốc độ vòng quay bánh xe (RPM = Revolutions Per Minute)
                    // RPM = (xung / xung_moi_vong) * (60s / dt)
                    float leftRPM  = ((float)accumLeftPulses  / ENCODER_PULSES_PER_REV) * (60.0f / dtDisplay);
                    float rightRPM = ((float)accumRightPulses / ENCODER_PULSES_PER_REV) * (60.0f / dtDisplay);

                    // 2. Vận tốc dài thực tế của từng bánh (cm/s)
                    // v = (RPM / 60) * Chu_vi_banh_xe
                    float leftSpeedCmS  = (leftRPM  / 60.0f) * wheelCircumferenceCm;
                    float rightSpeedCmS = (rightRPM / 60.0f) * wheelCircumferenceCm;

                    // 3. Hiển thị thông số ra máy tính qua Bluetooth
                    printf("[SPEED] L: PWM=%+5ld | Pls=%+4ld | Tot=%+7ld | %6.1f RPM | %5.1f cm/s || R: PWM=%+5ld | Pls=%+4ld | Tot=%+7ld | %6.1f RPM | %5.1f cm/s\r\n",
                           (long)s_CurrentLeftPWM,
                           (long)accumLeftPulses,
                           (long)totalLeft,
                           leftRPM,
                           leftSpeedCmS,
                           (long)s_CurrentRightPWM,
                           (long)accumRightPulses,
                           (long)totalRight,
                           rightRPM,
                           rightSpeedCmS);

                    // Đảo trạng thái LED PC13 để báo hiệu vi điều khiển đang hoạt động tốt
                    LED_BUILTIN_TOGGLE();
                }

                // Reset bộ cộng dồn sau khi in
                accumLeftPulses  = 0;
                accumRightPulses = 0;
            }
        }
    }
}