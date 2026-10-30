#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pin_config.h"
#include "stm32f4xx_lib.h"

#define PI						3.14159265f

// --- Biến toàn cục cho ngắt Timer ---
MPU6050_t imu;
Kalman_t kalmanPitch;
Kalman_t kalmanRoll;
HC05_Device_t hc05Dev;

MPU6050_Status_t mpu_status = MPU6050_ERR_NOT_FOUND;

volatile float print_ax, print_ay, print_az;
volatile float print_gx, print_gy, print_gz;
volatile float print_pitch, print_roll;
volatile uint8_t DataReady = 0;

void TIM2_Init(void) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;    // Cấp xung nhịp cho TIM2
	
	// Cấu hình tần số đếm Timer về 1MHz (1 us/tick) động theo SystemCoreClock
	TIM2->PSC = (SystemCoreClock / 1000000UL) - 1;
	TIM2->ARR = 10000 - 1;                 // 10000 us = 10ms (100Hz)
	
	TIM2->SR = 0;                          // Xóa các cờ ngắt cũ
	TIM2->DIER |= TIM_DIER_UIE;            // Bật ngắt Update
	
	NVIC_EnableIRQ(TIM2_IRQn);             // Bật ngắt NVIC cho TIM2
	TIM2->CR1 |= TIM_CR1_CEN;              // Chạy Timer
}

/****************************************************************************/
/*							INTERRUPT HANDLING								*/
/****************************************************************************/
// Trình phục vụ ngắt của USART2 để nhận dữ liệu về tham số Kp, Ki, Kd từ máy tính
void USART2_IRQHandler(void)
{
	HC05_IRQ_Handler(&hc05Dev);
}

// Trình phục vụ ngắt của TIM2 (Hardware nhảy vào đây mỗi 10ms - 100Hz)
void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_UIF) { // Kiểm tra cờ ngắt Update
		TIM2->SR &= ~TIM_SR_UIF; // Xóa cờ ngắt
		
		// Đọc dữ liệu (Tốn ~1ms)
		MPU6050_ReadScaledData(&imu);
		
		float ax = imu.ScaledData.Accel_X, ay = imu.ScaledData.Accel_Y, az = imu.ScaledData.Accel_Z;
		float gx = imu.ScaledData.Gyro_X, gy = imu.ScaledData.Gyro_Y, gz = imu.ScaledData.Gyro_Z;
		
		// Tính góc thô từ gia tốc kế (dùng sqrtf cho FPU đơn chính xác 32-bit của Cortex-M4)
		float pitch_accel = atan2f(-ax, sqrtf(az*az + ay*ay)) * (180.0f / PI);
		float roll_accel  = atan2f(ay, sqrtf(ax*ax + az*az)) * (180.0f / PI);
		
		// Cập nhật Kalman Filter (dt chính xác = 0.010s ứng với tần số 100Hz)
		pitch_accel = Kalman_GetAngle(&kalmanPitch, pitch_accel, gy, 0.010f);
		roll_accel  = Kalman_GetAngle(&kalmanRoll, roll_accel, gx, 0.010f);
		
		// Đẩy ra biến toàn cục để main() có thể in
		print_ax = ax; print_ay = ay; print_az = az;
		print_gx = gx; print_gy = gy; print_gz = gz;
		print_pitch = pitch_accel;
		print_roll = roll_accel;
		
		DataReady = 1; // Báo hiệu có dữ liệu mới
		
		// ==> TƯƠNG LAI: Hàm cập nhật PID của Robot cân bằng sẽ được đặt ngay tại đây!
	}
}

int main(void)
{
	char oled_buf[30];
	
	// Khởi tạo chân GPIO cho các linh kiện
	Pin_Config_MPU6050();
	Pin_Config_OLED();
	Pin_Config_HC05();
	
	// Khởi tạo HC05 (Default: USART2 peripheral)
	HC05_Init(&hc05Dev, NULL);
	
	// Khởi tạo Kalman Filter
	Kalman_Init(&kalmanPitch);
	Kalman_Init(&kalmanRoll);
	
	// Khởi tạo Màn hình OLED (I2C2 peripheral)
	OLED_Init(NULL);
	OLED_Clear_Buffer();
	
	mpu_status = MPU6050_Init(&imu, NULL);
	
	// Khởi tạo MPU6050 (I2C1 peripheral)
	if (mpu_status == MPU6050_OK)
	{
		OLED_Print_String("MPU6050 INIT OK", 0, 0, OLED_COLOR_WHITE);
	}
	else 
	{
		OLED_Print_String("MPU6050 INIT FAIL", 0, 0, OLED_COLOR_WHITE);
	}
	OLED_Update_Display();
	delay(500);
	
	// Hiệu chỉnh con quay hồi chuyển (Gyro) trong MPU6050 (Bỏ comment nếu bị nhiễu)
	MPU6050_CalibrateGyro(&imu, 500);
	
	// Hiệu chỉnh gia tốc kế (Accel) trong MPU6050 (Bỏ comment nếu bị nhiễu)
	MPU6050_CalibrateAccel(&imu, 500);
	
	// Khởi tạo góc ban đầu
	MPU6050_ReadScaledData(&imu);
	float pitch_init = atan2f(-imu.ScaledData.Accel_X, sqrtf(imu.ScaledData.Accel_Z*imu.ScaledData.Accel_Z + imu.ScaledData.Accel_Y*imu.ScaledData.Accel_Y)) * (180.0f / PI);
	float roll_init  = atan2f(imu.ScaledData.Accel_Y, sqrtf(imu.ScaledData.Accel_X*imu.ScaledData.Accel_X + imu.ScaledData.Accel_Z*imu.ScaledData.Accel_Z)) * (180.0f / PI);
	Kalman_SetAngle(&kalmanPitch, pitch_init);
	Kalman_SetAngle(&kalmanRoll, roll_init);
	
	// Khởi chạy Timer ngắt (Vòng lặp ngầm 100Hz bắt đầu hoạt động)
	TIM2_Init();
	
	// Vòng lặp chính rảnh rỗi (Chỉ dùng để truyền thông UART, không ảnh hưởng đến điều khiển)
	while (1) 
	{	
		if (DataReady) {
			// Tạm tắt ngắt để copy dữ liệu an toàn (tránh bị ngắt ghi đè lúc đang in)
			__disable_irq();
			float ax = print_ax, ay = print_ay, az = print_az;
			float gx = print_gx, gy = print_gy, gz = print_gz;
			float p = print_pitch, r = print_roll;
			float targetAngle = 0.0f;
			DataReady = 0;
			
			OLED_Clear_Buffer();
			
			sprintf(oled_buf, "Pitch: %6.2f", p);
			OLED_Print_String(oled_buf, 0, 10, OLED_COLOR_WHITE);
			
			sprintf(oled_buf, "Roll : %6.2f", r);
			OLED_Print_String(oled_buf, 0, 30, OLED_COLOR_WHITE);
			
			OLED_Update_Display();
			
			// Thực hiện gửi dữ liệu lên máy tính để xem Plot (tần số 100Hz mượt mà)
			HC05_Send_IMU_Frame(&hc05Dev, targetAngle, p, ax, ay, az, gx, gy, gz);
			__enable_irq();
		}
	}
}