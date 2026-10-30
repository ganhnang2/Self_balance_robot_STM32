#include <stdio.h>
#include <string.h>
#include <stdlib.h> // Để dùng hàm rand()
#include <math.h>
#include <stdbool.h>
#include "stm32f4xx_lib.h"		// Thư viện các linh kiện 
#include "stm32f4xx.h"			// Các driver được phát triển
#include "pin_config.h"			// Cấu hình chân GPIO cho toàn bộ dự án
#include "nvic_config.h"		// Cấu hình interrupt và priority interrupt cho toàn bộ dự án

/*****************************************************************************/
/*							SYMBOLIC CONSTANTS								 */
/*****************************************************************************/
/* Hằng số PI */
#define PI						3.14159265f
#define SAMPLING_TIME			0.0100f				// Tần số lấy mẫu: 100Hz (10 ms)
#define ALPHA					0.98f

/*****************************************************************************/
/*							GLOABAL VARIABLES								 */
/*****************************************************************************/
MPU6050_t imu;
Kalman_t kalmanPitch;
Complementary_Filter_t cpFilter;
HC05_Device_t hc05Dev;

volatile uint8_t DataReady = 0;

// Theo dõi thời gian thực 
uint32_t timer = 0;

// Góc pitch khi được tính toán với các điều kiện khác nhau
float gyro_angle = 0.0f;			// Tính bằng góc Gyro
float accel_angle = 0.0f;			// Tính bằng gia tốc kế
float kalman_angle = 0.0f;			// Được lọc bằng thuật toán Kalman
float complementary_angle = 0.0f;	// Được lọc bằng thuật toán bù 

// Biến lưu trạng thái PWM ngẫu nhiên
int32_t pwmOutput = 0;
uint16_t motorTimer = 0;

// Lưu giữ các mức để xuất ra động cơ 
const int16_t pwmLevels[] = {-800, -650, -500, -350, -200, 0, 200, 350, 500, 650, 800};
#define NUM_PWM_LEVELS 		(sizeof(pwmLevels) / sizeof(pwmLevels[0]))

/* Chế độ điều khiển tốc độ động cơ */
typedef enum {
	SPEED_MODE_RAMP   = 0,	// 0: Tốc độ tăng giảm dần đều tuần tự
	SPEED_MODE_RANDOM = 1,	// 1: Tốc độ ngẫu nhiên trong các mức pwmLevels
	SPEED_MODE_STOP   = 2	// 2: Dừng động cơ
} MotorSpeedMode_t;

// Cấu hình chế độ ban đầu: Dừng động cơ, chờ lệnh từ máy tính
volatile MotorSpeedMode_t motorMode = SPEED_MODE_STOP;

// Cờ: 0 = Đang chờ lệnh khởi động từ máy tính, 1 = Đã nhận lệnh và đang chạy
volatile uint8_t motorEnabled = 1;

int8_t idx = 0;				// Chỉ số của pwmLevels để xuất ra động cơ
uint8_t direction = 0;		// 0: Tăng chỉ số, 1: Giảm chỉ số

/*****************************************************************************/
/*								FUNCTION									 */
/*****************************************************************************/
void TIMControlLoop_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;    			// Cấp xung nhịp cho TIM2
	
	TIM2->PSC = SystemCoreClock / 1000000UL - 1;	// Cấu hình tần số Timer về 1MHz (1 us/tick)
	TIM2->ARR = 10000 - 1;                 			// 10000 us = 10ms (100Hz)
	
	TIM2->DIER |= TIM_DIER_UIE;            			// Bật ngắt Update
	TIM2->CR1 |= TIM_CR1_CEN;              			// Chạy Timer
}

/****************************************************************************/
/*							INTERRUPT HANDLING								*/
/****************************************************************************/
// Trình phục vụ ngắt của USART2 để nhận dữ liệu về tham số Kp, Ki, Kd từ máy tính
void USART2_IRQHandler(void)
{
	HC05_IRQ_Handler(&hc05Dev);
}

// Nhiệm vụ gửi dữ liệu qua DMA đến máy tính để thực hiện nhận dữ liệu IMU
void DMA1_Stream6_IRQHandler(void)
{
	HC05_DMA_IRQHandler(&hc05Dev);
}

// Trình phục vụ ngắt của TIM2 (Hardware nhảy vào đây mỗi 10ms)
void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_UIF) { // Kiểm tra cờ ngắt Update
		TIM2->SR &= ~TIM_SR_UIF; // Xóa cờ ngắt
		
		motorTimer++;
		// Cứ sau mỗi 25 * 10ms = 250ms thì ta cập nhật tốc độ động cơ một lần
		if (motorTimer >= 25) 
		{
			motorTimer = 0;
			
			if (motorEnabled) // Chỉ điều khiển động cơ khi đã nhận lệnh từ máy tính
			{
				if (motorMode == SPEED_MODE_RANDOM)
				{
					int randIdx = rand() % NUM_PWM_LEVELS;
					pwmOutput = pwmLevels[randIdx];
				}
				else if (motorMode == SPEED_MODE_RAMP)
				{
					pwmOutput = pwmLevels[idx];

					if (direction == 0)
					{
						idx++;
						if (idx >= NUM_PWM_LEVELS - 1) 
						{
							idx = NUM_PWM_LEVELS - 1;
							direction = 1;
						}
					}
					else
					{
						idx--;
						if (idx <= 0) 
						{
							idx = 0;
							direction = 0;
						}
					}
				}
				else
				{
					pwmOutput = 0;
				}
				
				TB6612_Control_Motors(pwmOutput, pwmOutput);
			}
		}
		
		// Đọc dữ liệu IMU (chỉ thực hiện sau khi đã nhận lệnh khởi động)
		if (motorEnabled)
		{
			// Đọc dữ liệu (Tốn ~0.35ms)
			MPU6050_ReadScaledData(&imu);
			
			float ax = imu.ScaledData.Accel_X, ay = imu.ScaledData.Accel_Y, az = imu.ScaledData.Accel_Z;
			float gyroRate = imu.ScaledData.Gyro_Y;
			
			// Tính góc thô từ gia tốc kế
			float pitch_accel = atan2f(-ax, sqrtf(az*az + ay*ay)) * (180.0f / PI);	
			
			uint32_t currentTick = System_Get_Tick();
			float dt = (float)(currentTick - timer) / (float)SystemCoreClock;
			
			// Tính toán góc bằng gia tốc kế
			accel_angle = pitch_accel;
			kalman_angle = Kalman_GetAngle(&kalmanPitch, accel_angle, gyroRate, dt);
			gyro_angle = gyro_angle + gyroRate * dt;
			complementary_angle = Complementary_Compute(&cpFilter, complementary_angle, gyroRate, accel_angle, dt);
			
			// Cập nhật mốc thời gian
			timer = currentTick;
			
			// Báo hiệu có dữ liệu mới
			DataReady = 1;
		}
	}
}

int main()
{
	/****************************************************************/
	/*							INITS								*/
	/****************************************************************/
	// Khởi tạo chân GPIO cho tất cả các chân linh kiện
	Pin_Config_All();
	
	// Khởi tạo MPU6050 (Default: I2C1 peripheral)
	// NOTE: Theo datasheet của InvenSense MPU-6050/6500, mạch cấp nguồn bên trong và khối xử ý số (Digital Interface) 
	// cần từ 30ms đến 100ms để điện áp ổn định hoàn toàn trước khi bus I2C có thể phản hồi lệnh
	while (MPU6050_Init(&imu, NULL) != MPU6050_OK)
	{
		delay(1);
	}
	
	// Khởi tạo HC05 (Default: USART2 peripheral)
	HC05_Init(&hc05Dev, NULL);
	
	// Khởi tạo Encoder 
	Motor_Encoder_Init();
	
	// Khởi tạo driver TB6612FNG
	TB6612_Init();
	
	// Tắt hoàn toàn động cơ
	TB6612_Control_Motors(0, 0);
	
	// Bật cầu H sau khi mọi thứ ở trạng thái an toàn
	GPIO_Write_Pin(PIN_MOTOR_STBY_PORT, PIN_MOTOR_STBY_PIN, GPIO_PIN_SET);  
	
	// Hiệu chỉnh con quay hồi chuyển (Gyro) trong MPU6050 (Bỏ comment nếu bị nhiễu)
	MPU6050_CalibrateGyro(&imu, 500);
	
	// Hiệu chỉnh gia tốc kế (Accel) trong MPU6050 (Bỏ comment nếu bị nhiễu)
	// MPU6050_CalibrateAccel(&imu, 500);
	
	// Khởi tạo góc ban đầu để chống giật
	MPU6050_ReadScaledData(&imu);
	accel_angle = atan2f(-imu.ScaledData.Accel_X, sqrtf(imu.ScaledData.Accel_Z*imu.ScaledData.Accel_Z + imu.ScaledData.Accel_Y*imu.ScaledData.Accel_Y)) * (180.0f / PI);
	gyro_angle = 0.0f;
	complementary_angle = accel_angle;
	
	// Khởi tạo Complementary Filter nâng cao (có ước lượng trôi Gyro bias)
	Complementary_Init(&cpFilter);
	Complementary_Set_Alpha(&cpFilter, ALPHA);
	cpFilter.angle = accel_angle;
	
	// Khởi tạo Kalman Filter
	Kalman_Init(&kalmanPitch);
	Kalman_SetAngle(&kalmanPitch, accel_angle);
	
	// Cấu hình ngắt và độ ưu tiên ngắt (TIM2 - Control Loop, USART2 - Data exchange)
	NVIC_Config_All();
	
	/****************************************************************/
	/*          CHờ LỆNH KHỚI ĐỘNG TỪ MÁY TÍNH                   */
	/****************************************************************/
	// Gửi thông báo "sẵn sàng chờ" để máy tính biết STM32 đã khởi động xong
	const char* readyMsg = "READY:WAITING_CMD\r\n";
	USART_Write_DMA(&hc05Dev.usart, (uint8_t*)readyMsg, (uint16_t)strlen(readyMsg));
	
	uint8_t waitBuf[32];
	
	/*
	// Vòng lặp blocking: chỉ thoát khi nhận được lệnh chế độ hợp lệ từ máy tính
	while (motorEnabled == 0)
	{
		if (HC05_Receive_Frame(&hc05Dev, waitBuf))
		{
			if (waitBuf[0] == '1' || strstr((char*)waitBuf, "RAMP") != NULL)
			{
				motorMode   = SPEED_MODE_RAMP;
				motorEnabled = 1;
				const char* msg = "MODE:RAMP_OK\r\n";
				USART_Write_DMA(&hc05Dev.usart, (uint8_t*)msg, (uint16_t)strlen(msg));
			}
			else if (waitBuf[0] == '2' || strstr((char*)waitBuf, "RAND") != NULL)
			{
				motorMode   = SPEED_MODE_RANDOM;
				motorEnabled = 1;
				const char* msg = "MODE:RANDOM_OK\r\n";
				USART_Write_DMA(&hc05Dev.usart, (uint8_t*)msg, (uint16_t)strlen(msg));
			}
			else if (waitBuf[0] == '0' || strstr((char*)waitBuf, "STOP") != NULL)
			{
				// Chế độ STOP: vẫn cho phép khởi động nhưng động cơ dừng yên
				motorMode   = SPEED_MODE_STOP;
				motorEnabled = 1;
				const char* msg = "MODE:STOP_OK\r\n";
				USART_Write_DMA(&hc05Dev.usart, (uint8_t*)msg, (uint16_t)strlen(msg));
			}
		}
	}
	*/
	
	// Đặt lại mốc thời gian ngay trước khi khởi động Timer để dt chu kỳ đầu tiên chính xác
	timer = System_Get_Tick();
	
	// Khởi động Timer điều khiển - chỉ bắt đầu sau khi đã có lệnh từ máy tính
	TIMControlLoop_Init();
	
	char imuBuffer[128];
	uint8_t rxBuffer[32];
	
	while(1)
	{
		// Kiểm tra lệnh chuyển chế độ từ máy tính
		if (HC05_Receive_Frame(&hc05Dev, rxBuffer))
		{
			if (rxBuffer[0] == '1' || strstr((char*)rxBuffer, "RAMP") != NULL)
			{
				motorMode = SPEED_MODE_RAMP;
				const char* msg = "MODE:RAMP_OK\r\n";
				USART_Write_DMA(&hc05Dev.usart, (uint8_t*)msg, (uint16_t)strlen(msg));
			}
			else if (rxBuffer[0] == '2' || strstr((char*)rxBuffer, "RAND") != NULL)
			{
				motorMode = SPEED_MODE_RANDOM;
				const char* msg = "MODE:RANDOM_OK\r\n";
				USART_Write_DMA(&hc05Dev.usart, (uint8_t*)msg, (uint16_t)strlen(msg));
			}
			else if (rxBuffer[0] == '0' || strstr((char*)rxBuffer, "STOP") != NULL)
			{
				motorMode = SPEED_MODE_STOP;
				TB6612_Control_Motors(0, 0);
				const char* msg = "MODE:STOP_OK\r\n";
				USART_Write_DMA(&hc05Dev.usart, (uint8_t*)msg, (uint16_t)strlen(msg));
			}
		}

		if (DataReady) 
		{
			__disable_irq();
			float a_angle = accel_angle;
			float g_angle = gyro_angle;
			float k_angle = kalman_angle;
			float c_angle = complementary_angle;
			
			float ax = imu.ScaledData.Accel_X, ay = imu.ScaledData.Accel_Y, az = imu.ScaledData.Accel_Z;
			float gx = imu.ScaledData.Gyro_X, gy = imu.ScaledData.Gyro_Y, gz = imu.ScaledData.Gyro_Z;
			DataReady = 0;		// Reset lại biến trạng thái
			__enable_irq();
			
			// Cấu trúc IMU: IMU:<Gyro_angle>,<Accel_angle>,<complementary_angle>,<Kalman_angle>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\r\n
			int len = sprintf(imuBuffer, "IMU:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", g_angle, a_angle, c_angle, k_angle, ax, ay, az, gx, gy, gz);
				
			// Thực hiện gửi dữ liệu (gửi đúng số lượng byte vừa format)
			USART_Write(&hc05Dev.usart, (uint8_t*)imuBuffer, (uint16_t)len);
		}
	}
}