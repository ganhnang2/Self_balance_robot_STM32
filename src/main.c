/**
 * @file    main.c
 * @author  Nguyen Hai Nam
 * @brief   Chương trình chính điều khiển Xe Tự Cân Bằng 2 Bánh (Self-Balancing Robot).
 * 
 * @details Firmware điều khiển thời gian thực trên vi điều khiển STM32F401CCU6 (Black Pill):
 *          - Vòng lặp điều khiển kín (Closed-loop Control Loop) chạy trong ngắt TIM2 chu kỳ 20ms (50Hz).
 *          - Thu thập dữ liệu cảm biến MPU6050 qua I2C1 và ước lượng góc nghiêng bằng Kalman Filter.
 *          - Thuật toán điều khiển Cascade PID: Vòng Vận tốc (ngoài) + Vòng Cân bằng (trong).
 *          - Điều khiển 2 động cơ DC JGA25-370 có gắn Encoder qua driver cầu H TB6612FNG (TIM3 PWM 12.5kHz).
 *          - Đo đạc vận tốc qua 2 kênh Encoder phần cứng (TIM1 & TIM4 Encoder Mode).
 *          - Giao tiếp không dây qua module Bluetooth HC-05 (USART2) để nhận lệnh điều hướng và Tune thông số PID.
 *          - Hiển thị góc nghiêng và thông số hệ thống thời gian thực lên màn hình OLED 128x64 (I2C2).
 *
 * @note    Danh sách phần cứng sử dụng:
 *          1. MCU: STM32F401CCU6 (ARM Cortex-M4, Black Pill)
 *          2. Cảm biến góc: MPU-6050 (6-DOF IMU)
 *          3. Màn hình: SSD1306 OLED 128x64 (I2C)
 *          4. Mạch công suất: Driver TB6612FNG
 *          5. Động cơ: 2x Motor JGA25-370 có đĩa Encoder từ tính (Hall)
 *          6. Giao tiếp: Module Bluetooth HC-05 (UART)
 *
 * @note    
 * 1. Sơ đồ đấu nối chân (Pin Configuration):
 *          + TB6612FNG:
 *            - AIN1: PB13, AIN2: PB12 (Chiều quay Motor Phải)
 *            - BIN1: PB14, BIN2: PB15 (Chiều quay Motor Trái)
 *            - PWMA: PB4 (TIM3_CH1), PWMB: PB5 (TIM3_CH2) - PWM 12.5kHz
 *            - STBY: PA12 (Standby Enable)
 *          + Encoder (Đo vận tốc):
 *            - Motor Trái (TIM1): ENC1_A = PA8, ENC1_B = PA9
 *            - Motor Phải (TIM4): ENC2_A = PB7, ENC2_B = PB6
 *          + MPU-6050 (I2C1):
 *            - SCL: PB8, SDA: PB9
 *          + OLED SSD1306 (I2C2):
 *            - SCL: PB10, SDA: PB3
 *          + HC-05 Bluetooth (USART2):
 *            - TXD (MCU): PA2 -> RXD (HC-05)
 *            - RXD (MCU): PA3 <- TXD (HC-05)
 *            - KEY/STATE: PA1
 * 2. Công thức quy đổi từ xung/delta_ms sang vòng/phút:
 * 			DeltaN			60000
 * RPM = ----------- * -------------
 *			CPR				T_ms
 * trong đó:
 * - RPM	: Số xòng quay được trên một phút (Motor được sử dụng dự án này có rpm tối đa là 280)
 * - deltaN	: Số xung đọc được trong khoảng thời gian lấy mẫu (Ví dụ: 10ms)
 * - CPR   	: Số xung trên một vòng bánh xe (đã tính x4 và hộp số). Trong dự án này, CPR = 937 xung/vòng
 * - T_ms  	: Chu kỳ lấy mẫu tính bằng ms
 * 
 * Ví dụ: T = 25ms, DeltaN = 110 xung (Xung tối đa đo được trong 40ms)
 * 			110			60000
 * RPM = ----------- * ------- = 0.113 * 2400 = 281.7 rpm (Xấp xỉ 280 rpm) 
 *			937			 25
 * => Kết quả này khớp tốc độ không tải 280 RPM trong datasheet.
 * 3. Quy đổi từ rpm sang m/s
 * 			RPM
 * v = ------------ * 2 * PI * R
 *			60
 * trong đó:
 * - v		: Vận tốc của bánh xe (đơn vị: m/s)
 * - RPM	: Số vòng/phút của bánh xe (không phải trục motor trước hộp số)
 * - R		: Bán kính bánh xe tính bằng mét (D = 65mm -> R = 0.0325 m)
 * 4. Quy đổi từ giá trị PWM Output (0 - 1250) ra tốc độ động cơ
 *			PWM Output
 * RPM = --------------- * 281
 *			  1250
 */

#include <stdio.h>
#include <string.h>
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

/* Góc cân bằng tĩnh */
#define ANGLE_BIAS				0.456f

#define ALPHA					0.98f

/* Hằng số áp dụng cho PID vận tốc (PID VELOCITY) */
#define MAX_TARGET_ANGLE		5.0f			// Độ nghiêng tối đa được phép (targetAngle): 5 độ
#define SPEED_MAX_STEP			219.0f			// Vận tốc tối đa mong muốn (xung/50ms)
#define SPEED_ACCEL_RATE		0.25f			// Gia tốc cho độ tăng của góc đặt
#define	PID_VEL_DT				0.05f			// Tần số lấy mẫu: 10Hz (100 ms)

/* Hằng số áp dụng vòng PID cân bằng (PID BALANCE) */
#define FALL_ANGLE				45.0f			// Góc khiến cho xe bị ngã
#define MOTOR_DEADBAND			100				// Vùng deadband - motor không thể quay được (Lấy từ sau khi thực nghiệm)
#define MOTOR_DEADBAND_THRES 	8				// Chỉ bù deadband khi |lệnh| lớn hơn ngưỡng này
#define MOTOR_MAX_SPEED			2000			// Tốc độ tối đa được phép để điều khiển motor
#define PID_BALANCE_DT			0.005f			// Tần số lấy mẫu: 200Hz (5 ms)

/* Hằng số áp dụng cho vòng rẽ hướng (PID STEERING) */
#define TARGET_YAW_RATE      	60.0f  			// Tốc độ xoay mong muốn khi rẽ (độ/giây)
#define MAX_TURN_PWM         	250	    		// Giới hạn PWM tối đa dành cho rẽ
#define PID_STEERING_DT			0.01f			// Tần số lấy mẫu: 100Hz (10 ms)

/* Giới hạn hợp lệ của dt đo được (giây) */
#define DT_MAX_VALID			0.05f			

/*****************************************************************************/
/*								DATA TYPES									 */
/*****************************************************************************/
typedef enum {
    BCT_STATE_INIT      = 0,
    BCT_STATE_IDLE      = 1,
    BCT_STATE_BALANCING = 2,
    BCT_STATE_FALLEN    = 3
} Robot_State_t;

/*****************************************************************************/
/*							GLOBAL VARIABLES								 */
/*****************************************************************************/
MPU6050_t imu;
PID_t pidVelocity;
PID_t pidBalance;
PID_t pidSteering;
Kalman_t kalmanPitch;
Complementary_Filter_t cp;
HC05_Device_t hc05Dev;

volatile HC05_DirectionCode_t direction = DIR_NONE;
volatile uint8_t DataReady = 0;

// Hai biến để kiểm tra thời gian thực thi (Debug chương trình)

// Tổng thời gian thực hiện ngắt (us) -> 602us ~= 0.6ms (Debug)
// volatile uint32_t isr_execution_time_us = 0;				

// Tổng thời gian thực hiện việc in lên màn hình OLED + USART2 (us) theo kiểu Polling -> 53356us ~= 53.356ms (Debug)	
// volatile uint32_t display_execution_time_us = 0;				
									
// Pith and Roll angle of robot
volatile float pitch = 0.0f;

// Các biến toàn cục được sử dụng trong hàm và ngắt
volatile uint32_t timer = 0;					// Timer để thực hiện đo chính xác thời gian xảy ra giữa 2 lần ngắt
volatile float currentTargetAngle = 0.0f;		// Góc mục tiêu sau khi Ramp
volatile float targetAngle = 0.0f;				// Góc mục tiêu từ vòng Velocity (ISR())
volatile float filteredSpeed = 0.0f;			// Tốc độ sau LPF (ISR())
volatile uint8_t velCount = 0;					// Bộ đếm kích hoạt vòng Velocity (ISR())

/*****************************************************************************/
/*								FUNCTION									 */
/*****************************************************************************/
void TIMControlLoop_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;    			// Cấp xung nhịp cho TIM2
	
	TIM2->PSC = SystemCoreClock / 1000000UL - 1;	// Cấu hình tần số Timer về 1MHz (1 us/tick)
	TIM2->ARR = 5000 - 1;                 			// 5000 us = 5ms (200Hz)
	TIM2->DIER |= TIM_DIER_UIE;            			// Bật ngắt Update
	TIM2->CR1 |= TIM_CR1_CEN;              			// Chạy Timer
}

void Robot_Reset_All(void)
{
	PID_Reset_State(&pidVelocity);
	PID_Reset_State(&pidBalance);
	PID_Reset_State(&pidSteering);
	
	currentTargetAngle = 0.0f;
	filteredSpeed = 0.0f;
	targetAngle = 0.0f;
	velCount = 0UL;
}

int32_t Robot_Apply_Deadband(int32_t cmd)
{
	int32_t magnitude = (cmd >= 0) ? cmd : -cmd;

	if (magnitude <= MOTOR_DEADBAND_THRES)
	{
		return 0;
	}

	// Tăng bù tuyến tính từ 0 đến MOTOR_DEADBAND.
	int32_t compensation = MOTOR_DEADBAND * (magnitude - MOTOR_DEADBAND_THRES) / (MOTOR_MAX_SPEED - MOTOR_DEADBAND_THRES);
	int32_t output = magnitude + compensation;

	if (output > MOTOR_MAX_SPEED)
	{
		output = MOTOR_MAX_SPEED;
	}

	return (cmd >= 0) ? output : -output;
}

int32_t Robot_Clamp_I32(int32_t value, int32_t limit)
{
	if (value > limit)
	{
		return limit;
	}
	if (value < -limit)
	{
		return -limit;
	}
	return value;
}

/* - Vòng PID này nhận input: 
		+) input: targetAngle (Đơn vị: độ), currentAngle (Đơn vị: độ), pitchRate (Đơn vị: rad/s)
		+) output: PWM Output (Đơn vị: giá trị so sánh thuộc [-1500, 1500] - Timer sử dụng để băm xung PWM)
   - Góc quay được sử dụng ở đây là Pitch (trục Y) 
   - Quy ước: Góc dương, động cơ cũng sẽ xuất xung dương và ngược lại.
*/
int32_t PID_Balance(float targetAngle, float currentAngle, float dt)
{	
	int32_t baseCmd = (int32_t)PID_Compute_Advanced(&pidBalance, targetAngle, currentAngle, dt); 
	return baseCmd;
}

/****************************************************************************/
/*							INTERRUPT HANDLING								*/
/****************************************************************************/
// Trình phục vụ ngắt của USART2 để nhận dữ liệu về tham số Kp, Ki, Kd từ máy tính
void USART2_IRQHandler(void)
{
	HC05_IRQ_Handler(&hc05Dev);
}

// Trình phục vụ ngắt của I2C2_TX để thực hiện việc truyền dữ liệu lên OLED128x64
void DMA1_Stream7_IRQHandler(void)
{
	OLED_DMA_IRQ_Handler();
}

// Trình phục vụ ngắt của TIM2 (Hardware nhảy vào đây mỗi 10ms)
void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_UIF) { // Kiểm tra cờ ngắt Update
		TIM2->SR &= ~TIM_SR_UIF; // Xóa cờ ngắt

		// Đọc dữ liệu (Tốn ~0.35ms)
		MPU6050_ReadScaledData(&imu);
		
		float ax = imu.ScaledData.Accel_X, ay = imu.ScaledData.Accel_Y, az = imu.ScaledData.Accel_Z;
		float gx = imu.ScaledData.Gyro_X, gy = imu.ScaledData.Gyro_Y, gz = imu.ScaledData.Gyro_Z;
		
		// Tính góc thô từ gia tốc kế
		float accelAngle = atan2f(-ax, sqrtf(az*az + ay*ay)) * (180.0f / PI);
		
		uint32_t currentTick = System_Get_Tick();
		float dt = (float)(uint32_t)(currentTick - timer) / (float)SystemCoreClock;
		
		if (dt <= 0.0f || dt > DT_MAX_VALID)
		{
			dt = PID_BALANCE_DT;
		}
		
		// Cập nhật Kalman Filter (dt chính xác tuyệt đối = 0.010s)
		// float pitch_kalman = Kalman_GetAngle(&kalmanPitch, pitch_kalman, gy, dt);
		cp.angle = Complementary_Compute(&cp, cp.angle, gy, accelAngle, dt);
		
		// Cập nhật mốc thời gian
		timer = currentTick;
		
		// Đẩy ra biến toàn cục để main() có thể in
		pitch = cp.angle;
		
		// Báo hiệu có dữ liệu mới
		DataReady = 1; 

		// Chốt chặn an toàn: Ngắt khẩn cấp động cơ nếu góc nghiêng quá lớn (xe bị ngã)
		if ((cp.angle > FALL_ANGLE) || (cp.angle < -FALL_ANGLE))
		{
			TB6612_Short_Brake();		// Cho xe dừng hẳn
			
			// Reset lại trạng thái Robot
			Robot_Reset_All();
			return;
		}

		/* Hai thông số cho vòng kín PID cân bằng */
		/* targetAngle được cập nhật từ vòng PID vận tốc (20Hz/ 50ms) */
		float currentAngle = 0.0f;					// Actual (Được đo bằng MPU6050)
		
		int32_t baseMotorSpeed = 0;
		int32_t leftMotorSpeed = 0;
		int32_t rightMotorSpeed = 0;
		
		/************************PID BALANCE (PD CONTROLLER)****************************/
		// 1. Chọn trục nghiêng điều khiển (trong dự án này là trục Y - Pitch)
		currentAngle = cp.angle + ANGLE_BIAS;

		// 2. Tính toán thuật toán PID
		baseMotorSpeed = PID_Balance(0.0f, currentAngle, dt);
		
		/***********************************PWM OUTPUT*****************************/
		// Phân phối vi sai cho 2 bánh
		leftMotorSpeed  = baseMotorSpeed;
		rightMotorSpeed = baseMotorSpeed;
		
		// Bù deadband cho 2 động cơ
		leftMotorSpeed  = Robot_Apply_Deadband(leftMotorSpeed);
		rightMotorSpeed = Robot_Apply_Deadband(rightMotorSpeed);
		
		// Giới hạn công suất tối đa của motor [-1250, 1250] (Clamping)
		leftMotorSpeed  = Robot_Clamp_I32(leftMotorSpeed,  MOTOR_MAX_SPEED);
		rightMotorSpeed = Robot_Clamp_I32(rightMotorSpeed, MOTOR_MAX_SPEED);
		
		// Xuất điều khiển 2 bánh qua cầu H TB6612
		TB6612_Control_Motors((int32_t)leftMotorSpeed, (int32_t)rightMotorSpeed);
	}
}

/****************************************************************************/
/*							MAIN FUNCTION									*/
/****************************************************************************/
int main()
{
	/*************************************************************************************/
	/*									  INITS										 	 */
	/*************************************************************************************/
	
	/*************************************GPIO******************************************/
	// Khởi tạo chân GPIO cho tất cả các chân linh kiện
	Pin_Config_All();

	/*************************************IMU*******************************************/
	// Khởi tạo MPU6050 (Default: I2C1 peripheral)
	// NOTE: Theo datasheet của InvenSense MPU-6050/6500, mạch cấp nguồn bên trong và khối xử ý số (Digital Interface) 
	// cần từ 30ms đến 100ms để điện áp ổn định hoàn toàn trước khi bus I2C có thể phản hồi lệnh
	uint8_t count = 0;
	while (MPU6050_Init(&imu, NULL) != MPU6050_OK)
	{
		if (count >= 500)
			break;
		 count++;
		delay(1);
	}
	
	// Hiệu chỉnh con quay hồi chuyển (Gyro) trong MPU6050 (Bỏ comment nếu bị nhiễu)
	MPU6050_CalibrateGyro(&imu, 500);
	
	// Hiệu chỉnh gia tốc kế (Accel) trong MPU6050 (Bỏ comment nếu bị nhiễu)
	// MPU6050_CalibrateAccel(&imu, 500);
	
	/*************************************HC05*******************************************/
	// Khởi tạo HC05 (Default: USART2 peripheral)
	HC05_Init(&hc05Dev, NULL);
	
	/*************************************OLED*******************************************/
	// Khởi tạo OLED (Default: I2C2 peripheral)
	// Truyền dữ liệu bằng DMA + I2C2
	OLED_Init(NULL);
	OLED_DMA_Init(NULL);
	
	/*************************************MOTOR******************************************/
	// Khởi tạo Encoder 
	Motor_Encoder_Init();
	
	// Khởi tạo driver TB6612FNG
	TB6612_Init();
	
	// Tắt hoàn toàn động cơ
	TB6612_Control_Motors(0, 0);
	
	// Bật cầu H sau khi mọi thứ ở trạng thái an toàn
	GPIO_Write_Pin(PIN_MOTOR_STBY_PORT, PIN_MOTOR_STBY_PIN, GPIO_PIN_SET);  
	
	/*************************************PID*******************************************/
	// Khởi tạo 2 bộ điều khiển PID
	// PID Angle: Bộ điều khiển có đầu ra là target angle
	// PID Balance: Bộ điểu khiển đầu ra xuất tín hiệu ra động cơ để bám target angle
	// PID Steering: Cho việc phân phối vi sai cho cả 2 bánh
	PID_Init(&pidVelocity);
	PID_Init(&pidBalance);
	PID_Init(&pidSteering);
	
	// PID Velocity (PI Controller)
	PID_Set_Max_Output(&pidVelocity, MAX_TARGET_ANGLE);
	PID_Set_Max_Integral(&pidVelocity, 0.85f * MAX_TARGET_ANGLE);
	PID_Set_Sampling_Time(&pidVelocity, PID_VEL_DT);
	
	// PID Balance (PID Controller)
	PID_Set_Max_Output(&pidBalance, MOTOR_MAX_SPEED);		
	PID_Set_Max_Integral(&pidBalance, 0.85f * MOTOR_MAX_SPEED);
	PID_Set_Sampling_Time(&pidBalance, PID_BALANCE_DT);
	
	// PID Steering (PD Controller)
	PID_Set_Max_Output(&pidSteering, MAX_TURN_PWM);
	PID_Set_Sampling_Time(&pidSteering, PID_STEERING_DT);
	
	/********************************KALMAN FILTER*************************************/
	// Khởi tạo Kalman Filter
	Kalman_Init(&kalmanPitch);
	
	// Khởi tạo góc ban đầu để chống giật
	// MPU6050_ReadScaledData(&imu);
	// float initPitch = atan2f(-imu.ScaledData.Accel_X, sqrtf(imu.ScaledData.Accel_Z*imu.ScaledData.Accel_Z + imu.ScaledData.Accel_Y*imu.ScaledData.Accel_Y)) * (180.0f / PI);
	// pitch = initPitch;
	// Kalman_SetAngle(&kalmanPitch, initPitch);
	
	// Khởi tạo bộ lọc bù
	Complementary_Init(&cp);
	Complementary_Set_Alpha(&cp, ALPHA);
	cp.angle = 0.0f;
	
	/**********************************INTERRUPT*************************************/
	// Cấu hình ngắt và độ ưu tiên ngắt (TIM2 - Control Loop, USART2 - Data exchange)
	NVIC_Config_All();
	
	// In ra khởi tạo
	OLED_Clear_Buffer();
	OLED_Print_String("Init OK!", 0, 25, OLED_COLOR_WHITE);
	OLED_Update_Display_DMA();
	
	// Khởi tạo Timer cho chương trình
	TIMControlLoop_Init();
	timer = System_Get_Tick();			// Đo đạc bằng Timer
	
	float kp = 0.0f, ki = 0.0f, kd = 0.0f;
	
	/*************************************************************************************/
	/*									  LOOP										 	 */
	/*************************************************************************************/
	while (1)
	{
		// Xác nhận xem có dữ liệu mới được gửi về
		if (HC05_Receive_Frame(&hc05Dev, NULL) != 0)
		{
			// Tham số PID Tuning
			if (HC05_Is_Parameter_New(&hc05Dev) != 0)
			{
				HC05_PID_Target_t target = HC05_Get_PID_Target(&hc05Dev);
				HC05_Get_Tuning_Parameters(&hc05Dev, &kp, &ki, &kd);
				
				if (target == PID_TARGET_BALANCE)
				{
					pidBalance.kp = kp;
					pidBalance.ki = ki;
					pidBalance.kd = kd;
				}
				else if (target == PID_TARGET_VELOCITY)
				{
					// Đầu vào: số xung/100ms (khoảng 0 - 439) và đầu ra: degree (khoảng 0 - 10 độ)
					// Do đó, để việc tính toán cxac thì ta lấy tham số Tuning chia cho 100
					pidVelocity.kp = kp / 100.0f;
					pidVelocity.ki = ki / 100.0f;
					pidVelocity.kd = kd / 100.0f;
				}
				else if (target == PID_TARGET_STEERING)
				{
					pidSteering.kp = kp;
					pidSteering.ki = ki;
					pidSteering.kd = kd;
				}
				else
				{
					pidBalance.kp = kp;
					pidBalance.ki = ki;
					pidBalance.kd = kd;
				}
			}
			
			// Điều khiển hướng đi
			if (HC05_Is_Direction_New(&hc05Dev) != 0)
			{
				direction = HC05_Get_Direction(&hc05Dev);
			}
		}
		
		if (DataReady) 
		{	
			// Tắt ngắt ngắn gọn CHỈ để copy dữ liệu volatile (không làm gì khác trong critical section)
			__disable_irq();		
			//float ax = imu.ScaledData.Accel_X, ay = imu.ScaledData.Accel_Y, az = imu.ScaledData.Accel_Z;
			//float gx = imu.ScaledData.Gyro_X, gy = imu.ScaledData.Gyro_Y, gz = imu.ScaledData.Gyro_Z;
			//float targetAngle = setpoint;
			float p = pitch;
			DataReady = 0;		
			__enable_irq();			// Mở ngắt NGAY sau khi copy xong — DMA IRQ có thể phục vụ từ đây
			
			// Hiển thị lên màn hình OLED
			char oled_buf[128];
			
			// Thực hiện update dữ liệu lên màn hình
			if (OLED_Get_State_DMA() == OLED_DMA_STATE_IDLE)
			{
				OLED_Clear_Buffer();
				sprintf(oled_buf, "Pitch [deg]: %.3f", p);
				OLED_Print_String(oled_buf, 0, 25, OLED_COLOR_WHITE);
				sprintf(oled_buf, "Kp : %.3f, Ki: %.3f, Kd: %.3f", kp, ki, kd);
				OLED_Print_String(oled_buf, 0, 45, OLED_COLOR_WHITE);
				OLED_Update_Display_DMA();
			}
			
			// Thực hiện gửi dữ liệu lên máy tính để xem Plot
			// HC05_Send_IMU_Frame(&hc05Dev, targetAngle, p, ax, ay, az, gx, gy, gz);
		}
	}
}