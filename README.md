# STM32F401CCU6 Bare-Metal & FreeRTOS Development Environment

## 1. Overview
This repository contains the embedded software architecture for the **Self-Balancing Robot** project, powered by the **STM32F401CCU6 (ARM Cortex-M4 with single-precision FPU, 256 KB Flash, 64 KB SRAM)**.

Unlike high-level vendor HAL/LL frameworks, this codebase is built **from the ground up** using bare-metal register manipulation and CMSIS standard headers. It provides an optimized, deterministic, and educational platform for:
- Direct hardware register programming (Clock tree, GPIO, Timer PWM/Encoder, I2C, SPI, USART, ADC).
- Sensor fusion algorithms (MPU6050 6-DoF IMU via Kalman / Complementary Filters).
- Real-time closed-loop motion control (Cascaded PID, DC motor PWM drive via TB6612FNG, quadrature optical/magnetic wheel encoders).
- Dual-execution modes: Bare-metal superloop and FreeRTOS preemptive multitasking.

---

## 2. Directory Structure

```text
Code/
├── CMSIS/                    # ARM Cortex-M4 CMSIS core, device registers & FreeRTOS
├── config/                   # System pinout and interrupt (NVIC) configurations
│   ├── pin_config.c / .h     # Centralized hardware GPIO pin mapping & initialization
│   └── nvic_config.c / .h    # Interrupt priority and vector table configuration
├── driver/                   # User-developed bare-metal microcontroller peripheral drivers
├── lib/                      # External component/device libraries (Modular packages)
│   ├── MPU6050/              # 6-DoF IMU driver (Accelerometer & Gyroscope)
│   ├── TB6612/               # Dual H-Bridge motor driver library
│   ├── OLED/                 # SSD1306 128x64 display library
│   ├── HC05/                 # Bluetooth wireless telemetry & control
│   └── Filter & PID/         # Kalman, Complementary filter & Cascaded PID
├── src/                      # Main application source files
│   ├── main.c                # System entry point, initialization & closed-loop control
│   ├── balance_control.c     # Unified state-machine balancing algorithm
│   └── syscalls.c            # POSIX system call stubs for printf/malloc (Newlib)
├── startup/                  # Architecture startup files
│   └── startup_stm32f401.c   # Vector table, hardware FPU enable, and Reset_Handler
├── tools/                    # Python scripts to support development phase
├── stm32f401ccu6.ld          # Custom linker script (256KB Flash, 64KB SRAM layout)
├── Makefile.mak              # Primary build script for main application
└── README.md                 # Project documentation (this file)
```

---

## 3. Hardware Architecture & Detailed Pinout Mapping (Sơ đồ nối chân phần cứng)

### 3.1. Danh sách linh kiện phần cứng (Bill of Materials - BOM)
1. **Vi điều khiển trung tâm (MCU)**: STM32F401CCU6 "Black Pill" (ARM Cortex-M4, 84 MHz, 3.3V Logic).
2. **Cảm biến góc nghiêng (IMU)**: MPU-6050 (6-DOF: Gyroscope 3 trục + Accelerometer 3 trục).
3. **Màn hình hiển thị**: SSD1306 OLED 0.96 inch I2C (Độ phân giải 128x64 pixels).
4. **Mạch điều khiển động cơ**: TB6612FNG Dual H-Bridge Driver (Hiệu suất cao, tỏa nhiệt thấp hơn L298N).
5. **Động cơ & Encoder**: 2x Động cơ DC JGA25-370 (12V / ~280 RPM, tích hợp đĩa Encoder Hall 2 pha A/B, CPR = 937 xung/vòng).
6. **Truyền thông không dây**: Module Bluetooth HC-05 (UART Serial để nhận lệnh và Tune PID từ xa).
7. **Nguồn cấp**: Pin LiPo / Li-ion (2S 7.4V hoặc 3S 11.1V - 12V), mạch hạ áp Buck DC-DC (LM2596 / Mini-360) hạ xuống 5V.

---

### 3.2. Bảng tổng hợp sơ đồ nối chân STM32F401CCU6

| Chân STM32 | Tên Ngoại vi (Peripheral) | Chế độ GPIO (Mode) | Linh kiện kết nối | Chân trên linh kiện | Chức năng / Ghi chú |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **PC13** | GPIO Output | Output Push-Pull | On-Board LED | LED âm (Cathode) | LED báo trạng thái hệ thống (Active-Low) |
| **PA1**  | GPIO Output | Output Push-Pull | HC-05 Bluetooth | KEY / EN | Chuyển chế độ: High = AT Mode, Low = Data Mode |
| **PA2**  | USART2_TX (AF7) | Alt-Function Push-Pull | HC-05 Bluetooth | RXD | Truyền dữ liệu telemetry / debug từ STM32 sang HC-05 |
| **PA3**  | USART2_RX (AF7) | Alt-Function Pull-up | HC-05 Bluetooth | TXD | Nhận lệnh điều khiển / thông số tune PID từ HC-05 |
| **PA8**  | TIM1_CH1 (AF1) | Alt-Function Pull-up | Động cơ Trái (Encoder) | Pha A (C1) | Đọc xung quadrature kênh 1 Encoder động cơ Trái |
| **PA9**  | TIM1_CH2 (AF1) | Alt-Function Pull-up | Động cơ Trái (Encoder) | Pha B (C2) | Đọc xung quadrature kênh 2 Encoder động cơ Trái |
| **PA12** | GPIO Output | Output Push-Pull | Driver TB6612FNG | STBY | Kích hoạt mạch cầu H (High = Hoạt động, Low = Standby/Ngắt) |
| **PA13** | SWDIO | Alt-Function Debug | ST-Link v2 Programmer | SWDIO | Nạp code và debug thời gian thực qua chuẩn SWD |
| **PA14** | SWCLK | Alt-Function Debug | ST-Link v2 Programmer | SWCLK | Xung nhịp đồng hồ cho nạp code và debug |
| **PB3**  | I2C2_SDA (AF9) | Alt-Function Open-Drain | OLED SSD1306 | SDA | Đường truyền dữ liệu I2C2 cho màn hình OLED |
| **PB4**  | TIM3_CH1 (AF2) | Alt-Function Push-Pull | Driver TB6612FNG | PWMA | Băm xung điều khiển tốc độ Motor Phải (PWM 12.5 kHz) |
| **PB5**  | TIM3_CH2 (AF2) | Alt-Function Push-Pull | Driver TB6612FNG | PWMB | Băm xung điều khiển tốc độ Motor Trái (PWM 12.5 kHz) |
| **PB6**  | TIM4_CH1 (AF2) | Alt-Function Pull-up | Động cơ Phải (Encoder) | Pha B (C2) | Đọc xung quadrature kênh 1 Encoder động cơ Phải |
| **PB7**  | TIM4_CH2 (AF2) | Alt-Function Pull-up | Động cơ Phải (Encoder) | Pha A (C1) | Đọc xung quadrature kênh 2 Encoder động cơ Phải |
| **PB8**  | I2C1_SCL (AF4) | Alt-Function Open-Drain | Cảm biến MPU-6050 | SCL | Xung nhịp đồng hồ I2C1 đọc dữ liệu IMU (Fast Mode 400kHz) |
| **PB9**  | I2C1_SDA (AF4) | Alt-Function Open-Drain | Cảm biến MPU-6050 | SDA | Đường dữ liệu I2C1 đọc gia tốc và vận tốc góc |
| **PB10** | I2C2_SCL (AF4) | Alt-Function Open-Drain | OLED SSD1306 | SCL | Xung nhịp đồng hồ I2C2 cho màn hình OLED |
| **PB12** | GPIO Output | Output Push-Pull | Driver TB6612FNG | AIN2 | Tín hiệu logic chọn chiều quay Motor Phải (AIN1/AIN2) |
| **PB13** | GPIO Output | Output Push-Pull | Driver TB6612FNG | AIN1 | Tín hiệu logic chọn chiều quay Motor Phải (AIN1/AIN2) |
| **PB14** | GPIO Output | Output Push-Pull | Driver TB6612FNG | BIN1 | Tín hiệu logic chọn chiều quay Motor Trái (BIN1/BIN2) |
| **PB15** | GPIO Output | Output Push-Pull | Driver TB6612FNG | BIN2 | Tín hiệu logic chọn chiều quay Motor Trái (BIN1/BIN2) |

---

### 3.3. Chi tiết kết nối theo từng cụm linh kiện

#### 1. Cảm biến MPU-6050 (Giao tiếp I2C1)
* **VCC** $\rightarrow$ Nối nguồn **3.3V** (hoặc 5V nếu module có IC ổn áp 662K).
* **GND** $\rightarrow$ Nối Mass (**GND**).
* **SCL** $\rightarrow$ Nối chân **PB8** (I2C1_SCL).
* **SDA** $\rightarrow$ Nối chân **PB9** (I2C1_SDA).
* **AD0** $\rightarrow$ Nối **GND** (Chọn địa chỉ I2C mặc định `0x68`).

#### 2. Màn hình OLED SSD1306 (Giao tiếp I2C2)
* **VCC** $\rightarrow$ Nối nguồn **3.3V** hoặc **5V**.
* **GND** $\rightarrow$ Nối Mass (**GND**).
* **SCL** $\rightarrow$ Nối chân **PB10** (I2C2_SCL).
* **SDA** $\rightarrow$ Nối chân **PB3** (I2C2_SDA, cấu hình Alternate Function AF9).

#### 3. Mạch điều khiển động cơ TB6612FNG
* **Phần điều khiển logic (Tín hiệu từ STM32)**:
  * **STBY** $\rightarrow$ Nối chân **PA12** (Cần kéo High để bật driver).
  * **PWMA** $\rightarrow$ Nối chân **PB4** (TIM3_CH1 - Điều khiển tốc độ bánh Phải).
  * **AIN1** $\rightarrow$ Nối chân **PB13** (Chiều quay bánh Phải).
  * **AIN2** $\rightarrow$ Nối chân **PB12** (Chiều quay bánh Phải).
  * **PWMB** $\rightarrow$ Nối chân **PB5** (TIM3_CH2 - Điều khiển tốc độ bánh Trái).
  * **BIN1** $\rightarrow$ Nối chân **PB14** (Chiều quay bánh Trái).
  * **BIN2** $\rightarrow$ Nối chân **PB15** (Chiều quay bánh Trái).
* **Phần nguồn & Công suất động cơ**:
  * **VM** $\rightarrow$ Nối nguồn pin động cơ (**7.4V - 12V DC**).
  * **VCC** $\rightarrow$ Nối nguồn logic **3.3V** hoặc **5V**.
  * **GND** $\rightarrow$ Nối chung **GND** toàn hệ thống.
  * **AO1, AO2** $\rightarrow$ Nối 2 dây nguồn động cơ DC Phải.
  * **BO1, BO2** $\rightarrow$ Nối 2 dây nguồn động cơ DC Trái.

#### 4. Đĩa xung Encoder (Động cơ JGA25-370)
* **Encoder Động cơ Trái (Left Motor)**:
  * **VCC** $\rightarrow$ Nối nguồn **3.3V** hoặc **5V**.
  * **GND** $\rightarrow$ Nối Mass (**GND**).
  * **Phase A (C1)** $\rightarrow$ Nối chân **PA8** (TIM1_CH1, Hardware Encoder Mode).
  * **Phase B (C2)** $\rightarrow$ Nối chân **PA9** (TIM1_CH2, Hardware Encoder Mode).
* **Encoder Động cơ Phải (Right Motor)**:
  * **VCC** $\rightarrow$ Nối nguồn **3.3V** hoặc **5V**.
  * **GND** $\rightarrow$ Nối Mass (**GND**).
  * **Phase A (C1)** $\rightarrow$ Nối chân **PB7** (TIM4_CH2, Hardware Encoder Mode).
  * **Phase B (C2)** $\rightarrow$ Nối chân **PB6** (TIM4_CH1, Hardware Encoder Mode).

#### 5. Module Bluetooth HC-05 (Giao tiếp USART2)
* **VCC** $\rightarrow$ Nối nguồn **5V**.
* **GND** $\rightarrow$ Nối Mass (**GND**).
* **TXD (HC-05)** $\rightarrow$ Nối chân **PA3** (USART2_RX của STM32).
* **RXD (HC-05)** $\rightarrow$ Nối chân **PA2** (USART2_TX của STM32).
* **KEY / EN** $\rightarrow$ Nối chân **PA1** (Điều khiển vào chế độ AT hoặc Data).

---

### 3.4. Sơ đồ khối kết nối hệ thống (System Wiring Diagram)

```text
                      +-----------------------------+
                      |   Pin Li-ion / LiPo 2S-3S   |
                      |         (7.4V - 12V)        |
                      +--------------+--------------+
                                     |
               +---------------------+---------------------+
               |                                           |
               v (VM: 7.4V - 12V)                          v
     +-------------------+                       +-------------------+
     | Driver TB6612FNG  |                       | Mạch Buck Hạ áp   |
     +---------+---------+                       |    (LM2596/360)   |
               |                                 +---------+---------+
    AO1/AO2    |    BO1/BO2                                | 5V DC
       v       |       v                                   v
  +---------+  |  +---------+                    +-------------------+
  | Motor R |  |  | Motor L |                    | Nguồn 5V Hệ Thống |
  +----+----+  |  +----+----+                    +---------+---------+
       |       |       |                                   |
       |       |       +-------------------+               +------------+
       |       |                           |               |            |
       v       v                           v               v            v
+--------------+-------------------------------------------+---+   +-----------+
|                           STM32F401CCU6                      |   | HC-05 BT  |
|                                                              |   | VCC: 5V   |
| [TIM3 PWM]     PB4  --> PWMA        PB5  --> PWMB            |   | TXD->PA3  |
| [Direction]    PB13 --> AIN1        PB14 --> BIN1            |   | RXD<-PA2  |
|                PB12 --> AIN2        PB15 --> BIN2            |   | KEY<-PA1  |
| [Standby]      PA12 --> STBY                                 |   +-----------+
|                                                              |
| [TIM1 Encoder] PA8  <-- Left Enc Phase A                     |   +-----------+
|                PA9  <-- Left Enc Phase B                     |   | OLED I2C2 |
| [TIM4 Encoder] PB7  <-- Right Enc Phase A                    |   | SCL: PB10 |
|                PB6  <-- Right Enc Phase B                    |   | SDA: PB3  |
|                                                              |   +-----------+
| [I2C1 Sensor]  PB8  <--> MPU6050 SCL                         |
|                PB9  <--> MPU6050 SDA                         |   +-----------+
|                                                              |   | MPU-6050  |
| [ST-Link SWD]  PA13 <--> SWDIO                               |   | SCL: PB8  |
|                PA14 <--> SWCLK                               |   | SDA: PB9  |
| [Built-in LED] PC13 ---> On-board Blue LED                   |   +-----------+
+--------------------------------------------------------------+
         * LƯU Ý QUAN TRỌNG: NỐI CHUNG TẤT CẢ DÂY MASS (COMMON GND)
```

---

## 4. Toolchain & Prerequisites

Ensure the following tools are installed and accessible in your system `PATH`:
1. **Arm GNU Toolchain**: `arm-none-eabi-gcc`, `arm-none-eabi-gdb`, `arm-none-eabi-objcopy`, `arm-none-eabi-size` (Version 10.3+ recommended).
2. **GNU Make**: Build automation tool (v3.81+).
3. **OpenOCD**: Open On-Chip Debugger for ST-Link v2 / v2.1 programmers.
4. **Hardware**: STM32F401CCU6 board (Black Pill), ST-Link v2 programmer.

---

## 5. Build System Architecture

### Key Build Features:
- **Auto-Discovery (`rwildcard`)**: Any new component folder added under `lib/` (e.g., `lib/OLED/`) is automatically scanned for `.c` source files and included with `-I` flags without touching the Makefile.
- **Floating Point Hardware (`FPU`)**: Fully activated with `-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16`.
- **Dynamic Semihosting**: Switching `USE_SEMIHOSTING=1` links `--specs=rdimon.specs` and excludes `syscalls.c` automatically to avoid duplicate symbol errors.

---

## 6. Usage & Workflow Guide - Main Application Workflow (`Makefile.mak`)

### 1. Compile the project
```bash
# Clean and build the entire main project
make -f Makefile.mak clean all

# Enable FreeRTOS compilation
make -f Makefile.mak USE_FREERTOS=1 clean all

# Enable Semihosting (printf through ST-Link to console)
make -f Makefile.mak USE_SEMIHOSTING=1 clean all
```

### 2. Flash to the Microcontroller
```bash
make -f Makefile.mak flash
```

### 3. Start Debugging Session
* **Terminal 1** (Start OpenOCD Server on port 3333):
  ```bash
  make -f Makefile.mak debug_server
  ```
* **Terminal 2** (Connect GDB Client):
  ```bash
  make -f Makefile.mak debug_gdb
  ```

---

## 7. Essential GDB Commands Cheatsheet

| Command | Shorthand | Description |
| :--- | :---: | :--- |
| `continue` | `c` | Resume program execution until the next breakpoint. |
| `next` | `n` | Step over next line of code. |
| `step` | `s` | Step into function. |
| `finish` | `fin` | Finish execution of current function and break on return. |
| `break <func/line>` | `b` | Set a breakpoint (e.g., `b main`, `b mpu6050.c:120`). |
| `info breakpoints` | `i b` | List all active breakpoints. |
| `delete <num>` | `d <num>` | Delete breakpoint by index (`d` deletes all). |
| `print <var>` | `p <var>` | Print variable or struct value (e.g., `p/x reg`, `p *pHandle`). |
| `info registers` | `i r` | Display CPU core registers (R0-R12, SP, LR, PC, xPSR). |
| `monitor reset halt` | | Reset MCU and immediately halt at `Reset_Handler`. |
| `monitor reset run` | | Reset MCU and allow it to run freely. |
| `detach` | | Detach GDB and allow CPU to continue running independently. |
| `quit` | `q` | Exit the GDB console. |

---

## 8. Floating-Point & Telemetry Notes

1. **Hardware FPU Activation**:
   Before performing any floating-point computations, initialize the hardware FPU by calling `enable_FPU()` from `driver/inc/stm32f4xx_utils.h`:
   ```c
   #include "stm32f4xx_utils.h"

   int main(void) {
       enable_FPU(); // Enables Full Access to CP10 and CP11
       ...
   }
   ```
2. **Sending Float Values via USART**:
   The USART driver provides two transmission options for real-time telemetry:
   - **ASCII String Formatting**:
     ```c
     USART_Write_Float_String(&hUsart1, pitch_angle); // Outputs "12.3456\r\n"
     ```
     *(Requires `-u _printf_float` enabled in `LDFLAGS` when using `nano.specs`)*.
   - **Binary IEEE-754 High-Speed Stream**:
     ```c
     USART_Write_Float_Bin(&hUsart1, pitch_angle);    // Outputs 4 raw bytes (Little-Endian)
     ```
     *(Minimal CPU overhead; ideal for MATLAB, Python, or Vofa+ serial visualizers)*.
