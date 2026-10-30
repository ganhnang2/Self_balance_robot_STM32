# MPU-6050 REGISTERS & MATHEMATICAL FORMULAS REFERENCE

Tài liệu này tổng hợp toàn bộ các nhóm thanh ghi và công thức tính toán từ tài liệu chính thức `MPU-6000 and MPU-6050 Register Map and Descriptions` (Revision 4.2), phục vụ cho việc triển khai driver và thuật toán **Sensor Fusion** (Kalman Filter / Complementary Filter) trong dự án **Self-Balancing Robot**.

---

## MỤC LỤC
1. [Thông số Giao tiếp I2C](#1-thông-số-giao-tiếp-i2c)
2. [Bảng Nhóm Thanh Ghi & Chức Năng](#2-bảng-nhóm-thanh-ghi--chức-năng)
3. [Chi Tiết Các Thanh Ghi Quan Trọng](#3-chi-tiết-các-thanh-ghi-quan-trọng)
4. [Tổng Hợp Các Công Thức Tính Toán](#4-tổng-hợp-các-công-thức-tính-toán)
5. [Trình Tự Khởi Tạo & Đọc Dữ Liệu Thực Chiến](#5-trình-tự-khởi-tạo--đọc-dữ-liệu-thực-chiến)

---

## 1. THÔNG SỐ GIAO TIẾP I2C

Địa chỉ I2C của MPU6050 là địa chỉ 7-bit, phụ thuộc vào chân phần cứng **`AD0`**:
* **6 bit cao (Bit 6 -> Bit 1):** Cố định phần cứng là `110100` (`0x34`).
* **Bit thấp nhất (Bit 0 - LSB):** Xác định bởi mức logic chân `AD0`.

| Trạng thái chân `AD0` | Địa chỉ 7-bit (Hex) | Địa chỉ ghi (Write 8-bit) | Địa chỉ đọc (Read 8-bit) |
| :--- | :---: | :---: | :---: |
| **GND (Mặc định)** | **`0x68`** | `0xD0` | `0xD1` |
| **VCC (3.3V)** | **`0x69`** | `0xD2` | `0xD3` |

* **Tốc độ I2C tối đa:** Standard Mode (100 kHz) hoặc Fast Mode (400 kHz).
* **Kiểm tra kết nối:** Đọc thanh ghi `WHO_AM_I` (`0x75`), giá trị trả về mặc định luôn là **`0x68`**.

---

## 2. BẢNG NHÓM THANH GHI & CHỨC NĂNG

| Nhóm chức năng | Tên thanh ghi đại diện | Địa chỉ (Hex/Dec) | Chức năng chính |
| :--- | :--- | :---: | :--- |
| **1. Quản lý Nguồn & Xung nhịp** | `PWR_MGMT_1`<br>`PWR_MGMT_2` | `0x6B` (107)<br>`0x6C` (108) | • Đánh thức chip (`SLEEP = 0`).<br>• Chọn xung nhịp (`CLKSEL`): PLL trục X/Y/Z Gyro.<br>• Reset chip (`DEVICE_RESET = 1`).<br>• Bật/tắt chế độ Standby cho từng trục. |
| **2. Tần số lấy mẫu & Bộ lọc số** | `SMPLRT_DIV`<br>`CONFIG` | `0x19` (25)<br>`0x1A` (26) | • `SMPLRT_DIV`: Chia tần số lấy mẫu.<br>• `CONFIG`: Cấu hình bộ lọc thông thấp số (DLPF) lọc rung cơ học động cơ. |
| **3. Cấu hình Thang đo Cảm biến** | `GYRO_CONFIG`<br>`ACCEL_CONFIG` | `0x1B` (27)<br>`0x1C` (28) | • `GYRO_CONFIG`: Chọn dải đo Gyro ($\pm 250, \pm 500, \pm 1000, \pm 2000\,^\circ/\text{s}$).<br>• `ACCEL_CONFIG`: Chọn dải đo Accel ($\pm 2g, \pm 4g, \pm 8g, \pm 16g$). |
| **4. Dữ liệu Đo đạc (14 Bytes liên tiếp)** | `ACCEL_XOUT_H/L`<br>`ACCEL_YOUT_H/L`<br>`ACCEL_ZOUT_H/L`<br>`TEMP_OUT_H/L`<br>`GYRO_XOUT_H/L`<br>`GYRO_YOUT_H/L`<br>`GYRO_ZOUT_H/L` | `0x3B` - `0x3C`<br>`0x3D` - `0x3E`<br>`0x3F` - `0x40`<br>`0x41` - `0x42`<br>`0x43` - `0x44`<br>`0x45` - `0x46`<br>`0x47` - `0x48` | Lưu giá trị đo thô dạng 16-bit bù 2 (Signed 16-bit):<br>• Gia tốc 3 trục (X, Y, Z)<br>• Nhiệt độ nội<br>• Vận tốc góc 3 trục (X, Y, Z)<br>*(Đọc Burst một chuỗi 14 byte qua I2C DMA)*. |
| **5. Nhận diện & Kiểm tra (Self-Test)** | `WHO_AM_I`<br>`SELF_TEST_X/Y/Z/A` | `0x75` (117)<br>`0x0D` - `0x10` | • Xác thực chip (`0x68`).<br>• Tự kiểm tra cơ - điện tử MEMS của cảm biến từ nhà máy. |
| **6. Ngắt & Giao tiếp Phụ (Aux I2C)** | `INT_PIN_CFG`<br>`INT_ENABLE`<br>`INT_STATUS`<br>`I2C_MST_CTRL`<br>`I2C_SLVx_...` | `0x37` (55)<br>`0x38` (56)<br>`0x3A` (58)<br>`0x24` (36)<br>`0x25` - `0x35` | • Cấu hình chân ngắt `INT` (Bật ngắt dữ liệu sẵn sàng `DATA_RDY_EN`).<br>• Bật `I2C_BYPASS_EN` để STM32 giao tiếp trực tiếp với La bàn phụ (HMC5883L).<br>• Cấu hình MPU6050 làm I2C Master phụ. |
| **7. Bộ đệm FIFO** | `USER_CTRL`<br>`FIFO_EN`<br>`FIFO_COUNTH/L`<br>`FIFO_R_W` | `0x6A` (106)<br>`0x23` (35)<br>`0x72` - `0x73`<br>`0x74` (116) | Quản lý bộ đệm phần cứng 1024 bytes (ghi tuần tự và đọc dữ liệu hàng loạt). |

---

## 3. CHI TIẾT CÁC THANH GHI QUAN TRỌNG

### 3.1. Thanh ghi `PWR_MGMT_1` (`0x6B` / 107) - Power Management 1
| Bit | Tên Bit | Giá trị khuyên dùng | Ý nghĩa |
| :---: | :---: | :---: | :--- |
| **7** | `DEVICE_RESET` | `0` | Ghi `1` để reset toàn bộ thanh ghi về mặc định (tự xóa về 0). |
| **6** | `SLEEP` | **`0`** | **0: Đánh thức chip dậy** (mặc định sau khi bật nguồn là `1` - ngủ). |
| **5** | `CYCLE` | `0` | Chế độ chu kỳ tiết kiệm điện (chỉ dùng khi đo gia tốc tần số thấp). |
| **3** | `TEMP_DIS` | `0` | 0: Bật cảm biến nhiệt độ; 1: Tắt cảm biến nhiệt độ. |
| **2:0** | `CLKSEL[2:0]` | **`001` (`1`)** | **Chọn nguồn xung nhịp:**<br>• `0`: Xung nội 8MHz (kém ổn định).<br>• **`1`: PLL khóa pha theo trục X Gyro (Khuyên dùng).**<br>• `2`: PLL khóa pha theo trục Y Gyro.<br>• `3`: PLL khóa pha theo trục Z Gyro. |

### 3.2. Thanh ghi `CONFIG` (`0x1A` / 26) - Digital Low Pass Filter (DLPF)
Cấu hình 3 bit cuối `DLPF_CFG[2:0]` để lọc nhiễu cơ học từ động cơ:

| `DLPF_CFG` | Băng thông Accel (Hz) | Độ trễ Accel (ms) | Băng thông Gyro (Hz) | Độ trễ Gyro (ms) | Tần số Gyro nội (Fs) | Đánh giá |
| :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| `0` | 260 | 0.0 | 256 | 0.98 | 8 kHz | Không lọc |
| `1` | 184 | 2.0 | 188 | 1.9 | 1 kHz | Lọc nhẹ |
| `2` | 94 | 3.0 | 98 | 2.8 | 1 kHz | Rất tốt |
| **`3`** | **44** | **4.9** | **42** | **4.8** | **1 kHz** | ⭐ **Tối ưu nhất cho Robot cân bằng** |
| `4` | 21 | 8.5 | 20 | 8.3 | 1 kHz | Trễ bắt đầu tăng |
| `5` | 10 | 13.8 | 10 | 13.4 | 1 kHz | Quá trễ cho xe cân bằng |
| `6` | 5 | 19.0 | 5 | 18.6 | 1 kHz | Rất trễ |

### 3.3. Thanh ghi `GYRO_CONFIG` (`0x1B` / 27) & `ACCEL_CONFIG` (`0x1C` / 28)
* **`GYRO_CONFIG` (Bits [4:3] - `FS_SEL[1:0]`):**
  * `0`: $\pm 250\,^\circ/\text{s}$ $\rightarrow$ Độ nhạy: **`131.0 LSB/(°/s)`** (Tối ưu nhất cho xe cân bằng).
  * `1`: $\pm 500\,^\circ/\text{s}$ $\rightarrow$ Độ nhạy: `65.5 LSB/(°/s)`.
  * `2`: $\pm 1000\,^\circ/\text{s}$ $\rightarrow$ Độ nhạy: `32.8 LSB/(°/s)`.
  * `3`: $\pm 2000\,^\circ/\text{s}$ $\rightarrow$ Độ nhạy: `16.4 LSB/(°/s)`.
* **`ACCEL_CONFIG` (Bits [4:3] - `AFS_SEL[1:0]`):**
  * `0`: $\pm 2g$ $\rightarrow$ Độ nhạy: **`16384 LSB/g`** (Tối ưu nhất cho xe cân bằng).
  * `1`: $\pm 4g$ $\rightarrow$ Độ nhạy: `8192 LSB/g`.
  * `2`: $\pm 8g$ $\rightarrow$ Độ nhạy: `4096 LSB/g`.
  * `3`: $\pm 16g$ $\rightarrow$ Độ nhạy: `2048 LSB/g`.

---

## 4. TỔNG HỢP CÁC CÔNG THỨC TÍNH TOÁN

### 4.1. Công thức tính Tần số lấy mẫu (Sample Rate)
*(Trang 12 — Mục 4.2 Register 25 `SMPLRT_DIV`)*

$$\text{Sample Rate} = \frac{\text{Gyroscope Output Rate}}{1 + \text{SMPLRT\_DIV}}$$

* Khi DLPF được bật (`DLPF_CFG = 1 -> 6`), $\text{Gyroscope Output Rate} = 1000\,\text{Hz}$.
* **Công thức tính giá trị cần nạp vào thanh ghi:**
  $$\text{SMPLRT\_DIV} = \frac{1000}{\text{Desired Sample Rate (Hz)}} - 1$$
* **Trường hợp sử dụng:**
  * Đồng bộ tần số ngắt của cảm biến với chu kỳ thuật toán PID ($100\text{Hz} - 200\text{Hz}$).
  * *Ví dụ:* Muốn tần số lấy mẫu $200\,\text{Hz}$ ($5\,\text{ms}$ một lần):
    $$\text{SMPLRT\_DIV} = \frac{1000}{200} - 1 = \mathbf{4}$$
    *(Ghi giá trị `4` vào thanh ghi `0x19`)*.

---

### 4.2. Công thức quy đổi giá trị Thô sang Đơn vị Vật lý
*(Trang 29 Mục 4.17 & Trang 31 Mục 4.19)*

Dữ liệu đọc về là số nguyên 16-bit có dấu bù 2 (từ $-32768$ đến $+32767$):

$$\text{Gia tốc } (g) = \frac{\text{ACCEL\_RAW}}{\text{LSB Sensitivity}}$$

$$\text{Vận tốc góc } (^\circ/\text{s}) = \frac{\text{GYRO\_RAW}}{\text{LSB Sensitivity}}$$

* **Trường hợp sử dụng:** Áp dụng liên tục trong vòng lặp chính ngay sau khi đọc 14 byte dữ liệu thô:
  ```c
  float ax = (float)raw_accel_x / 16384.0f; // Đơn vị: g
  float ay = (float)raw_accel_y / 16384.0f;
  float az = (float)raw_accel_z / 16384.0f;

  float gx = (float)raw_gyro_x / 131.0f;   // Đơn vị: độ/giây
  float gy = (float)raw_gyro_y / 131.0f;
  float gz = (float)raw_gyro_z / 131.0f;
  ```

---

### 4.3. Công thức chuyển đổi Nhiệt độ ra độ C ($^\circ\text{C}$)
*(Trang 30 — Mục 4.18 Registers 65 & 66 `TEMP_OUT`)*

$$\text{Temperature } (^\circ\text{C}) = \frac{\text{TEMP\_OUT (signed 16-bit)}}{340.0} + 36.53$$

* **Trường hợp sử dụng:**
  1. Giám sát cảnh báo quá nhiệt cho cảm biến.
  2. **Bù trôi nhiệt cho con quay hồi chuyển (Gyro Temperature Drift Compensation):** Điểm 0 của Gyro trôi theo nhiệt độ, dùng giá trị này để hiệu chỉnh sai số trôi khi chip nóng lên.

---

### 4.4. Công thức tính Góc nghiêng từ Gia tốc (Dung hợp Kalman / Complementary Filter)
Dùng gia tốc trọng trường $g$ làm mỏ neo không trôi:

$$\text{Pitch}_{\text{accel}} = \text{atan2f}(\text{Accel}_Y, \text{Accel}_Z) \times \frac{180.0}{\pi}$$

$$\text{Roll}_{\text{accel}} = \text{atan2f}(\text{Accel}_X, \text{Accel}_Z) \times \frac{180.0}{\pi}$$

* **Bộ lọc bù (Complementary Filter):**
  $$\theta_{k} = \alpha \times (\theta_{k-1} + \text{Gyro}_X \times \Delta t) + (1 - \alpha) \times \text{Pitch}_{\text{accel}}$$
  *(Thường chọn $\alpha = 0.98$ với chu kỳ lấy mẫu $\Delta t = 5\text{ms}$)*.

---

### 4.5. Công thức Self-Test (Kiểm tra phần cứng nhà máy)
*(Trang 9 - 11 — Mục 4.1 Registers 13 - 16)*

* **Phản ứng đo kiểm (Self-Test Response - STR):**
  $$\text{STR} = \text{Output}_{\text{Self-Test Enabled}} - \text{Output}_{\text{Self-Test Disabled}}$$
* **Giá trị chuẩn nhà máy (Factory Trim - FT):**
  * Gyro: $\text{FT}[Xg] = 25 \times 131 \times 1.046^{(\text{XG\_TEST} - 1)}$
  * Accel: $\text{FT}[Xa] = 4096 \times 0.34 \times \left(\frac{0.92}{0.34}\right)^{\frac{\text{XA\_TEST} - 1}{2^5 - 2}}$
* **Độ lệch phần trăm:**
  $$\text{Change}(\%) = \frac{\text{STR} - \text{FT}}{\text{FT}} \times 100\%$$
* **Trường hợp sử dụng:** Khâu kiểm tra xuất xưởng (QA/QC) hoặc tự chẩn đoán lỗi phần cứng (BIST - Built-In Self Test) để phát hiện chip lỗi/hỏng phiến vi cơ MEMS.

---

### 4.6. Công thức giảm tần số truy cập cảm biến phụ (Auxiliary Slave)
*(Trang 23 & 36 — Mục 4.12 & 4.25)*

$$\text{Reduced Access Rate} = \frac{\text{Sample Rate}}{1 + \text{I2C\_MST\_DLY}}$$

* **Trường hợp sử dụng:** Khi MPU6050 làm Master phụ đọc thêm La bàn số HMC5883L với tốc độ thấp hơn tốc độ lấy mẫu của MPU6050.

---

## 5. TRÌNH TỰ KHỞI TẠO & ĐỌC DỮ LIỆU THỰC CHIẾN

```text
[BƯỚC 1: KHỞI TẠO CẢM BIẾN]
  │
  ├── 1. Đọc thanh ghi WHO_AM_I (0x75) -> Phải nhận về 0x68 (Nếu sai: Báo lỗi kết nối I2C).
  ├── 2. Ghi PWR_MGMT_1 (0x6B) = 0x80  -> Reset toàn bộ thanh ghi chip (Chờ 100ms).
  ├── 3. Ghi PWR_MGMT_1 (0x6B) = 0x01  -> Đánh thức chip (SLEEP=0) và chọn Clock PLL trục X Gyro.
  ├── 4. Ghi CONFIG (0x1A)     = 0x03  -> Bật DLPF ~42Hz (lọc nhiễu rung cơ học động cơ).
  ├── 5. Ghi SMPLRT_DIV (0x19) = 0x04  -> Cài đặt Sample Rate = 1000 / (1 + 4) = 200 Hz.
  ├── 6. Ghi GYRO_CONFIG (0x1B)= 0x00  -> Thang đo Gyro ±250 °/s (Độ nhạy: 131 LSB/°/s).
  └── 7. Ghi ACCEL_CONFIG(0x1C)= 0x00  -> Thang đo Accel ±2g (Độ nhạy: 16384 LSB/g).

[BƯỚC 2: CALIBRATION (HIỆU CHUẨN ĐIỂM 0 TĨNH)]
  │
  └── Giữ yên robot trong 2 giây, lấy trung bình 500 mẫu để tính sai số tĩnh (Gyro Offset):
      Gyro_Offset_X = Tổng_Raw_X / 500;

[BƯỚC 3: VÒNG LẶP ĐIỀU KHIỂN CHÍNH (CHU KỲ 5ms - 200Hz)]
  │
  ├── 1. Gửi lệnh đọc Burst 14 bytes liên tiếp bắt đầu từ địa chỉ 0x3B (ACCEL_XOUT_H).
  ├── 2. Ghép các cặp byte High/Low thành số nguyên 16-bit có dấu bù 2:
  │      raw_accel_x = (buffer[0] << 8) | buffer[1];
  │      raw_accel_y = (buffer[2] << 8) | buffer[3];
  │      raw_accel_z = (buffer[4] << 8) | buffer[5];
  │      raw_temp    = (buffer[6] << 8) | buffer[7];
  │      raw_gyro_x  = (buffer[8] << 8) | buffer[9];
  │      raw_gyro_y  = (buffer[10] << 8) | buffer[11];
  │      raw_gyro_z  = (buffer[12] << 8) | buffer[13];
  │
  ├── 3. Áp dụng hệ số nhạy (LSB Sensitivity) và trừ Offset:
  │      ax = raw_accel_x / 16384.0f;
  │      ay = raw_accel_y / 16384.0f;
  │      az = raw_accel_z / 16384.0f;
  │      gx = (raw_gyro_x - Gyro_Offset_X) / 131.0f;
  │
  ├── 4. Tính góc nghiêng gia tốc mỏ neo:
  │      pitch_accel = atan2f(ay, az) * (180.0f / 3.14159265f);
  │
  └── 5. Đưa pitch_accel và gx vào thuật toán Kalman Filter / Complementary Filter
         để xuất ra góc nghiêng thực tế chuẩn xác cho vòng lặp điều khiển PID!
```
