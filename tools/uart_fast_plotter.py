import sys
import time
import threading
import queue
import serial
from collections import deque
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# ==================== CẤU HÌNH SERIAL UART ==================== #
# Lưu ý: Bạn cần thay đổi COM_PORT cho khớp với cổng kết nối thực tế trên máy tính
COM_PORT = "COM21"  
BAUD_RATE = 115200

# ==================== CẤU HÌNH ĐỒ THỊ VÀ TRUYỀN THÔNG ==================== #
MAX_POINTS = 500  
data_q = queue.Queue() # Hàng đợi nhận dữ liệu IMU để vẽ
tx_queue = queue.Queue() # Hàng đợi gửi lệnh (VD: PID) từ GUI xuống STM32

times = deque(maxlen=MAX_POINTS)
acc_x = deque(maxlen=MAX_POINTS)
acc_y = deque(maxlen=MAX_POINTS)
acc_z = deque(maxlen=MAX_POINTS)

gyr_x = deque(maxlen=MAX_POINTS)
gyr_y = deque(maxlen=MAX_POINTS)
gyr_z = deque(maxlen=MAX_POINTS)

angle_pitch = deque(maxlen=MAX_POINTS)
angle_roll = deque(maxlen=MAX_POINTS)

start_time = time.time()

# ==================== SERIAL LOGIC ==================== #
def serial_thread_func():
    """Chạy vòng lặp Serial đọc/ghi ngầm trong một luồng (thread) riêng"""
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.05)
        print(f"[+] KẾT NỐI THÀNH CÔNG UART ({COM_PORT} - {BAUD_RATE} bps)!\n")
    except Exception as e:
        print(f"[!] LỖI: Không thể mở cổng {COM_PORT}. Vui lòng kiểm tra cáp nối hoặc số cổng COM.")
        print(f"Chi tiết: {e}")
        return

    print_counter = 0

    while True:
        try:
            # 1. Gửi lệnh (TX) nếu có lệnh trong hàng đợi
            while not tx_queue.empty():
                msg = tx_queue.get()
                ser.write(msg.encode('utf-8'))
                print(f"\n[TX -> STM32]: {repr(msg)}")
                time.sleep(0.02) # Trễ một chút để STM32 kịp xử lý

            # 2. Nhận dữ liệu (RX)
            # Thay vì chỉ đọc 1 dòng (if ser.in_waiting), ta dùng while để đọc SẠCH buffer,
            # tránh việc dữ liệu cũ bị kẹt lại dẫn đến "lag" thời gian thực.
            while ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Phân tích cú pháp chuỗi IMU
                if line.startswith("IMU:"):
                    payload = line[4:]
                    parts = payload.split(',')
                    
                    # Cấu trúc khung truyền: "IMU:Ax,Ay,Az,Gx,Gy,Gz,Pitch,Roll" (8 thông số)
                    if len(parts) == 8:
                        try:
                            ax, ay, az, gx, gy, gz, pitch, roll = [float(x) for x in parts]
                            
                            # In ra Terminal (Throttling: Chỉ in mỗi 5 frame để tránh treo Terminal Windows)
                            print_counter += 1
                            if print_counter % 5 == 0:
                                sys.stdout.write(f"\r[IMU] Pitch: {pitch:>6.2f}° | Roll: {roll:>6.2f}° | "
                                                 f"Acc: [{ax:>5.2f}, {ay:>5.2f}, {az:>5.2f}] g | "
                                                 f"Gyr: [{gx:>7.2f}, {gy:>7.2f}, {gz:>7.2f}] °/s   ")
                                sys.stdout.flush()
                                  
                            data_q.put((time.time() - start_time, ax, ay, az, gx, gy, gz, pitch, roll))
                        except ValueError:
                            pass

                # Gói tin phản hồi ACK từ STM32
                elif line.startswith("ACK:"):
                    status = line[4:].strip()
                    print(f"\n[ACK] -> STM32 Phản hồi: {status}")

        except serial.SerialException as e:
            print(f"\n[!] Lỗi đứt kết nối Serial: {e}")
            break
        except Exception:
            pass

# ==================== PYQTGRAPH GUI VỚI MENU ĐIỀU KHIỂN ==================== #
class RealTimePlotter(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MPU6050 Real-time Telemetry & PID Tuning (UART)")
        self.resize(1100, 850)
        
        # 1. Main Layout
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QVBoxLayout()
        central_widget.setLayout(main_layout)
        
        # 2. Control Panel (Khu vực nhập PID)
        control_group = QtWidgets.QGroupBox("Điều chỉnh thông số PID")
        control_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        control_layout = QtWidgets.QHBoxLayout()
        control_group.setLayout(control_layout)
        
        control_layout.addWidget(QtWidgets.QLabel("Kp:"))
        self.input_kp = QtWidgets.QLineEdit("0.000")
        control_layout.addWidget(self.input_kp)
        
        control_layout.addWidget(QtWidgets.QLabel("Ki:"))
        self.input_ki = QtWidgets.QLineEdit("0.000")
        control_layout.addWidget(self.input_ki)
        
        control_layout.addWidget(QtWidgets.QLabel("Kd:"))
        self.input_kd = QtWidgets.QLineEdit("0.000")
        control_layout.addWidget(self.input_kd)
        
        self.btn_send = QtWidgets.QPushButton("GỬI XUỐNG STM32")
        self.btn_send.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 6px; border-radius: 3px;")
        self.btn_send.clicked.connect(self.send_pid)
        control_layout.addWidget(self.btn_send)
        
        # Khoảng trống đẩy layout gọn lại
        control_layout.addStretch()
        
        main_layout.addWidget(control_group)
        
        # 3. Graphics Layout (Khu vực vẽ đồ thị)
        self.graph_widget = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self.graph_widget)
        
        pg.setConfigOptions(antialias=True)
        
        # ---- Đồ thị Gia tốc kế ----
        self.plot_acc = self.graph_widget.addPlot(title="Accelerometer (g)")
        self.plot_acc.showGrid(x=True, y=True, alpha=0.3)
        self.plot_acc.setYRange(-2.5, 2.5)
        self.plot_acc.addLegend()
        
        self.curve_ax = self.plot_acc.plot(pen=pg.mkPen('r', width=2), name="Ax (X)")
        self.curve_ay = self.plot_acc.plot(pen=pg.mkPen('g', width=2), name="Ay (Y)")
        self.curve_az = self.plot_acc.plot(pen=pg.mkPen('b', width=2), name="Az (Z)")
        
        self.graph_widget.nextRow()
        
        # ---- Đồ thị Con quay hồi chuyển ----
        self.plot_gyr = self.graph_widget.addPlot(title="Gyroscope (°/s)")
        self.plot_gyr.showGrid(x=True, y=True, alpha=0.3)
        self.plot_gyr.setYRange(-250, 250)
        self.plot_gyr.addLegend()
        
        self.curve_gx = self.plot_gyr.plot(pen=pg.mkPen('r', width=2), name="Gx (X)")
        self.curve_gy = self.plot_gyr.plot(pen=pg.mkPen('g', width=2), name="Gy (Y)")
        self.curve_gz = self.plot_gyr.plot(pen=pg.mkPen('b', width=2), name="Gz (Z)")
        
        self.graph_widget.nextRow()
        
        # ---- Đồ thị Góc nghiêng (Angle) ----
        self.plot_ang = self.graph_widget.addPlot(title="Angle (°)")
        self.plot_ang.showGrid(x=True, y=True, alpha=0.3)
        self.plot_ang.setYRange(-90, 90)
        self.plot_ang.addLegend()
        self.plot_ang.setLabel('bottom', 'Thời gian', units='s')
        
        self.curve_pitch = self.plot_ang.plot(pen=pg.mkPen('r', width=2), name="Pitch")
        self.curve_roll = self.plot_ang.plot(pen=pg.mkPen('b', width=2), name="Roll")
        
        # Biến phục vụ Auto-Scroll
        self.time_window = 10.0 # Hiển thị 10 giây dữ liệu gần nhất
        
        # ---- Thiết lập QTimer ----
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(20)
        
    def send_pid(self):
        """Hàm xử lý sự kiện bấm nút Gửi PID"""
        try:
            kp = float(self.input_kp.text())
            ki = float(self.input_ki.text())
            kd = float(self.input_kd.text())
            
            # Khung truyền PID giống phiên bản gốc
            frame_kp = f"PID:Kp={kp:.3f}\r\n"
            frame_ki = f"PID:Ki={ki:.3f}\r\n"
            frame_kd = f"PID:Kd={kd:.3f}\r\n"
            
            tx_queue.put(frame_kp)
            tx_queue.put(frame_ki)
            tx_queue.put(frame_kd)
            
            print(f"\n[GUI] Đã đưa 3 lệnh vào hàng đợi:")
            print(f"      -> {repr(frame_kp)}")
            print(f"      -> {repr(frame_ki)}")
            print(f"      -> {repr(frame_kd)}")
            
        except ValueError:
            QtWidgets.QMessageBox.warning(self, "Lỗi định dạng", "Vui lòng nhập Kp, Ki, Kd dưới dạng số thực hợp lệ (ví dụ: 12.345)!")

    def update_plots(self):
        """Cập nhật dữ liệu đồ thị"""
        has_new_data = False
        
        while not data_q.empty():
            has_new_data = True
            t, ax, ay, az, gx, gy, gz, p, r = data_q.get()
            times.append(t)
            acc_x.append(ax)
            acc_y.append(ay)
            acc_z.append(az)
            gyr_x.append(gx)
            gyr_y.append(gy)
            gyr_z.append(gz)
            angle_pitch.append(p)
            angle_roll.append(r)
            
        if has_new_data and len(times) > 0:
            t_list = list(times)
            self.curve_ax.setData(t_list, list(acc_x))
            self.curve_ay.setData(t_list, list(acc_y))
            self.curve_az.setData(t_list, list(acc_z))
            
            self.curve_gx.setData(t_list, list(gyr_x))
            self.curve_gy.setData(t_list, list(gyr_y))
            self.curve_gz.setData(t_list, list(gyr_z))
            
            self.curve_pitch.setData(t_list, list(angle_pitch))
            self.curve_roll.setData(t_list, list(angle_roll))
            
            # Scroll X-axis
            current_time = t_list[-1]
            if current_time > self.time_window:
                self.plot_acc.setXRange(current_time - self.time_window, current_time, padding=0)
                self.plot_gyr.setXRange(current_time - self.time_window, current_time, padding=0)
                self.plot_ang.setXRange(current_time - self.time_window, current_time, padding=0)

def main():
    print("======================================================")
    print("      MPU6050 REAL-TIME TELEMETRY & PID TUNING (UART) ")
    print("======================================================")
    
    # 1. Khởi động luồng chạy Serial độc lập để không cản trở GUI
    serial_thread = threading.Thread(target=serial_thread_func, daemon=True)
    serial_thread.start()

    # 2. Khởi chạy GUI
    app = QtWidgets.QApplication(sys.argv)
    
    # Ép style cho app đẹp hơn nếu cần
    app.setStyle("Fusion")
    
    plotter = RealTimePlotter()
    plotter.show()
    
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
