import sys
import time
import asyncio
import threading
import queue
from collections import deque
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from bleak import BleakScanner, BleakClient

# ==================== CẤU HÌNH PHẦN CỨNG HC-05 ==================== #
TARGET_MAC  = "19:36:B9:1A:5B:79"
TARGET_NAME = "HC-05"
SCAN_TIMEOUT = 6.0

# ==================== CẤU HÌNH ĐỒ THỊ VÀ TRUYỀN THÔNG ==================== #
MAX_POINTS = 500  
data_q = queue.Queue() # Hàng đợi nhận dữ liệu IMU để vẽ
tx_queue = queue.Queue() # Hàng đợi gửi lệnh (VD: PID) từ GUI xuống STM32

times = deque(maxlen=MAX_POINTS)
target_data = deque(maxlen=MAX_POINTS)
pitch_data = deque(maxlen=MAX_POINTS)
acc_x = deque(maxlen=MAX_POINTS)
acc_y = deque(maxlen=MAX_POINTS)
acc_z = deque(maxlen=MAX_POINTS)

gyr_x = deque(maxlen=MAX_POINTS)
gyr_y = deque(maxlen=MAX_POINTS)
gyr_z = deque(maxlen=MAX_POINTS)

start_time = None
rx_stream_buffer = ""

# ==================== BLUETOOTH LOGIC ==================== #
def notification_handler(sender, data: bytearray):
    """Callback nhận dữ liệu từ Characteristic Notify của BLE"""
    global rx_stream_buffer, start_time
    try:
        chunk = data.decode('utf-8', errors='ignore')
    except Exception:
        return

    if start_time is None:
        start_time = time.time()

    rx_stream_buffer += chunk

    while '\n' in rx_stream_buffer:
        line, rx_stream_buffer = rx_stream_buffer.split('\n', 1)
        line = line.strip()
        
        # Phân tích cú pháp chuỗi IMU
        if line.startswith("IMU:"):
            payload = line[4:]
            parts = payload.split(',')
            
            # Firmware STM32 gửi 8 thông số: targetAngle, pitch, ax, ay, az, gx, gy, gz
            if len(parts) == 8:
                try:
                    target_angle, pitch, ax, ay, az, gx, gy, gz = [float(x) for x in parts]
                    
                    # In dữ liệu ra Terminal 
                    err = pitch - target_angle
                    sys.stdout.write(f"\r[IMU] Target: {target_angle:>5.2f}° | Pitch: {pitch:>5.2f}° | Err: {err:>5.2f}° | "
                                     f"Acc: [{ax:>5.2f}, {ay:>5.2f}, {az:>5.2f}] g | "
                                     f"Gyr: [{gx:>6.1f}, {gy:>6.1f}, {gz:>6.1f}] °/s   ")
                    sys.stdout.flush()
                          
                    data_q.put((time.time() - start_time, target_angle, pitch, ax, ay, az, gx, gy, gz))
                except ValueError:
                    pass

        # Gói tin phản hồi ACK từ STM32
        elif line.startswith("ACK:"):
            status = line[4:].strip()
            print(f"\n[ACK] -> STM32 Phản hồi: {status}")

async def tx_worker(client, write_char):
    """Vòng lặp chạy ngầm trong BLE thread để lấy lệnh từ tx_queue và gửi đi"""
    while client.is_connected:
        if not tx_queue.empty():
            msg = tx_queue.get()
            payload = msg.encode('utf-8')
            chunk_size = 20  # Gửi chia nhỏ 20 bytes mỗi lần để không quá tải BLE
            try:
                for i in range(0, len(payload), chunk_size):
                    chunk = payload[i:i + chunk_size]
                    if "write-without-response" in write_char.properties:
                        await client.write_gatt_char(write_char, chunk, response=False)
                    else:
                        await client.write_gatt_char(write_char, chunk, response=True)
                    await asyncio.sleep(0.015)
                print(f"\n[TX -> STM32]: {repr(msg)}")
            except Exception as e:
                print(f"\n[TX LỖI]: {e}")
        await asyncio.sleep(0.05)

async def find_target_ble_device():
    print(f"[*] Đang tìm kiếm module BLE HC-05 (MAC: {TARGET_MAC})...")
    device = await BleakScanner.find_device_by_address(TARGET_MAC, timeout=3.0)
    if device:
        print(f"[+] ĐÃ TÌM THẤY HC-05 THEO ĐỊA CHỈ MAC: {device.name or 'HC-05'} [{device.address}]")
        return device

    print(f"[*] Đang quét sóng BLE xung quanh...")
    devices = await BleakScanner.discover(timeout=SCAN_TIMEOUT)
    for d in devices:
        if d.address.upper() == TARGET_MAC.upper():
            return d
    return None

async def find_uart_characteristics(client: BleakClient):
    """Tìm UUID cho luồng nhận (Notify) và luồng gửi (Write)"""
    notify_char = None
    write_char = None
    COMMON_UUIDS = [
        "0000ffe1-0000-1000-8000-00805f9b34fb",
        "0000ffe2-0000-1000-8000-00805f9b34fb",
    ]
    
    for service in client.services:
        for char in service.characteristics:
            uuid_str = str(char.uuid).lower()
            if uuid_str in COMMON_UUIDS:
                if "notify" in char.properties or "indicate" in char.properties:
                    notify_char = char
                if "write" in char.properties or "write-without-response" in char.properties:
                    write_char = char
                    
    # Fallback nếu không có UUID chuẩn
    if not notify_char or not write_char:
        for service in client.services:
            for char in service.characteristics:
                if not notify_char and ("notify" in char.properties or "indicate" in char.properties):
                    notify_char = char
                if not write_char and ("write" in char.properties or "write-without-response" in char.properties):
                    write_char = char
                    
    return notify_char, write_char

async def ble_main():
    device = await find_target_ble_device()
    if not device:
        print("[!] Không có thiết bị. Thoát BLE thread.")
        return

    print(f"\n[*] Đang thiết lập kết nối BLE tới HC-05 [{device.address}]...")
    async with BleakClient(device.address) as client:
        if not client.is_connected:
            print("[!] Không thể kết nối tới HC-05!")
            return

        print(f"[+] KẾT NỐI BLE THÀNH CÔNG VỚI HC-05 ({device.name or 'HC-05'})!\n")

        notify_char, write_char = await find_uart_characteristics(client)
        if not notify_char or not write_char:
            print("\n[!] LỖI: Không tìm thấy kênh UART Notify/Write phù hợp trên thiết bị.")
            return

        await client.start_notify(notify_char.uuid, notification_handler)
        print("[+] Bắt đầu nhận dữ liệu IMU để vẽ đồ thị và sẵn sàng gửi PID...")
        
        # Tạo tiến trình ngầm để chạy tx_worker (chuyên gửi dữ liệu đi)
        tx_task = asyncio.create_task(tx_worker(client, write_char))
        
        try:
            while client.is_connected:
                await asyncio.sleep(1.0)
        except asyncio.CancelledError:
            pass
        finally:
            tx_task.cancel()
            await client.stop_notify(notify_char.uuid)
            print("[*] Đã đóng kết nối BLE an toàn.")

def ble_thread_func():
    """Chạy vòng lặp Asyncio cho Bleak trong một luồng (thread) riêng"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(ble_main())
    except Exception as e:
        print(f"\nBLE Thread Error: {e}")

# ==================== PYQTGRAPH GUI VỚI MENU ĐIỀU KHIỂN ==================== #
class RealTimePlotter(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MPU6050 Real-time Telemetry & PID Tuning (PyQtGraph)")
        self.resize(1150, 920)
        
        # 1. Main Layout
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QVBoxLayout()
        central_widget.setLayout(main_layout)
        
        # 2. Control Panel (Bảng điều khiển 3 vòng PID độc lập)
        control_group = QtWidgets.QGroupBox("Bảng điều khiển thông số 3 vòng PID & Quan sát đáp ứng")
        control_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        grid_layout = QtWidgets.QGridLayout()
        grid_layout.setHorizontalSpacing(10)
        grid_layout.setVerticalSpacing(8)
        control_group.setLayout(grid_layout)

        input_style = "font-size: 13px; font-weight: bold; padding: 4px; border: 1px solid #555; border-radius: 3px;"
        label_param_style = "font-size: 13px; font-weight: bold;"

        # --- HÀNG 1: [1] PID CÂN BẰNG (BALANCE) ---
        lbl_bal = QtWidgets.QLabel("[1] PID CÂN BẰNG  :")
        lbl_bal.setStyleSheet("font-size: 13px; font-weight: bold; color: #FFD700;")
        grid_layout.addWidget(lbl_bal, 0, 0)

        grid_layout.addWidget(QtWidgets.QLabel("Kp:"), 0, 1)
        self.input_bal_kp = QtWidgets.QLineEdit("0.000")
        self.input_bal_kp.setFixedWidth(75)
        self.input_bal_kp.setStyleSheet(input_style)
        grid_layout.addWidget(self.input_bal_kp, 0, 2)

        grid_layout.addWidget(QtWidgets.QLabel("Ki:"), 0, 3)
        self.input_bal_ki = QtWidgets.QLineEdit("0.000")
        self.input_bal_ki.setFixedWidth(75)
        self.input_bal_ki.setStyleSheet(input_style)
        grid_layout.addWidget(self.input_bal_ki, 0, 4)

        grid_layout.addWidget(QtWidgets.QLabel("Kd:"), 0, 5)
        self.input_bal_kd = QtWidgets.QLineEdit("0.000")
        self.input_bal_kd.setFixedWidth(75)
        self.input_bal_kd.setStyleSheet(input_style)
        grid_layout.addWidget(self.input_bal_kd, 0, 6)

        self.btn_send_bal = QtWidgets.QPushButton("GỬI BALANCE")
        self.btn_send_bal.setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold; padding: 5px 14px; border-radius: 3px;")
        self.btn_send_bal.clicked.connect(lambda: self.send_pid_loop("PID CÂN BẰNG", "BAL", self.input_bal_kp, self.input_bal_ki, self.input_bal_kd))
        grid_layout.addWidget(self.btn_send_bal, 0, 7)

        # --- HÀNG 2: [2] PID VẬN TỐC (VELOCITY) ---
        lbl_vel = QtWidgets.QLabel("[2] PID VẬN TỐC   :")
        lbl_vel.setStyleSheet("font-size: 13px; font-weight: bold; color: #64B5F6;")
        grid_layout.addWidget(lbl_vel, 1, 0)

        grid_layout.addWidget(QtWidgets.QLabel("Kp:"), 1, 1)
        self.input_vel_kp = QtWidgets.QLineEdit("0.000")
        self.input_vel_kp.setFixedWidth(75)
        self.input_vel_kp.setStyleSheet(input_style)
        grid_layout.addWidget(self.input_vel_kp, 1, 2)

        grid_layout.addWidget(QtWidgets.QLabel("Ki:"), 1, 3)
        self.input_vel_ki = QtWidgets.QLineEdit("0.000")
        self.input_vel_ki.setFixedWidth(75)
        self.input_vel_ki.setStyleSheet(input_style)
        grid_layout.addWidget(self.input_vel_ki, 1, 4)

        grid_layout.addWidget(QtWidgets.QLabel("Kd:"), 1, 5)
        self.input_vel_kd = QtWidgets.QLineEdit("0.000")
        self.input_vel_kd.setFixedWidth(75)
        self.input_vel_kd.setStyleSheet(input_style)
        grid_layout.addWidget(self.input_vel_kd, 1, 6)

        self.btn_send_vel = QtWidgets.QPushButton("GỬI VẬN TỐC")
        self.btn_send_vel.setStyleSheet("background-color: #1565C0; color: white; font-weight: bold; padding: 5px 14px; border-radius: 3px;")
        self.btn_send_vel.clicked.connect(lambda: self.send_pid_loop("PID VẬN TỐC", "VEL", self.input_vel_kp, self.input_vel_ki, self.input_vel_kd))
        grid_layout.addWidget(self.btn_send_vel, 1, 7)

        # --- HÀNG 3: [3] PID HƯỚNG RẼ (STEERING) ---
        lbl_str = QtWidgets.QLabel("[3] PID HƯỚNG RẼ  :")
        lbl_str.setStyleSheet("font-size: 13px; font-weight: bold; color: #FFB74D;")
        grid_layout.addWidget(lbl_str, 2, 0)

        grid_layout.addWidget(QtWidgets.QLabel("Kp:"), 2, 1)
        self.input_str_kp = QtWidgets.QLineEdit("0.000")
        self.input_str_kp.setFixedWidth(75)
        self.input_str_kp.setStyleSheet(input_style)
        grid_layout.addWidget(self.input_str_kp, 2, 2)

        grid_layout.addWidget(QtWidgets.QLabel("Ki:"), 2, 3)
        self.input_str_ki = QtWidgets.QLineEdit("0.000")
        self.input_str_ki.setFixedWidth(75)
        self.input_str_ki.setStyleSheet(input_style)
        grid_layout.addWidget(self.input_str_ki, 2, 4)

        grid_layout.addWidget(QtWidgets.QLabel("Kd:"), 2, 5)
        self.input_str_kd = QtWidgets.QLineEdit("0.000")
        self.input_str_kd.setFixedWidth(75)
        self.input_str_kd.setStyleSheet(input_style)
        grid_layout.addWidget(self.input_str_kd, 2, 6)

        self.btn_send_str = QtWidgets.QPushButton("GỬI HƯỚNG RẼ")
        self.btn_send_str.setStyleSheet("background-color: #E65100; color: white; font-weight: bold; padding: 5px 14px; border-radius: 3px;")
        self.btn_send_str.clicked.connect(lambda: self.send_pid_loop("PID HƯỚNG RẼ", "STR", self.input_str_kp, self.input_str_ki, self.input_str_kd))
        grid_layout.addWidget(self.btn_send_str, 2, 7)

        # Hỗ trợ bấm phím Enter trong ô nhập để gửi ngay
        self.input_bal_kp.returnPressed.connect(self.btn_send_bal.click)
        self.input_bal_ki.returnPressed.connect(self.btn_send_bal.click)
        self.input_bal_kd.returnPressed.connect(self.btn_send_bal.click)
        self.input_vel_kp.returnPressed.connect(self.btn_send_vel.click)
        self.input_vel_ki.returnPressed.connect(self.btn_send_vel.click)
        self.input_vel_kd.returnPressed.connect(self.btn_send_vel.click)
        self.input_str_kp.returnPressed.connect(self.btn_send_str.click)
        self.input_str_ki.returnPressed.connect(self.btn_send_str.click)
        self.input_str_kd.returnPressed.connect(self.btn_send_str.click)

        # Cột trạng thái hiển thị trực tiếp góc Pitch & Target thời gian thực ở bên phải
        self.lbl_pitch_live = QtWidgets.QLabel("Target: 0.00°\nPitch:  0.00°\nErr:    0.00°")
        align_center = QtCore.Qt.AlignCenter if hasattr(QtCore.Qt, 'AlignCenter') else QtCore.Qt.AlignmentFlag.AlignCenter
        self.lbl_pitch_live.setAlignment(align_center)
        self.lbl_pitch_live.setStyleSheet("font-family: Consolas, monospace; font-size: 13px; font-weight: bold; color: #FFD700; background-color: #212121; padding: 6px 12px; border: 1px solid #444; border-radius: 5px; min-width: 125px;")
        grid_layout.addWidget(self.lbl_pitch_live, 0, 8, 3, 1)

        main_layout.addWidget(control_group)
        
        # 3. Graphics Layout (Khu vực vẽ đồ thị)
        self.graph_widget = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self.graph_widget)
        
        pg.setConfigOptions(antialias=True)
        
        dash_style = QtCore.Qt.DashLine if hasattr(QtCore.Qt, 'DashLine') else QtCore.Qt.PenStyle.DashLine

        # ---- Đồ thị 1: Góc nghiêng Pitch vs Target Angle (Quan sát đáp ứng bám Setpoint) ----
        self.plot_pitch = self.graph_widget.addPlot(title="<b>Pitch vs Target Angle (°) - Quan sát đáp ứng bám Setpoint</b>")
        self.plot_pitch.showGrid(x=True, y=True, alpha=0.3)
        self.plot_pitch.setYRange(-30, 30)
        self.plot_pitch.addLegend(offset=(10, 10))
        
        # Đường tham chiếu 0 độ (Mốc thăng bằng tuyệt đối)
        ref_pen = pg.mkPen(color='#555555', width=1.0, style=dash_style)
        self.plot_pitch.addItem(pg.InfiniteLine(pos=0, angle=0, pen=ref_pen))
        
        # Đường Setpoint mong muốn (targetAngle) - đường nét đứt nằm ngang màu đỏ cam
        setpoint_pen = pg.mkPen(color='#FF5252', width=2.0, style=dash_style)
        self.line_setpoint = pg.InfiniteLine(pos=0.0, angle=0, pen=setpoint_pen)
        self.plot_pitch.addItem(self.line_setpoint)

        # Đường góc Pitch thực tế (Actual Pitch) nét liền màu vàng sáng
        self.curve_pitch  = self.plot_pitch.plot(pen=pg.mkPen(color='#FFD700', width=2.5), name="Pitch Actual (°)")
        
        self.graph_widget.nextRow()

        # ---- Đồ thị 2: Con quay hồi chuyển (Gyroscope) ----
        self.plot_gyr = self.graph_widget.addPlot(title="<b>Gyroscope (°/s)</b>")
        self.plot_gyr.showGrid(x=True, y=True, alpha=0.3)
        self.plot_gyr.setYRange(-250, 250)
        self.plot_gyr.addLegend(offset=(10, 10))
        
        self.curve_gx = self.plot_gyr.plot(pen=pg.mkPen('r', width=1.8), name="Gx (Roll Rate)")
        self.curve_gy = self.plot_gyr.plot(pen=pg.mkPen('g', width=1.8), name="Gy (Pitch Rate)")
        self.curve_gz = self.plot_gyr.plot(pen=pg.mkPen('b', width=1.8), name="Gz (Yaw Rate)")
        
        self.graph_widget.nextRow()

        # ---- Đồ thị 3: Gia tốc kế (Accelerometer) ----
        self.plot_acc = self.graph_widget.addPlot(title="<b>Accelerometer (g)</b>")
        self.plot_acc.showGrid(x=True, y=True, alpha=0.3)
        self.plot_acc.setYRange(-2.5, 2.5)
        self.plot_acc.addLegend(offset=(10, 10))
        self.plot_acc.setLabel('bottom', 'Thời gian', units='s')
        
        self.curve_ax = self.plot_acc.plot(pen=pg.mkPen('r', width=1.8), name="Ax (X)")
        self.curve_ay = self.plot_acc.plot(pen=pg.mkPen('g', width=1.8), name="Ay (Y)")
        self.curve_az = self.plot_acc.plot(pen=pg.mkPen('b', width=1.8), name="Az (Z)")
        
        # Liên kết trục thời gian X của cả 3 đồ thị để cuộn đồng bộ
        self.plot_gyr.setXLink(self.plot_pitch)
        self.plot_acc.setXLink(self.plot_pitch)

        # ---- Thiết lập QTimer ----
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(20)
        
    def send_pid_loop(self, loop_name, prefix, input_kp, input_ki, input_kd):
        """Hàm gửi thông số Kp, Ki, Kd cho từng vòng PID riêng biệt"""
        try:
            kp = float(input_kp.text())
            ki = float(input_ki.text())
            kd = float(input_kd.text())

            # Tách thành 3 Frame nhỏ gọn để không vượt quá 20 bytes của BLE
            frame_kp = f"{prefix}:Kp={kp:.3f}\r\n"
            frame_ki = f"{prefix}:Ki={ki:.3f}\r\n"
            frame_kd = f"{prefix}:Kd={kd:.3f}\r\n"

            # Đẩy lần lượt 3 frame vào hàng đợi (mỗi frame sẽ được gửi cách nhau 50ms nhờ tx_worker)
            tx_queue.put(frame_kp)
            tx_queue.put(frame_ki)
            tx_queue.put(frame_kd)

            print(f"\n[GUI] Đã đưa 3 lệnh vào hàng đợi cho {loop_name} ({prefix}):")
            print(f"      -> {repr(frame_kp)}")
            print(f"      -> {repr(frame_ki)}")
            print(f"      -> {repr(frame_kd)}")

        except ValueError:
            QtWidgets.QMessageBox.warning(self, "Lỗi định dạng", f"Vui lòng nhập Kp, Ki, Kd dưới dạng số thực hợp lệ cho {loop_name} (ví dụ: 12.345)!")

    def update_plots(self):
        """Cập nhật dữ liệu đồ thị"""
        has_new_data = False
        
        while not data_q.empty():
            has_new_data = True
            t, target_angle, pitch, ax, ay, az, gx, gy, gz = data_q.get()
            times.append(t)
            target_data.append(target_angle)
            pitch_data.append(pitch)
            acc_x.append(ax)
            acc_y.append(ay)
            acc_z.append(az)
            gyr_x.append(gx)
            gyr_y.append(gy)
            gyr_z.append(gz)
            
        if has_new_data and len(times) > 0:
            t_list = list(times)
            # 1. Cập nhật đồ thị góc nghiêng Pitch thực tế
            self.curve_pitch.setData(t_list, list(pitch_data))

            # Cập nhật vị trí đường nét đứt nằm ngang Setpoint (targetAngle)
            latest_pitch = pitch_data[-1]
            latest_target = target_data[-1]
            self.line_setpoint.setPos(latest_target)

            # 2. Cập nhật đồ thị con quay hồi chuyển Gyroscope
            self.curve_gx.setData(t_list, list(gyr_x))
            self.curve_gy.setData(t_list, list(gyr_y))
            self.curve_gz.setData(t_list, list(gyr_z))

            # 3. Cập nhật đồ thị gia tốc kế Accelerometer
            self.curve_ax.setData(t_list, list(acc_x))
            self.curve_ay.setData(t_list, list(acc_y))
            self.curve_az.setData(t_list, list(acc_z))

            # 4. Tự động cuộn trục thời gian X theo thời gian thực (Cửa sổ 8 giây gần nhất)
            x_max = t_list[-1]
            x_window = 8.0
            if x_max > x_window:
                self.plot_pitch.setXRange(x_max - x_window, x_max, padding=0.01)
            else:
                self.plot_pitch.setXRange(0.0, x_window, padding=0.01)

            # Cập nhật giá trị hiển thị trực tiếp lên nhãn (Target, Pitch, Error)
            err = latest_pitch - latest_target
            self.lbl_pitch_live.setText(f"Target: {latest_target:>5.2f}°\nPitch:  {latest_pitch:>5.2f}°\nErr:    {err:>5.2f}°")

def main():
    print("======================================================")
    print("      MPU6050 REAL-TIME TELEMETRY & PID TUNING")
    print("======================================================")
    
    # 1. Khởi động luồng chạy BLE độc lập để không cản trở GUI
    ble_thread = threading.Thread(target=ble_thread_func, daemon=True)
    ble_thread.start()

    # 2. Khởi chạy GUI
    app = QtWidgets.QApplication(sys.argv)
    
    # Ép style cho app đẹp hơn nếu cần
    app.setStyle("Fusion")
    
    plotter = RealTimePlotter()
    plotter.show()
    
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
