#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
  ROBOT DIRECTION CONTROLLER - GIAO DIỆN ĐIỀU KHIỂN ROBOT CÂN BẰNG
=============================================================================
  Chức năng:
    - Sử dụng chuẩn thư viện GUI pyqtgraph.Qt (giống hc05_fast_plotter.py).
    - Kết nối không dây qua Bluetooth (BLE) tới module HC-05 (STM32F4).
    - Điều khiển robot di chuyển theo các hướng:
        + FORWARD  : "DIR:FORWARD\r\n"
        + REVERSED : "DIR:REVERSED\r\n"
        + LEFT     : "DIR:LEFT\r\n"
        + RIGHT    : "DIR:RIGHT\r\n"
        + NONE     : "DIR:NONE\r\n" (Dừng lại)
    - Hỗ trợ cả 2 phương thức điều khiển:
        1. Bấm chuột trực tiếp trên giao diện (D-Pad nút bấm).
        2. Phím tắt bàn phím: W, A, S, D, Space hoặc các phím mũi tên.
    - Chế độ an toàn "Deadman switch": Nhấn giữ phím để chạy, thả phím tự dừng.
    - Hiển thị phản hồi ACK từ STM32 thời gian thực.
=============================================================================
"""

import sys
import time
import asyncio
import threading
import queue
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
from bleak import BleakScanner, BleakClient

# ==================== ĐỒNG BỘ CẤU HÌNH QT ==================== #
# Căn lề tương thích cả Qt5 và Qt6 qua pyqtgraph.Qt
if hasattr(QtCore.Qt, 'AlignmentFlag'):
    ALIGN_CENTER = QtCore.Qt.AlignmentFlag.AlignCenter
else:
    ALIGN_CENTER = QtCore.Qt.AlignCenter

# Lấy mã phím bấm (Key codes) tương thích mọi phiên bản Qt
def get_key_code(name):
    if hasattr(QtCore.Qt, 'Key') and hasattr(QtCore.Qt.Key, name):
        return getattr(QtCore.Qt.Key, name)
    if hasattr(QtCore.Qt, name):
        return getattr(QtCore.Qt, name)
    return None

KEY_W     = get_key_code('Key_W')
KEY_S     = get_key_code('Key_S')
KEY_A     = get_key_code('Key_A')
KEY_D     = get_key_code('Key_D')
KEY_UP    = get_key_code('Key_Up')
KEY_DOWN  = get_key_code('Key_Down')
KEY_LEFT  = get_key_code('Key_Left')
KEY_RIGHT = get_key_code('Key_Right')
KEY_SPACE = get_key_code('Key_Space')

# ==================== CẤU HÌNH PHẦN CỨNG HC-05 ==================== #
TARGET_MAC   = "19:36:B9:1A:5B:79"   # MAC mặc định của HC-05 (giống hc05_fast_plotter.py)
TARGET_NAME  = "HC-05"
SCAN_TIMEOUT = 6.0

# Hàng đợi lệnh gửi đi từ GUI sang BLE thread
tx_queue = queue.Queue()

# Hàng đợi log / event gửi từ BLE thread về GUI
rx_queue = queue.Queue()


# ==================== BLUETOOTH BLE LOGIC ==================== #
async def find_target_ble_device(target_mac):
    """Tìm thiết bị BLE theo địa chỉ MAC hoặc quét tên"""
    rx_queue.put(("LOG", f"[*] Đang tìm kiếm module HC-05 (MAC: {target_mac})..."))
    try:
        device = await BleakScanner.find_device_by_address(target_mac, timeout=3.0)
        if device:
            rx_queue.put(("LOG", f"[+] Đã tìm thấy: {device.name or 'HC-05'} [{device.address}]"))
            return device
    except Exception:
        pass

    rx_queue.put(("LOG", "[*] Đang quét sóng BLE xung quanh..."))
    devices = await BleakScanner.discover(timeout=SCAN_TIMEOUT)
    for d in devices:
        if d.address.upper() == target_mac.upper():
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
            if notify_char and write_char:
                break
    return notify_char, write_char

async def tx_worker(client, write_char):
    """Vòng lặp gửi frame từ tx_queue qua BLE"""
    while client.is_connected:
        if not tx_queue.empty():
            msg = tx_queue.get()
            payload = msg.encode('utf-8')
            chunk_size = 20  # BLE MTU chuẩn 20 bytes
            try:
                for i in range(0, len(payload), chunk_size):
                    chunk = payload[i:i + chunk_size]
                    if "write-without-response" in write_char.properties:
                        await client.write_gatt_char(write_char, chunk, response=False)
                    else:
                        await client.write_gatt_char(write_char, chunk, response=True)
                    await asyncio.sleep(0.015)
                rx_queue.put(("TX", msg.strip()))
            except Exception as e:
                rx_queue.put(("ERR", f"Lỗi gửi TX: {e}"))
        await asyncio.sleep(0.02)

def notification_handler(sender, data: bytearray):
    """Xử lý dữ liệu nhận về từ STM32"""
    try:
        raw_text = data.decode('utf-8', errors='ignore')
        for line in raw_text.splitlines():
            line = line.strip()
            if not line:
                continue
            if line.startswith("ACK:"):
                status = line[4:].strip()
                rx_queue.put(("ACK", status))
            elif line.startswith("IMU:"):
                rx_queue.put(("IMU", line))
            else:
                rx_queue.put(("RX", line))
    except Exception as e:
        rx_queue.put(("ERR", f"Parse Error: {e}"))

async def ble_runner(target_mac, stop_event):
    """Tiến trình BLE chính"""
    while not stop_event.is_set():
        rx_queue.put(("STATUS", "CONNECTING"))
        device = await find_target_ble_device(target_mac)
        if not device:
            rx_queue.put(("STATUS", "DISCONNECTED"))
            rx_queue.put(("LOG", "[-] Không tìm thấy thiết bị. Thử lại sau 3s..."))
            await asyncio.sleep(3.0)
            continue

        try:
            async with BleakClient(device) as client:
                rx_queue.put(("LOG", f"[+] Đã kết nối thành công: {device.name or 'HC-05'}!"))
                rx_queue.put(("STATUS", "CONNECTED"))

                notify_char, write_char = await find_uart_characteristics(client)
                if not notify_char or not write_char:
                    rx_queue.put(("ERR", "Không tìm thấy UART Characteristics phù hợp!"))
                    break

                await client.start_notify(notify_char.uuid, notification_handler)
                tx_task = asyncio.create_task(tx_worker(client, write_char))

                while client.is_connected and not stop_event.is_set():
                    await asyncio.sleep(0.2)

                tx_task.cancel()
                try:
                    await client.stop_notify(notify_char.uuid)
                except Exception:
                    pass
        except Exception as e:
            rx_queue.put(("ERR", f"Mất kết nối: {e}"))
            rx_queue.put(("STATUS", "DISCONNECTED"))
            await asyncio.sleep(2.0)

    rx_queue.put(("STATUS", "DISCONNECTED"))

def ble_thread_entry(target_mac, stop_event):
    """Hàm chạy luồng BLE nền"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(ble_runner(target_mac, stop_event))
    except Exception as e:
        print(f"BLE Thread Error: {e}")


# ==================== GIAO DIỆN ĐIỀU KHIỂN ROBOT ==================== #
class RobotControllerWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Self-Balancing Robot - Direction Controller (HC-05)")
        self.resize(750, 680)
        self.setMinimumSize(650, 580)

        self.current_direction = "NONE"
        self.stop_ble_event = threading.Event()
        self.ble_thread = None

        self.init_ui()
        self.apply_dark_theme()

        # Timer kiểm tra hàng đợi rx_queue cập nhật GUI (50Hz)
        self.gui_timer = QtCore.QTimer()
        self.gui_timer.timeout.connect(self.process_rx_queue)
        self.gui_timer.start(20)

    def init_ui(self):
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QVBoxLayout(central_widget)
        main_layout.setSpacing(12)
        main_layout.setContentsMargins(16, 16, 16, 16)

        # ---------------- 1. KHU VỰC KẾT NỐI (CONNECTION BAR) ----------------
        conn_group = QtWidgets.QGroupBox("1. KẾT NỐI BLUETOOTH (HC-05)")
        conn_layout = QtWidgets.QHBoxLayout(conn_group)

        conn_layout.addWidget(QtWidgets.QLabel("MAC:"))
        self.txt_mac = QtWidgets.QLineEdit(TARGET_MAC)
        self.txt_mac.setPlaceholderText("VD: 19:36:B9:1A:5B:79")
        self.txt_mac.setMaximumWidth(170)
        conn_layout.addWidget(self.txt_mac)

        self.btn_connect = QtWidgets.QPushButton("KẾT NỐI")
        self.btn_connect.setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold; padding: 6px 14px;")
        self.btn_connect.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.btn_connect)

        conn_layout.addSpacing(15)
        self.lbl_conn_status = QtWidgets.QLabel("● CHƯA KẾT NỐI")
        self.lbl_conn_status.setStyleSheet("color: #E53935; font-weight: bold; font-size: 13px;")
        conn_layout.addWidget(self.lbl_conn_status)

        conn_layout.addStretch()
        main_layout.addWidget(conn_group)

        # ---------------- 2. KHU VỰC BẢNG ĐIỀU KHIỂN HƯỚNG (D-PAD) ----------------
        control_group = QtWidgets.QGroupBox("2. BẢNG ĐIỀU KHIỂN HƯỚNG DI CHUYỂN")
        control_layout = QtWidgets.QHBoxLayout(control_group)

        # Cột D-Pad bên trái
        dpad_container = QtWidgets.QWidget()
        dpad_grid = QtWidgets.QGridLayout(dpad_container)
        dpad_grid.setSpacing(10)

        btn_size = QtCore.QSize(110, 60)

        # Nút TIẾN (W)
        self.btn_fwd = QtWidgets.QPushButton("▲ TIẾN\n[W / ↑]")
        self.btn_fwd.setFixedSize(btn_size)
        self.btn_fwd.setStyleSheet("background-color: #1976D2; color: white; font-size: 13px; font-weight: bold; border-radius: 8px;")
        self.btn_fwd.clicked.connect(lambda: self.send_direction("FORWARD"))

        # Nút TRÁI (A)
        self.btn_left = QtWidgets.QPushButton("◄ TRÁI\n[A / ←]")
        self.btn_left.setFixedSize(btn_size)
        self.btn_left.setStyleSheet("background-color: #00897B; color: white; font-size: 13px; font-weight: bold; border-radius: 8px;")
        self.btn_left.clicked.connect(lambda: self.send_direction("LEFT"))

        # Nút DỪNG (Space)
        self.btn_stop = QtWidgets.QPushButton("■ DỪNG\n[SPACE]")
        self.btn_stop.setFixedSize(btn_size)
        self.btn_stop.setStyleSheet("background-color: #D32F2F; color: white; font-size: 14px; font-weight: bold; border-radius: 8px;")
        self.btn_stop.clicked.connect(lambda: self.send_direction("NONE"))

        # Nút PHẢI (D)
        self.btn_right = QtWidgets.QPushButton("► PHẢI\n[D / →]")
        self.btn_right.setFixedSize(btn_size)
        self.btn_right.setStyleSheet("background-color: #00897B; color: white; font-size: 13px; font-weight: bold; border-radius: 8px;")
        self.btn_right.clicked.connect(lambda: self.send_direction("RIGHT"))

        # Nút LÙI (S)
        self.btn_rev = QtWidgets.QPushButton("▼ LÙI\n[S / ↓]")
        self.btn_rev.setFixedSize(btn_size)
        self.btn_rev.setStyleSheet("background-color: #F57C00; color: white; font-size: 13px; font-weight: bold; border-radius: 8px;")
        self.btn_rev.clicked.connect(lambda: self.send_direction("REVERSED"))

        dpad_grid.addWidget(self.btn_fwd, 0, 1)
        dpad_grid.addWidget(self.btn_left, 1, 0)
        dpad_grid.addWidget(self.btn_stop, 1, 1)
        dpad_grid.addWidget(self.btn_right, 1, 2)
        dpad_grid.addWidget(self.btn_rev, 2, 1)

        control_layout.addWidget(dpad_container)
        control_layout.addSpacing(20)

        # Cột trạng thái & tùy chọn bên phải
        info_panel = QtWidgets.QWidget()
        info_layout = QtWidgets.QVBoxLayout(info_panel)
        info_layout.setSpacing(12)

        info_layout.addWidget(QtWidgets.QLabel("TRẠNG THÁI HIỆN TẠI:"))
        self.lbl_current_dir = QtWidgets.QLabel("■ ĐỨNG YÊN (NONE)")
        self.lbl_current_dir.setStyleSheet("""
            background-color: #212121;
            color: #E91E63;
            font-size: 18px;
            font-weight: bold;
            padding: 10px;
            border: 2px solid #E91E63;
            border-radius: 6px;
        """)
        self.lbl_current_dir.setAlignment(ALIGN_CENTER)
        info_layout.addWidget(self.lbl_current_dir)

        # Frame gửi gần nhất
        self.lbl_last_frame = QtWidgets.QLabel("Frame TX: Chưa gửi")
        self.lbl_last_frame.setStyleSheet("color: #9E9E9E; font-family: monospace; font-size: 12px;")
        info_layout.addWidget(self.lbl_last_frame)

        # ACK nhận gần nhất
        self.lbl_last_ack = QtWidgets.QLabel("Phản hồi ACK: Đang chờ...")
        self.lbl_last_ack.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 12px;")
        info_layout.addWidget(self.lbl_last_ack)

        # Chế độ Deadman switch
        self.chk_deadman = QtWidgets.QCheckBox("Nhả phím tự động DỪNG (Deadman's Switch)")
        self.chk_deadman.setChecked(True)
        self.chk_deadman.setStyleSheet("font-size: 12px; font-weight: bold; color: #FFD54F;")
        self.chk_deadman.setToolTip("Khi BẬT: Giữ phím W/A/S/D để chạy, thả phím xe sẽ tự động gửi lệnh DIR:NONE để dừng.")
        info_layout.addWidget(self.chk_deadman)

        info_layout.addStretch()
        control_layout.addWidget(info_panel)

        main_layout.addWidget(control_group)

        # ---------------- 3. KHU VỰC TERMINAL LOG ----------------
        log_group = QtWidgets.QGroupBox("3. NHẬT KÝ TRUYỀN THÔNG (COMMUNICATION LOG)")
        log_layout = QtWidgets.QVBoxLayout(log_group)

        self.txt_log = QtWidgets.QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setStyleSheet("""
            background-color: #121212;
            color: #00E676;
            font-family: Consolas, monospace;
            font-size: 12px;
            border-radius: 4px;
        """)
        log_layout.addWidget(self.txt_log)

        log_btn_layout = QtWidgets.QHBoxLayout()
        btn_clear = QtWidgets.QPushButton("Xóa màn hình Log")
        btn_clear.clicked.connect(self.txt_log.clear)
        log_btn_layout.addWidget(btn_clear)
        log_btn_layout.addStretch()

        self.lbl_guide = QtWidgets.QLabel("💡 Mẹo: Nhấn phím W/A/S/D để lái, Space để dừng khẩn cấp.")
        self.lbl_guide.setStyleSheet("color: #90CAF9; font-style: italic;")
        log_btn_layout.addWidget(self.lbl_guide)

        log_layout.addLayout(log_btn_layout)
        main_layout.addWidget(log_group)

    def apply_dark_theme(self):
        """Áp dụng bảng màu Dark Mode hiện đại"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1E1E1E;
            }
            QGroupBox {
                color: #FFFFFF;
                font-weight: bold;
                font-size: 13px;
                border: 1px solid #424242;
                border-radius: 6px;
                margin-top: 10px;
                padding-top: 14px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding-left: 10px;
                padding-right: 10px;
            }
            QLabel {
                color: #E0E0E0;
            }
            QLineEdit {
                background-color: #2D2D2D;
                color: #FFFFFF;
                border: 1px solid #555555;
                border-radius: 4px;
                padding: 4px 8px;
            }
            QPushButton {
                background-color: #333333;
                color: #FFFFFF;
                border: none;
                border-radius: 4px;
                padding: 6px 12px;
            }
            QPushButton:hover {
                background-color: #424242;
            }
            QPushButton:pressed {
                background-color: #262626;
            }
            QCheckBox {
                color: #E0E0E0;
            }
        """)

    # ---------------- XỬ LÝ KẾT NỐI BLE ---------------- #
    def toggle_connection(self):
        if self.ble_thread and self.ble_thread.is_alive():
            # Đang kết nối -> Bấm để Ngắt
            self.stop_ble_event.set()
            self.btn_connect.setEnabled(False)
            self.append_log("[*] Đang ngắt kết nối BLE...")
            QtCore.QTimer.singleShot(1500, self.reset_connect_button)
        else:
            # Chưa kết nối -> Bấm để Kết nối
            mac = self.txt_mac.text().strip()
            if not mac:
                QtWidgets.QMessageBox.warning(self, "Lỗi", "Vui lòng nhập địa chỉ MAC!")
                return
            self.stop_ble_event.clear()
            self.ble_thread = threading.Thread(target=ble_thread_entry, args=(mac, self.stop_ble_event), daemon=True)
            self.ble_thread.start()
            self.btn_connect.setText("NGẮT KẾT NỐI")
            self.btn_connect.setStyleSheet("background-color: #C62828; color: white; font-weight: bold; padding: 6px 14px;")

    def reset_connect_button(self):
        self.btn_connect.setEnabled(True)
        self.btn_connect.setText("KẾT NỐI")
        self.btn_connect.setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold; padding: 6px 14px;")
        self.lbl_conn_status.setText("● CHƯA KẾT NỐI")
        self.lbl_conn_status.setStyleSheet("color: #E53935; font-weight: bold; font-size: 13px;")

    # ---------------- GỬI LỆNH ĐIỀU HƯỚNG ---------------- #
    def send_direction(self, dir_name):
        """Đóng gói frame 'DIR:<direction>\\r\\n' và đẩy vào tx_queue"""
        # Nếu trùng với hướng hiện tại (và không phải NONE) thì bỏ qua để tránh spam
        if self.current_direction == dir_name and dir_name != "NONE":
            return

        self.current_direction = dir_name
        frame = f"DIR:{dir_name}\r\n"
        tx_queue.put(frame)

        self.lbl_last_frame.setText(f"Frame TX: {repr(frame)}")

        # Cập nhật style và nhãn trạng thái
        status_styles = {
            "FORWARD":  ("▲ ĐANG TIẾN (FORWARD)", "#1976D2", "border-color: #1976D2; color: #64B5F6;"),
            "REVERSED": ("▼ ĐANG LÙI (REVERSED)", "#F57C00", "border-color: #F57C00; color: #FFB74D;"),
            "LEFT":     ("◄ ĐANG RẼ TRÁI (LEFT)", "#00897B", "border-color: #00897B; color: #4DB6AC;"),
            "RIGHT":    ("► ĐANG RẼ PHẢI (RIGHT)", "#00897B", "border-color: #00897B; color: #4DB6AC;"),
            "NONE":     ("■ ĐỨNG YÊN (NONE)", "#E91E63", "border-color: #E91E63; color: #F06292;")
        }
        text, color, border = status_styles.get(dir_name, (f"Hướng: {dir_name}", "#FFFFFF", "border-color: #FFFFFF;"))
        self.lbl_current_dir.setText(text)
        self.lbl_current_dir.setStyleSheet(f"""
            background-color: #212121;
            font-size: 18px;
            font-weight: bold;
            padding: 10px;
            border: 2px solid;
            border-radius: 6px;
            {border}
        """)

    # ---------------- BẮT PHÍM BÀN PHÍM (W, A, S, D, SPACE) ---------------- #
    def keyPressEvent(self, event: QtGui.QKeyEvent):
        if event.isAutoRepeat():
            return

        # Nếu đang focus vào ô nhập MAC thì cho phép gõ bình thường
        if isinstance(self.focusWidget(), QtWidgets.QLineEdit):
            super().keyPressEvent(event)
            return

        key = event.key()
        if key in (KEY_W, KEY_UP):
            self.send_direction("FORWARD")
        elif key in (KEY_S, KEY_DOWN):
            self.send_direction("REVERSED")
        elif key in (KEY_A, KEY_LEFT):
            self.send_direction("LEFT")
        elif key in (KEY_D, KEY_RIGHT):
            self.send_direction("RIGHT")
        elif key == KEY_SPACE:
            self.send_direction("NONE")
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QtGui.QKeyEvent):
        if event.isAutoRepeat():
            return

        if isinstance(self.focusWidget(), QtWidgets.QLineEdit):
            super().keyReleaseEvent(event)
            return

        key = event.key()
        # Chế độ Deadman switch: Nhả phím điều hướng sẽ tự động gửi lệnh dừng NONE
        if self.chk_deadman.isChecked():
            if key in (KEY_W, KEY_UP, KEY_S, KEY_DOWN, KEY_A, KEY_LEFT, KEY_D, KEY_RIGHT):
                self.send_direction("NONE")

        super().keyReleaseEvent(event)

    # ---------------- TIẾN TRÌNH CẬP NHẬT LOG & TRẠNG THÁI ---------------- #
    def process_rx_queue(self):
        while not rx_queue.empty():
            msg_type, content = rx_queue.get()
            timestamp = time.strftime("%H:%M:%S")

            if msg_type == "STATUS":
                if content == "CONNECTED":
                    self.lbl_conn_status.setText("● ĐÃ KẾT NỐI")
                    self.lbl_conn_status.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 13px;")
                elif content == "CONNECTING":
                    self.lbl_conn_status.setText("● ĐANG KẾT NỐI...")
                    self.lbl_conn_status.setStyleSheet("color: #FFC107; font-weight: bold; font-size: 13px;")
                elif content == "DISCONNECTED":
                    self.lbl_conn_status.setText("● MẤT KẾT NỐI")
                    self.lbl_conn_status.setStyleSheet("color: #E53935; font-weight: bold; font-size: 13px;")

            elif msg_type == "ACK":
                self.lbl_last_ack.setText(f"Phản hồi ACK: STM32 -> [{content}]")
                if content == "OK":
                    self.lbl_last_ack.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 12px;")
                else:
                    self.lbl_last_ack.setStyleSheet("color: #F44336; font-weight: bold; font-size: 12px;")
                self.append_log(f"[{timestamp}] [ACK TỪ STM32]: {content}")

            elif msg_type == "TX":
                self.append_log(f"[{timestamp}] [TX GỬI ĐI]: {content}")

            elif msg_type == "LOG":
                self.append_log(f"[{timestamp}] {content}")

            elif msg_type == "ERR":
                self.append_log(f"[{timestamp}] [LỖI]: {content}")

    def append_log(self, text):
        self.txt_log.append(text)
        scrollbar = self.txt_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def closeEvent(self, event):
        """Xử lý dọn dẹp khi tắt ứng dụng"""
        self.stop_ble_event.set()
        event.accept()


# ==================== MAIN ENTRY POINT ==================== #
def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")

    window = RobotControllerWindow()
    window.show()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()
