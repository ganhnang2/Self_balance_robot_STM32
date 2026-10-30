#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
  HC-05 BLE IMU CSV LOGGER
=============================================================================
  Chương trình nhận dữ liệu IMU từ STM32 qua Bluetooth BLE (HC-05)
  và ghi trực tiếp vào file .csv.

  Cấu trúc dữ liệu nhận từ STM32:
    "IMU:<Gyro_angle>,<Accel_angle>,<Complement_angle>,<Kalman_angle>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>"
    (10 thông số, phân cách bởi dấu phẩy)

  File CSV xuất ra có dạng:
    Time_sec, Gyro_angle, Accel_angle, Complement_angle, Kalman_angle, Ax, Ay, Az, Gx, Gy, Gz

  Cách sử dụng:
    python hc05_csv_logger.py
    python hc05_csv_logger.py --output my_data.csv
    python hc05_csv_logger.py --mac "AA:BB:CC:DD:EE:FF"

  Nhấn Ctrl+C để dừng ghi và đóng file an toàn.
=============================================================================
"""

import sys
import os
import csv
import time
import asyncio
import argparse
import threading
from bleak import BleakScanner, BleakClient

# ==================== CẤU HÌNH MẶC ĐỊNH ==================== #
TARGET_MAC   = "19:36:B9:1A:5B:79"
TARGET_NAME  = "HC05"
SCAN_TIMEOUT = 6.0

COMMON_UUIDS = [
    "0000ffe1-0000-1000-8000-00805f9b34fb",
    "0000ffe2-0000-1000-8000-00805f9b34fb",
]

# UUID thường dùng cho kênh WRITE (gửi dữ liệu đến HC-05)
COMMON_WRITE_UUIDS = [
    "0000ffe2-0000-1000-8000-00805f9b34fb",
    "0000ffe1-0000-1000-8000-00805f9b34fb",
]

CSV_HEADER = [
    'Time_sec',
    'Gyro_angle',
    'Accel_angle',
    'Complement_angle',
    'Kalman_angle',
    'Ax',
    'Ay',
    'Az',
    'Gx',
    'Gy',
    'Gz',
]

NUM_IMU_FIELDS = 10  # Số thông số trong chuỗi IMU (phải khớp với header - 1)

# ==================== BIẾN TOÀN CỤC ==================== #
rx_stream_buffer = ""   # Bộ đệm ghép gói BLE chống phân mảnh MTU
start_time       = None # Mốc thời gian bắt đầu nhận dữ liệu
csv_file         = None
csv_writer       = None
sample_count     = 0

write_char       = None               # Characteristic dùng để GỬI lệnh xuống STM32
cmd_queue        = asyncio.Queue()    # Hàng đợi lệnh: keyboard_thread -> ble event loop
ble_loop         = None               # Tham chiếu đến event loop để thread bàn phím enqueue an toàn

# ==================== CÁC LỆNH HỖ TRỢ ==================== #
KEYBOARD_HELP = """
┌─────────────────────────────────────────────────────┐
│             LỆNH ĐIỀU KHIỂN QUA BÀN PHÍM            │
├──────────┬──────────────────────────────────────────┤
│  Phím    │  Chức năng                               │
├──────────┼──────────────────────────────────────────┤
│  1       │  Chế độ RAMP   (tăng/giảm dần đều)       │
│  2       │  Chế độ RANDOM (tốc độ ngẫu nhiên)       │
│  0       │  STOP          (dừng động cơ)             │
│  Chuỗi   │  Gửi thẳng chuỗi tùy ý xuống STM32      │
│  q/quit  │  Thoát chương trình                      │
└──────────┴──────────────────────────────────────────┘
Gõ lệnh rồi nhấn Enter để gửi:
"""


# ==================== TÌM THIẾT BỊ BLE ==================== #
async def find_target_ble_device(target_mac):
    """Tìm module HC-05 theo địa chỉ MAC, nếu không thấy thì quét sóng BLE"""
    print(f"[*] Đang tìm kiếm module BLE HC-05 (MAC: {target_mac})...")
    try:
        device = await BleakScanner.find_device_by_address(target_mac, timeout=3.0)
        if device:
            print(f"[+] ĐÃ TÌM THẤY HC-05: {device.name or TARGET_NAME} [{device.address}]")
            return device
    except Exception:
        pass

    print(f"[*] Đang quét sóng BLE xung quanh...")
    devices = await BleakScanner.discover(timeout=SCAN_TIMEOUT)
    for d in devices:
        if d.address.upper() == target_mac.upper():
            print(f"[+] ĐÃ TÌM THẤY (quét sóng): {d.name or TARGET_NAME} [{d.address}]")
            return d
    return None


async def find_uart_characteristics(client: BleakClient):
    """Tìm UUID cho luồng nhận (Notify) trên thiết bị BLE"""
    notify_char = None
    for service in client.services:
        for char in service.characteristics:
            uuid_str = str(char.uuid).lower()
            if uuid_str in COMMON_UUIDS:
                if "notify" in char.properties or "indicate" in char.properties:
                    notify_char = char
                    break
        if notify_char:
            break

    # Fallback nếu không có UUID chuẩn HC-05
    if not notify_char:
        for service in client.services:
            for char in service.characteristics:
                if "notify" in char.properties or "indicate" in char.properties:
                    notify_char = char
                    break
            if notify_char:
                break

    return notify_char


async def find_write_characteristic(client: BleakClient):
    """Tìm UUID cho luồng gửi (Write / WriteWithoutResponse) trên thiết bị BLE"""
    write_char_found = None
    for service in client.services:
        for char in service.characteristics:
            uuid_str = str(char.uuid).lower()
            if uuid_str in COMMON_WRITE_UUIDS:
                if "write" in char.properties or "write-without-response" in char.properties:
                    write_char_found = char
                    break
        if write_char_found:
            break

    # Fallback: tìm bất kỳ kênh write nào
    if not write_char_found:
        for service in client.services:
            for char in service.characteristics:
                if "write" in char.properties or "write-without-response" in char.properties:
                    write_char_found = char
                    break
            if write_char_found:
                break

    return write_char_found


# ==================== LUỒNG NHẬN LỆNH TỪ BÀN PHÍM ==================== #
def keyboard_input_thread(loop: asyncio.AbstractEventLoop):
    """
    Thread riêng biệt chạy song song với asyncio event loop.
    Đọc input từ bàn phím (blocking) rồi enqueue vào cmd_queue.
    Không bao giờ gọi trực tiếp BLE write ở đây — đó là việc của coroutine ble_sender.
    """
    while True:
        try:
            cmd = input("")  # Blocking input - an toàn vì đang ở thread riêng
        except EOFError:
            break

        cmd = cmd.strip()
        if not cmd:
            continue

        if cmd.lower() in ('q', 'quit', 'exit'):
            print("\n[*] Người dùng yêu cầu thoát.")
            asyncio.run_coroutine_threadsafe(
                cmd_queue.put("__QUIT__"), loop
            )
            break

        # Enqueue lệnh vào event loop asyncio một cách thread-safe
        asyncio.run_coroutine_threadsafe(
            cmd_queue.put(cmd), loop
        )


async def ble_sender(client: BleakClient):
    """
    Coroutine lắng nghe cmd_queue và gửi lệnh xuống STM32 qua BLE.
    Chạy song song với start_notify trong cùng event loop.
    """
    global write_char

    while client.is_connected:
        try:
            cmd = await asyncio.wait_for(cmd_queue.get(), timeout=0.5)
        except asyncio.TimeoutError:
            continue

        if cmd == "__QUIT__":
            break

        if write_char is None:
            print(f"\n[!] Không tìm thấy kênh Write BLE. Không thể gửi lệnh.")
            continue

        # Chuyển đổi phím tắt thành chuỗi lệnh thân thiện
        label_map = {'0': 'STOP', '1': 'RAMP', '2': 'RAND'}
        display_cmd = label_map.get(cmd, cmd)

        # Đảm bảo chuỗi kết thúc bằng \r\n để STM32 nhận dạng IDLE line
        payload = cmd if cmd.endswith('\n') else cmd + '\r\n'

        try:
            use_response = "write" in write_char.properties
            await client.write_gatt_char(
                write_char.uuid,
                payload.encode('utf-8'),
                response=use_response
            )
            print(f"\n[TX -> STM32] Đã gửi: '{display_cmd}'")
        except Exception as e:
            print(f"\n[!] Lỗi khi gửi BLE: {e}")


# ==================== XỬ LÝ DỮ LIỆU NHẬN VỀ ==================== #
def notification_handler(sender, data: bytearray):
    """
    Callback xử lý dữ liệu nhận về từ STM32.
    Sử dụng bộ đệm rx_stream_buffer để ghép các gói BLE bị phân mảnh (MTU 20 bytes)
    thành từng dòng hoàn chỉnh trước khi phân tích.
    """
    global rx_stream_buffer, start_time, csv_writer, csv_file, sample_count

    try:
        chunk = data.decode('utf-8', errors='ignore')
    except Exception:
        return

    if start_time is None:
        start_time = time.time()

    rx_stream_buffer += chunk

    # Tách từng dòng hoàn chỉnh (kết thúc bằng '\n')
    while '\n' in rx_stream_buffer:
        line, rx_stream_buffer = rx_stream_buffer.split('\n', 1)
        line = line.strip()

        if not line:
            continue

        # Phân tích cú pháp: "IMU:<Gyro_angle>,<Accel_angle>,<Complement_angle>,<Kalman_angle>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>"
        if line.startswith("IMU:"):
            payload = line[4:]
            parts = payload.split(',')

            if len(parts) == NUM_IMU_FIELDS:
                try:
                    values = [float(x) for x in parts]
                    gyro_angle, accel_angle, complement_angle, kalman_angle, ax, ay, az, gx, gy, gz = values

                    # Tính thời gian tương đối (giây)
                    time_sec = time.time() - start_time

                    # Ghi vào file CSV
                    if csv_writer is not None:
                        csv_writer.writerow([
                            f"{time_sec:.4f}",
                            f"{gyro_angle:.3f}",
                            f"{accel_angle:.3f}",
                            f"{complement_angle:.3f}",
                            f"{kalman_angle:.3f}",
                            f"{ax:.4f}",
                            f"{ay:.4f}",
                            f"{az:.4f}",
                            f"{gx:.3f}",
                            f"{gy:.3f}",
                            f"{gz:.3f}",
                        ])
                        sample_count += 1

                        # Xả buffer mỗi 50 mẫu để đảm bảo dữ liệu được ghi xuống ổ đĩa
                        if sample_count % 50 == 0:
                            csv_file.flush()

                    # In ra terminal để theo dõi
                    sys.stdout.write(
                        f"\r[{sample_count:>6d}] t={time_sec:>7.2f}s | "
                        f"Gyro: {gyro_angle:>+7.2f}° | Accel: {accel_angle:>+7.2f}° | Compl: {complement_angle:>+7.2f}° | Kalman: {kalman_angle:>+7.2f}° | "
                        f"Acc: [{ax:>+5.2f}, {ay:>+5.2f}, {az:>+5.2f}] g | "
                        f"Gyr: [{gx:>+6.1f}, {gy:>+6.1f}, {gz:>+6.1f}] °/s   "
                    )
                    sys.stdout.flush()
                except ValueError:
                    pass
            else:
                print(f"\n[?] IMU frame sai số lượng trường: {len(parts)} (cần {NUM_IMU_FIELDS}): {line}")

        elif line.startswith("ACK:"):
            status = line[4:].strip()
            print(f"\n[ACK] -> STM32 Phản hồi: {status}")

        else:
            print(f"\n[RX] {line}")


# ==================== VÒNG LẶP CHÍNH BLE ==================== #
async def ble_main(target_mac):
    """Kết nối BLE tới HC-05, nhận dữ liệu IMU và gửi lệnh từ bàn phím"""
    global write_char

    while True:
        device = await find_target_ble_device(target_mac)
        if not device:
            print("[!] Không tìm thấy thiết bị. Thử lại sau 3 giây...")
            await asyncio.sleep(3.0)
            continue

        print(f"\n[*] Đang thiết lập kết nối BLE tới HC-05 [{device.address}]...")
        try:
            async with BleakClient(device) as client:
                if not client.is_connected:
                    print("[!] Không thể kết nối tới HC-05. Đang thử lại...")
                    await asyncio.sleep(2.0)
                    continue

                print(f"[+] KẾT NỐI BLE THÀNH CÔNG VỚI {device.name or TARGET_NAME}!")

                # Tìm kênh nhận (Notify)
                notify_char = await find_uart_characteristics(client)
                if not notify_char:
                    print("[!] LỖI: Không tìm thấy kênh UART Notify trên thiết bị.")
                    return

                # Tìm kênh gửi (Write)
                write_char = await find_write_characteristic(client)
                if write_char:
                    print(f"[+] Tìm thấy kênh Write BLE: {write_char.uuid}")
                    print(f"[+] Có thể gửi lệnh từ bàn phím xuống STM32.")
                else:
                    print(f"[!] Cảnh báo: Không tìm thấy kênh Write. Chỉ có thể nhận dữ liệu.")

                print(KEYBOARD_HELP)
                print(f"[+] Bắt đầu nhận dữ liệu IMU...\n")

                await client.start_notify(notify_char.uuid, notification_handler)

                # Chạy coroutine gửi lệnh song song với vòng lặp nhận
                sender_task = asyncio.create_task(ble_sender(client))

                try:
                    while client.is_connected:
                        await asyncio.sleep(0.5)
                        # Kiểm tra xem người dùng có yêu cầu thoát không
                        if not sender_task.done():
                            pending = cmd_queue.qsize()
                            _ = pending  # suppress unused warning
                        else:
                            break
                except asyncio.CancelledError:
                    pass
                finally:
                    sender_task.cancel()
                    try:
                        await client.stop_notify(notify_char.uuid)
                    except Exception:
                        pass
                    print("\n[*] Mất kết nối BLE.")

        except Exception as e:
            print(f"\n[!] Lỗi kết nối BLE: {e}")
            print("[*] Đang thử kết nối lại sau 2 giây...")
            await asyncio.sleep(2.0)


# ==================== MAIN ENTRY POINT ==================== #
def main():
    global csv_file, csv_writer, ble_loop

    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description="HC-05 BLE IMU CSV Logger - Nhận dữ liệu IMU từ STM32 và ghi file .csv"
    )
    parser.add_argument(
        "--mac", type=str, default=TARGET_MAC,
        help=f"Địa chỉ MAC của module HC-05 (mặc định: {TARGET_MAC})"
    )
    parser.add_argument(
        "--output", "-o", type=str, default=None,
        help="Tên file CSV xuất ra (mặc định: imu_log_YYYYMMDD_HHMMSS.csv)"
    )
    args = parser.parse_args()

    # Tạo tên file CSV tự động nếu không chỉ định
    if args.output:
        csv_filename = args.output
    else:
        csv_filename = f"imu_log_angle_param4.csv"

    # Đảm bảo thư mục tồn tại
    csv_dir = os.path.dirname(os.path.abspath(csv_filename))
    if not os.path.exists(csv_dir):
        os.makedirs(csv_dir, exist_ok=True)

    print("=" * 72)
    print("  HC-05 BLE IMU CSV LOGGER  +  KEYBOARD COMMAND SENDER")
    print("=" * 72)
    print(f"  MAC Target  : {args.mac}")
    print(f"  File CSV    : {os.path.abspath(csv_filename)}")
    print(f"  IMU Format  : IMU:<Gyro_angle>,<Accel_angle>,<Complement_angle>,<Kalman_angle>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>")
    print(f"  Nhấn Ctrl+C hoặc gõ 'q' để dừng ghi và đóng file an toàn.")
    print("=" * 72)

    # Mở file CSV và ghi Header
    csv_file = open(csv_filename, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(CSV_HEADER)
    csv_file.flush()

    print(f"\n[+] Đã tạo file CSV: {csv_filename}")
    print(f"[+] Header: {CSV_HEADER}\n")

    # Tạo event loop riêng để có thể tham chiếu từ keyboard thread
    ble_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(ble_loop)

    # Khởi động thread bàn phím (daemon=True: tự thoát khi main thread thoát)
    kb_thread = threading.Thread(
        target=keyboard_input_thread,
        args=(ble_loop,),
        daemon=True,
        name="KeyboardInputThread"
    )
    kb_thread.start()

    # Chạy vòng lặp BLE trên event loop riêng
    try:
        ble_loop.run_until_complete(ble_main(args.mac))
    except KeyboardInterrupt:
        print(f"\n\n[*] Người dùng dừng chương trình (Ctrl+C).")
    finally:
        ble_loop.close()
        # Đóng file CSV an toàn
        if csv_file:
            csv_file.flush()
            csv_file.close()
            duration = time.time() - start_time if start_time else 0.0
            print(f"[+] ĐÃ LƯU FILE CSV THÀNH CÔNG!")
            print(f"    -> File   : {os.path.abspath(csv_filename)}")
            print(f"    -> Số mẫu : {sample_count}")
            print(f"    -> Thời gian: {duration:.1f} giây")


if __name__ == "__main__":
    main()
