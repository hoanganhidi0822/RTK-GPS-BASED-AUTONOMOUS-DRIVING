import serial
import time

# === Thông số Serial ===
PORT = "/dev/ttyUSB1"     # hoặc COMx nếu dùng Windows
BAUDRATE = 115200
TIMEOUT = 1

# === Cấu hình cụ thể của bạn ===
# Chỉnh sửa các giá trị dưới đây cho đúng với thực tế
IMU_AXIS_TYPE = 2                      # Hướng về đuôi xe
DR_LEVERARM = (1.0, -0.3, -1)        # anten cách module 1m phía trước, thấp hơn 0.3m

# === Lệnh cấu hình cần gửi ===
CONFIG_COMMANDS = [
    "inscontrol enable",
    "set smootheddr on",
    f"set imuaxestype {IMU_AXIS_TYPE}",
    f"set drleverarm {DR_LEVERARM[0]} {DR_LEVERARM[1]} {DR_LEVERARM[2]}",
    "saveconfig"
]
# CONFIG_COMMANDS = [
#     "inscontrol enable
    
# ]


def send_command(ser, cmd):
    """Gửi lệnh đến thiết bị và in phản hồi"""
    full_cmd = cmd.strip() + "\r\n"
    ser.write(full_cmd.encode('ascii'))
    print(f"[→] Sent: {cmd}")
    time.sleep(0.2)  # Delay giữa các lệnh
    response = ser.read_all().decode(errors='ignore')
    if response.strip():
        print(f"[←] Response:\n{response}\n")
    else:
        print("[←] No response or silent ACK.\n")

def main():
    try:
        print(f"[⏳] Connecting to {PORT} at {BAUDRATE} bps...")
        ser = serial.Serial(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        print(f"[✅] Connected to {PORT}")

        for cmd in CONFIG_COMMANDS:
            send_command(ser, cmd)

        print("[✅] Cấu hình hoàn tất. Vui lòng khởi động lại thiết bị để áp dụng.")

    except serial.SerialException as e:
        print(f"[❌] Serial error: {e}")
    except Exception as e:
        print(f"[❌] Unexpected error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("[🔌] Đã ngắt kết nối.")

if __name__ == "__main__":
    main()
