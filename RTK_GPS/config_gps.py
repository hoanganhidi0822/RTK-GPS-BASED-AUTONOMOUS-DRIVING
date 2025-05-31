import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

# Lệnh cần gửi đến thiết bị
CMD = "log inspvax ontime 1\r\n"  # Phải kết thúc bằng \r\n

def send_command():
    with serial.Serial(PORT, baudrate=BAUDRATE, timeout=1) as ser:
        print("[INFO] Sending command to GNSS receiver...")
        ser.write(CMD.encode())
        time.sleep(0.5)

        # Đọc phản hồi nếu có
        while ser.in_waiting:
            response = ser.readline().decode(errors="ignore").strip()
            print(f"[RESPONSE] {response}")

if __name__ == "__main__":
    send_command()
