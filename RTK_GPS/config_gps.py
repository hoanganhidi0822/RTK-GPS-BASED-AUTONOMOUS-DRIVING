import serial

# Thông số Serial
PORT = "/dev/ttyUSB1"   # Cập nhật theo cổng thật
BAUDRATE = 115200

def parse_ins_status(line):
    try:
        parts = line.strip().split(',')
        if "$INSPVAX" in line or "$INSSTATUS" in line:
            ins_status = int(parts[2])
            return ins_status
    except Exception as e:
        print(f"[ERROR parsing] {e}")
    return None

def explain_status(status):
    if status == 0:
        return "❌ INS chưa hoạt động (No INS)"
    elif status == 1:
        return "🔄 Đang căn chỉnh IMU (Aligning...)"
    elif status == 2:
        return "✅ Fusion hoạt động (INS Good)"
    elif status == 3:
        return "✅ Fusion vẫn duy trì (INS Aiding - mất RTK)"
    else:
        return "❓ Trạng thái không xác định"

def main():
    print(f"[INFO] Kết nối cổng {PORT}...")
    with serial.Serial(PORT, baudrate=BAUDRATE, timeout=1) as ser:
        while True:
            try:
                line = ser.readline().decode("utf-8", errors="ignore")
                print(line)
                if "$INSPVAX" in line or "$INSSTATUS" in line:
                    status = parse_ins_status(line)
                    if status is not None:
                        print(f"[INS_STATUS = {status}] {explain_status(status)}")
            except KeyboardInterrupt:
                print("\n[INFO] Thoát.")
                break

if __name__ == "__main__":
    main()
