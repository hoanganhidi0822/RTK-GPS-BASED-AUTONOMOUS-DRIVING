import serial
import time
import re

PORT = "/dev/ttyUSB1"  # Thay đổi nếu cần
BAUDRATE = 115200
TIMEOUT = 1.5

STATUS_KEYWORDS = {
    "INS_INACTIVE": "🔴 INS chưa kích hoạt",
    "INS_ALIGNING": "🟡 INS đang căn chỉnh (giữ thiết bị đứng yên)",
    "INS_ALIGNMENT_COMPLETE": "🟢 INS đã căn chỉnh (chờ RTK Fix)",
    "INS_SOLUTION_GOOD": "✅ INS Fusion hoạt động tốt",
    "INS_SOLUTION_FREE": "🟡 INS đang DR (mất GNSS)",
    "INS_ALIGNMENT_FAILED": "❌ INS lỗi căn chỉnh"
}

def parse_insstatus(data):
    for key, desc in STATUS_KEYWORDS.items():
        if key in data:
            return key, desc
    return None, None

def parse_fallback_status(data):
    if "#HEADINGA" in data or "$PTNL,AVR" in data:
        # Lấy yaw & tilt nếu có từ AVR
        avr_match = re.search(r"\$PTNL,AVR,.*?,\+?(-?\d+\.\d+),Yaw,\+?(-?\d+\.\d+),Tilt", data)
        if avr_match:
            yaw = float(avr_match.group(1))
            tilt = float(avr_match.group(2))
            return f"✅ INS đang hoạt động.\n   ↪ Yaw: {yaw:.2f}°, Tilt: {tilt:.2f}°"
        return "✅ INS đang hoạt động (tìm thấy HEADINGA hoặc AVR)"
    return None

def main():
    try:
        print(f"[⏳] Đang kết nối tới {PORT} @ {BAUDRATE} bps...")
        with serial.Serial(PORT, baudrate=BAUDRATE, timeout=TIMEOUT) as ser:
            ser.reset_input_buffer()
            ser.write(b"log insstatus once\r\n")
            time.sleep(0.6)
            data = ser.read(8192).decode(errors="ignore")

            print("[←] Phản hồi từ thiết bị:\n")
            print(data)

            # Ưu tiên phân tích INSSTATUS
            status_code, status_desc = parse_insstatus(data)
            if status_desc:
                print(f"\n[📌] Trạng thái INS (từ INSSTATUS): {status_desc}")
            else:
                # Fallback: kiểm tra HEADINGA hoặc PTNL,AVR
                fallback = parse_fallback_status(data)
                if fallback:
                    print(f"\n[📌] Trạng thái INS (qua fallback): {fallback}")
                else:
                    print("\n[📌] ⚠️ Không xác định được trạng thái INS.")

    except serial.SerialException as e:
        print(f"[❌] Lỗi kết nối serial: {e}")
    except Exception as e:
        print(f"[❌] Lỗi khác: {e}")

if __name__ == "__main__":
    main()
