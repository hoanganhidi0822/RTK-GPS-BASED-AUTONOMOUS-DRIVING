import serial

# Chỉnh đúng cổng serial và baudrate
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

def parse_inspvax(line):
    try:
        # Example sentence: $INSPVAX,123456.00,12,1.2345,2.3456,3.4567,...*CS
        parts = line.strip().split(",")
        if len(parts) < 11:
            return None

        # Trích xuất dữ liệu
        lat = float(parts[3])
        lon = float(parts[4])
        heading = float(parts[8])
        ins_status = int(parts[2])  # 2 = INS GOOD, 3 = INS AIDING

        return lat, lon, heading, ins_status
    except Exception as e:
        print(f"[Parse Error] {e}")
        return None

def main():
    try:
        ser = serial.Serial(PORT, baudrate=BAUDRATE, timeout=1)
        print(f"[INFO] Listening on {PORT} at {BAUDRATE} baud...")

        while True:
            line = ser.readline().decode("utf-8", errors="ignore")
            if "$INSPVAX" in line:
                result = parse_inspvax(line)
                if result:
                    lat, lon, heading, ins_status = result
                    if ins_status == 3:  # INS AIDING
                        print(f"[INS AIDING] Lat: {lat:.6f}, Lon: {lon:.6f}, Heading: {heading:.2f}°")
    except KeyboardInterrupt:
        print("\n[INFO] Exit by user.")
    except Exception as e:
        print(f"[ERROR] {e}")

if __name__ == "__main__":
    main()
