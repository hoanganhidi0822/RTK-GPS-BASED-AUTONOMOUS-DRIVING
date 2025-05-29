import serial

def read_gps(port="/dev/ttyUSB0", baudrate=115200):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Đang đọc dữ liệu từ {port} ở {baudrate} baud...\n")
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            gps_data_split = line.split(',')
            print(gps_data_split)
    except serial.SerialException as e:
        print(f"Lỗi kết nối serial: {e}")
    except KeyboardInterrupt:
        print("\nĐã dừng.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    read_gps()
