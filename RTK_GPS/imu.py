import serial
import struct
import time
import math
import serial.tools.list_ports
import adafruit_bno055

class BNO055Compass:
    def __init__(self, baudrate=115200, timeout=0.1):
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.sensor = None
        self.connect()

    def _read_magnetic_raw(self, uart):
        for _ in range(3):
            uart.reset_input_buffer()
            uart.write(bytes([0xAA, 0x01, 0x0E, 6]))
            start = time.monotonic()
            while uart.in_waiting < 8 and (time.monotonic() - start) < self.timeout:
                pass
            if uart.in_waiting >= 8:
                resp = uart.read(uart.in_waiting)
                if resp[0] == 0xBB and len(resp) >= 8:
                    data = resp[2:8]
                    try:
                        x, y, z = struct.unpack("<hhh", data)
                        return x, y, z
                    except:
                        pass
        return None

    def _find_bno055_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            try:
                uart = serial.Serial(port.device, baudrate=self.baudrate, timeout=self.timeout)
                time.sleep(0.1)
                result = self._read_magnetic_raw(uart)
                uart.close()
                if result:
                    print(f"✅ Found BNO055 at {port.device} → Magnetic: {result}")
                    return port.device
            except:
                continue
        return None

    def connect(self):
        port = self._find_bno055_port()
        if not port:
            raise RuntimeError("❌ No BNO055 IMU found.")
        
        self.ser = serial.Serial(port, baudrate=self.baudrate, timeout=self.timeout)
        self.sensor = adafruit_bno055.BNO055_UART(self.ser)

        # Đợi sensor sẵn sàng (rất quan trọng!)
        time.sleep(1)

        # Chuyển về chế độ fusion
        self.sensor.mode = adafruit_bno055.NDOF_MODE

        # Đợi tiếp để ổn định sau khi set mode
        time.sleep(0.5)

        print(f"📡 Connected to IMU on {port}")


    def read_heading(self):
        try:
            values = self.sensor.magnetic
            if values is not None:
                heading = math.degrees(math.atan2(values[1], values[0]))
                return (heading + 360) % 360
        except RuntimeError as e:
            print(f"⚠️ IMU RuntimeError: {e}")
            self._safe_reconnect()
        except Exception as e:
            print(f"❌ IMU lỗi khác: {e}")
            self._safe_reconnect()
        return None
    def _safe_reconnect(self):
        try:
            if self.ser:
                self.ser.close()
        except:
            pass
        time.sleep(0.5)
        self.connect()
    def measure_fps(self, duration=1.0):
        count = 0
        start_time = time.time()
        while time.time() - start_time < duration:
            heading = self.read_heading()
            if heading is not None:
                count += 1
            time.sleep(0.01)
        return count

if __name__ == "__main__":
    compass = BNO055Compass()
    print("📡 Bắt đầu đo Heading và FPS...")
    frame_count = 0
    start_time = time.time()

    while True:
        heading = compass.read_heading()
        if heading is not None:
            print(f"🧭 Heading: {heading:.2f}°")
            
        time.sleep(0.05)
