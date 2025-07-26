import serial
import serial.tools.list_ports
import time
import config as cf

cf.camera_error = 0

import serial.tools.list_ports
import serial
import time

def find_gps_and_stm32_ports(baudrate=115200):
    gps_port = None
    stm32_port = None

    ports = [p.device for p in serial.tools.list_ports.comports() if "ttyUSB" in p.device]
    print(f"[INFO] 🔍 Found ports: {ports}")

    for port in ports:
        try:
            with serial.Serial(port, baudrate=baudrate, timeout=0.3) as ser:
                for _ in range(5):
                    line = ser.readline().decode(errors="ignore").strip()
                    if line.startswith("$GP"):  # NMEA pattern
                        gps_port = port
                        break
        except Exception:
            continue

    if gps_port:
        # STM32 là port còn lại
        other_ports = [p for p in ports if p != gps_port]
        if other_ports:
            stm32_port = other_ports[0]

    return gps_port, stm32_port



class STM32:
    def __init__(self, port=None, baudrate=115200, known_gps_port=None):
        self.port = port
        self.baudrate = baudrate
        self.known_gps_port = known_gps_port
        self.stm32 = None
        self.connect()

    def connect(self):
        while True:
            if self.port is None:
                gps_port, stm32_port = find_gps_and_stm32_ports()
                
                self.port = stm32_port
                if not self.port:
                    print("[INFO] ⏳ Waiting for STM32 device...")
                    time.sleep(1)
                    continue
            try:
                self.stm32 = serial.Serial(self.port, self.baudrate, timeout=0.1)
                print(f"[INFO] ✅ Connected to STM32 at {self.port}")
                return
            except serial.SerialException as e:
                print(f"[WARN] 🔌 Cannot connect to {self.port}: {e}")
                self.port = None
                time.sleep(0.1)

    def __call__(self, angle=0, speed=0, brake_state=False):
        if self.stm32 is None or not self.stm32.is_open:
            print("[WARNING] Serial port is not available. Reconnecting...")
            self.connect()
            return

        try:
            angle = self.parse_angle(angle)
            speed = self.parse_speed(speed)
            brake = self.parse_brake(brake_state)

            data_to_send = self.preprocess(angle, speed, brake)
            self.stm32.write(data_to_send.encode())

        except (serial.SerialException, OSError) as e:
            cf.camera_error = 1
            print(f"[ERROR] ❌ Serial communication failed: {e}")
            try:
                if self.stm32:
                    self.stm32.close()
            except:
                pass
            self.stm32 = None
            self.port = None
            self.connect()
        except ValueError as e:
            print(f"[ERROR] Invalid input: {e}")

    @staticmethod
    def parse_angle(angle):
        if -35 <= angle <= 35:
            return 100 + abs(angle) if angle < 0 else angle
        else:
            raise ValueError("Angle must be between -35 and 35.")

    @staticmethod
    def parse_speed(speed):
        if 0 <= speed <= 10:
            return speed
        else:
            raise ValueError("Speed must be between 0 and 10.")

    @staticmethod
    def parse_brake(brake_state):
        return 1 if brake_state else 0

    @staticmethod
    def preprocess(angle, speed, brake):
        angle_str = f"{angle:03}"
        speed_str = f"{speed:03}"
        brake_str = f"{brake:01}"
        return angle_str + speed_str + brake_str
