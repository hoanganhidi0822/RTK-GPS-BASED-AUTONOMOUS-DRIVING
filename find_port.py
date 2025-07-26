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
gps_port, stm32_port = find_gps_and_stm32_ports()

if gps_port:
    print(f"[✅] GPS is on: {gps_port}")
else:
    print("[❌] GPS not found!")

if stm32_port:
    print(f"[✅] STM32 is on: {stm32_port}")
else:
    print("[❌] STM32 not found!")
