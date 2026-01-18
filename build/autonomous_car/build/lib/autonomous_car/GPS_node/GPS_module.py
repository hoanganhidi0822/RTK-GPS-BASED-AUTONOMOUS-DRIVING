import serial
import serial.tools.list_ports
import time
import config as cf


cf.latitude = None
cf.longitude = None
cf.heading = None

def dec2deg(value):
    if not value:
        return None
    dec = value / 100.00
    deg = int(dec)
    minutes = (dec - deg) * 100 / 60
    position = deg + minutes
    return "{:.10f}".format(position)

def is_gps_device(port_path, baudrate=115200):
    try:
        with serial.Serial(port_path, baudrate=baudrate, timeout=0.1) as ser:
            for _ in range(2):
                line = ser.readline().decode(errors="ignore").strip()
                if line.startswith("$GP"):
                    print(f"[INFO] ✅ GPS detected on {port_path}")
                    return True
    except Exception:
        pass
    return False

def find_gps_port():
    ports = [p.device for p in serial.tools.list_ports.comports() if "ttyUSB" in p.device]
    for port in ports:
        print(f"[INFO] 🔍 Probing {port}...")
        if is_gps_device(port):
            return port
    return None

def get_gps_data(ser=None, baudrate=115200, timeout=0.1):
    lat = lon = heading = sat_count = rtk_status = speed = None

    while True:
        try:
            # (Re)connect if serial is None or closed
            if ser is None:
                port = find_gps_port()
                if not port:
                    print("[INFO] ⏳ Waiting for GPS device...")
                    # time.sleep(0.1)
                    continue
                print(f"[INFO] Connecting to {port}...")
                ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)

            # Read line from GPS
            gps_data = ser.readline().decode("utf8", errors="ignore").strip()

            if gps_data:
                gps_data_split = gps_data.split(',')

                if gps_data.startswith("$GPYBM") and len(gps_data_split) > 6 and gps_data_split[6]:
                    heading = float(gps_data_split[6])

                elif gps_data.startswith("$GPGGA") and len(gps_data_split) > 9:
                    lat = dec2deg(float(gps_data_split[2])) if gps_data_split[2] else None
                    lon = dec2deg(float(gps_data_split[4])) if gps_data_split[4] else None
                    sat_count = int(gps_data_split[7]) if gps_data_split[7] else None
                    fix_quality = int(gps_data_split[6]) if gps_data_split[6] else 0

                    rtk_status = {
                        4: "RTK Fixed",
                        5: "RTK Float",
                        2: "DGPS",
                        1: "Single",
                        6: "RTK INS Fusion"
                    }.get(fix_quality, "GPS Weak")

                elif gps_data.startswith("$GPVTG") and len(gps_data_split) > 7 and gps_data_split[7]:
                    speed = float(gps_data_split[7])

                if lat and lon and heading and sat_count and rtk_status and speed is not None:
                    return lat, lon, heading, sat_count, rtk_status, speed, ser

        except (serial.SerialException, OSError) as e:
            print(f"[WARN] 🔌 Lost connection: {e}")
            try:
                if ser:
                    ser.close()
            except:
                pass
            ser = None
            print("[INFO] 🔄 Waiting for GPS reconnect...")
            time.sleep(0.1)

        except UnicodeDecodeError:
            continue

        except Exception as e:
            print(f"[ERROR] ❗ Unexpected error: {e}")
            time.sleep(0.1)

import serial
import time

import time
import math

previous_heading = None
previous_time = None

def angular_diff(h1, h2):
    delta = (h1 - h2 + 180) % 360 - 180
    return delta  # giữ được dấu

def get_gps_data_for_dead_reckoning(ser, baudrate=115200, timeout=0.1):
    global previous_heading, previous_time

    lat = lon = heading = pitch = roll = None
    vN = vE = vU = 0.0
    sat_count = ins_status = rtk_status = speed = hdop = age = None

    while True:
        try:
            if ser is None or not ser.is_open:
                port = find_gps_port()
                if not port:
                    print("[INFO] ⏳ Waiting for GPS device...")
                    time.sleep(0.1)
                    continue
                print(f"[INFO] Connecting to {port}...")
                ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)

            if ser.in_waiting:
                gps_data = ser.readline().decode("utf8", errors="ignore").strip()
                if not gps_data:
                    continue

                parts = gps_data.split(',')

                # Parse $GPYBM — GNSS + IMU fusion
                if gps_data.startswith("$GPYBM") and len(parts) >= 20:
                    lat = float(parts[3])
                    lon = float(parts[4])
                    raw_heading = float(parts[6])
                    pitch = float(parts[7])
                    roll = float(parts[8])
                    vN = float(parts[9])
                    vE = float(parts[10])
                    vU = float(parts[11])
                    ins_status = int(parts[17]) if parts[17].isdigit() else None
                    sat_count = int(parts[18]) if parts[18].isdigit() else None
                    age = float(parts[20]) if parts[20] else None

                    # Heading filtering (anti-jump)
                    current_time = time.time()
                    
                    max_rate = 80  # deg/s
                    if previous_heading is None or previous_time is None:
                        heading = raw_heading
                    else:
                        delta = angular_diff(raw_heading, previous_heading)
                        dt = current_time - previous_time
                        max_delta = max_rate * dt

                        if abs(delta) > max_delta:
                            heading = (previous_heading + max_delta * (1 if delta > 0 else -1)) % 360
                            print(f"[WARN] ⚠️ Capped heading change: Δ{delta:.1f}° → applied Δ={max_delta:.1f}°")
                        else:
                            heading = raw_heading



                    previous_heading = heading
                    previous_time = current_time

                    rtk_status = {
                        4: "RTK Fixed",
                        5: "RTK Float",
                        2: "DGPS",
                        1: "Single",
                        6: "RTK INS Fusion",
                    }.get(ins_status, "Unknown")

                elif gps_data.startswith("$GPVTG") and len(parts) > 7 and parts[7]:
                    speed = float(parts[7])

                elif gps_data.startswith("$GPGGA") and len(parts) > 13:
                    try:
                        hdop = float(parts[8]) if parts[8] else None
                        age = float(parts[13]) if parts[13] else None
                    except:
                        pass

                if (
                    lat is not None and lon is not None and
                    heading is not None and speed is not None and
                    age is not None and age < 1.3
                ):
                    return lat, lon, heading, sat_count, rtk_status, speed, age, ser

            else:
                time.sleep(0.01)

        except UnicodeDecodeError:
            continue

        except (serial.SerialException, OSError) as e:
            print(f"[WARN] 🔌 Lost connection: {e}")
            try:
                if ser:
                    ser.close()
            except:
                pass
            ser = None
            print("[INFO] 🔄 Waiting for GPS reconnect...")
            time.sleep(0.1)

        except Exception as e:
            print(f"[ERROR] ❗ Unexpected error: {e}")
            time.sleep(0.1)
