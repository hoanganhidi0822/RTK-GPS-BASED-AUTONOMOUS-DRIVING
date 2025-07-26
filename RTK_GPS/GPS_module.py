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

def get_gps_data(ser=None, baudrate=115200, timeout=0.2):
    lat = lon = heading = sat_count = rtk_status = speed = None

    while True:
        try:
            # (Re)connect if serial is None or closed
            if ser is None or not ser.is_open:
                port = find_gps_port()
                if not port:
                    print("[INFO] ⏳ Waiting for GPS device...")
                    time.sleep(0.1)
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
                        6: "Fusion"
                    }.get(fix_quality, "GPS Weak")

                elif gps_data.startswith("$GPVTG") and len(gps_data_split) > 7 and gps_data_split[7]:
                    speed = float(gps_data_split[7])

                if lat and lon and heading and sat_count and rtk_status and speed is not None:
                    return lat, lon, heading, sat_count, rtk_status, speed

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
