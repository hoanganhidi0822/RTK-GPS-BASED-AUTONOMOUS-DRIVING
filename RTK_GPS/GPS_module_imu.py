import serial
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

def wait_for_com_port(port):
    while True:
        try:
            ser = serial.Serial(port, baudrate=115200, timeout=0.1)
            ser.close()
            return
        except serial.SerialException:
            print(f"Port {port} not available yet, waiting...")
            time.sleep(1)

def connect_to_serial(port, baudrate=115200, timeout=0.1):
    wait_for_com_port(port)
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        return ser
    except serial.SerialException as e:
        print(f"Failed to connect to {port}: {e}")
        return None

def get_gps_data(ser):
    """Reads GPS data from an already connected serial port."""
    lat = lon = sat_count = heading = speed = None
    rtk_status = None

    while True:
        try:
            gps_data = ser.readline().decode("utf8", errors="ignore").strip()
            if not gps_data:
                continue

            gps_data_split = gps_data.split(',')

            if gps_data.startswith("$GPYBM"):
                if len(gps_data_split) > 6 and gps_data_split[6]:
                    heading = float(gps_data_split[6])

            elif gps_data.startswith("$GPGGA"):
                if len(gps_data_split) > 9:
                    lat = dec2deg(float(gps_data_split[2])) if gps_data_split[2] else None
                    lon = dec2deg(float(gps_data_split[4])) if gps_data_split[4] else None
                    sat_count = int(gps_data_split[7]) if gps_data_split[7] else None
                    fix_quality = int(gps_data_split[6]) if gps_data_split[6] else 0

                    # Map fix_quality to RTK status
                    rtk_status_map = {
                        1: "Single",
                        2: "DGPS",
                        4: "RTK Fixed",
                        5: "RTK Float",
                        6: "GNSS+INS"  # bổ sung trạng thái này
                    }
                    rtk_status = rtk_status_map.get(fix_quality, "GPS Weak")

            elif gps_data.startswith("$GPVTG"):
                if len(gps_data_split) > 7 and gps_data_split[7]:
                    try:
                        speed = float(gps_data_split[7])
                    except ValueError:
                        speed = None

            # Chỉ return khi tất cả dữ liệu đã có
            if all([lat, lon, heading is not None, sat_count, rtk_status, speed is not None]):
                cf.latitude = lat
                cf.longitude = lon
                cf.heading = heading
                return lat, lon, heading, sat_count, rtk_status, speed

        except Exception as e:
            print("Error processing GPS data:", e)
