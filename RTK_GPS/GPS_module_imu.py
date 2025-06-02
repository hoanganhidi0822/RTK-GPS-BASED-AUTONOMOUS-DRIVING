import serial
import time
import config as cf

cf.latitude = None
cf.longitude = None
cf.heading = None
def dmm_to_deg(dmm, direction):
    """
    Chuyển định dạng DMM (Degrees Minutes) sang Decimal Degrees.
    Ví dụ: 1051.1900053,N --> 10.85316676
    """
    dmm = float(dmm)
    degrees = int(dmm / 100)
    minutes = dmm - (degrees * 100)
    decimal_degrees = degrees + (minutes / 60.0)
    
    if direction in ['S', 'W']:
        decimal_degrees = -decimal_degrees
    return decimal_degrees
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
    """Handles the connection to the serial port."""
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
        # gps_data = ser.readline().decode("utf8").strip()
        try:
            gps_data = ser.readline ().decode("utf8").strip()
        except UnicodeDecodeError:
            continue
        if gps_data:
            try:
                gps_data_split = gps_data.split(',')
                
                # if gps_data.startswith("$GPHDT"):
                #     heading = float(gps_data_split[1])
                
                if gps_data.startswith("$GPYBM"):
                    if len(gps_data_split) > 6 and gps_data_split[6]:
                        heading = float(gps_data_split[6])
                
                elif gps_data.startswith("$GPGGA"):
                    if len(gps_data_split) > 9:
                        lat = dec2deg(float(gps_data_split[2])) if gps_data_split[2] else None
                        lon = dec2deg(float(gps_data_split[4])) if gps_data_split[4] else None
                        sat_count = int(gps_data_split[7]) if gps_data_split[7] else None
                        fix_quality = int(gps_data_split[6]) if gps_data_split[6] else 0

                        # Determine RTK status
                        if fix_quality == 4:
                            rtk_status = "RTK Fixed"
                        elif fix_quality == 5:
                            rtk_status = "RTK Float"
                        elif fix_quality == 2:
                            rtk_status = "DGPS"
                        elif fix_quality == 1:
                            rtk_status = "Single"
                        elif fix_quality == 6:
                            rtk_status = "Fusion"
                        else:
                            rtk_status = "GPS Weak"

                elif gps_data.startswith("$GPVTG"):
                    if len(gps_data_split) > 7 and gps_data_split[7]:
                        speed = float(gps_data_split[7])  # Speed in km/h

                # Only return when all values are available
                if lat and lon and heading and sat_count and rtk_status and speed is not None:
                    return lat, lon, heading, sat_count, rtk_status, speed

            except Exception as e:
                print("Error processing GPS data:", e)

def get_gps_data_for_dead_reckoning(ser):
    """Reads fused GNSS+IMU data for dead reckoning from the serial port."""
    lat = lon = heading = pitch = roll = None
    vN = vE = vU = 0.0
    sat_count = ins_status = rtk_status = None

    while True:
        try:
            gps_data = ser.readline().decode("utf8").strip()
        except UnicodeDecodeError:
            continue

        if gps_data:
            parts = gps_data.split(',')

            try:
                # Parse from $GPYBM — full fused GNSS + IMU
                if gps_data.startswith("$GPYBM") and len(parts) >= 20:
                    # Example: $GPYBM,SN...,TIME,LAT,LON,ALT,HEAD,PITCH,ROLL,vN,vE,vU,...
                    lat = float(parts[3])
                    lon = float(parts[4])
                    heading = float(parts[5])
                    pitch = float(parts[6])
                    roll = float(parts[7])
                    vN = float(parts[8])
                    vE = float(parts[9])
                    vU = float(parts[10])
                    sat_count = int(parts[18]) if parts[18].isdigit() else None
                    ins_status = int(parts[17]) if parts[17].isdigit() else None
                    # RTK status từ phần fix trong GGA hoặc INS_STATE
                    rtk_status = {
                        4: "RTK Fixed",
                        5: "RTK Float",
                        2: "DGPS",
                        1: "Single",
                        6: "Fusion",
                    }.get(ins_status, "Unknown")

                # Nếu thiếu heading, lấy từ GPHDT
                elif gps_data.startswith("$GPHDT") and len(parts) > 1:
                    heading = float(parts[1])

                # Nếu thiếu lat/lon, lấy từ GPGGA
                elif gps_data.startswith("$GPGGA") and len(parts) > 6:
                    if not lat and parts[2] and parts[3]:
                        lat = dmm_to_deg(float(parts[2]), parts[3])
                    if not lon and parts[4] and parts[5]:
                        lon = dmm_to_deg(float(parts[4]), parts[5])
                    if not sat_count and parts[7]:
                        sat_count = int(parts[7])
                    fix_quality = int(parts[6]) if parts[6] else 0
                    if not rtk_status:
                        rtk_status = {
                            4: "RTK Fixed",
                            5: "RTK Float",
                            2: "DGPS",
                            1: "Single",
                            6: "Fusion",
                        }.get(fix_quality, "GPS Weak")

                # Khi đủ dữ liệu thì return
                if lat is not None and lon is not None and heading is not None:
                    return {
                        "lat": lat,
                        "lon": lon,
                        "heading": heading,
                        "pitch": pitch,
                        "roll": roll,
                        "vN": vN,
                        "vE": vE,
                        "vU": vU,
                        "sat_count": sat_count,
                        "rtk_status": rtk_status,
                        "ins_status": ins_status
                    }

            except Exception as e:
                print("Error parsing line:", e)
