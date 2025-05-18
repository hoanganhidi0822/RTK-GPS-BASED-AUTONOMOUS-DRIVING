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
        gps_data = ser.readline().decode("utf8").strip()
        if gps_data:
            try:
                gps_data_split = gps_data.split(',')
                
                if gps_data.startswith("$GPHDT"):
                    heading = float(gps_data_split[1])
                
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