from GPS_module import *
gps_ser = connect_to_serial("/dev/ttyUSB0", 115200)
while 1:
    lat, lon, heading, sat_count, rtk_status, speed = get_gps_data(gps_ser)
    print(f"heading {heading}, rtk_status {rtk_status}, speed {speed}")


