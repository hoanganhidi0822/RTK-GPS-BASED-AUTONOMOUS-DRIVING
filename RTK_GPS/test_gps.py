from GPS_module import *
gps_ser = connect_to_serial("/dev/ttyUSB0", 115200)
while 1:
    lat, lon, car_heading, sat_count, rtk_status = get_gps_data(gps_ser)
    print(lat, lon)


