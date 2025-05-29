from GPS_module import *
gps_ser = connect_to_serial("/dev/ttyUSB1", 115200)
while 1:
    data = get_gps_data(gps_ser)
    print(data)


