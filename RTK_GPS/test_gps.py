from GPS_module_imu import *
gps_ser = connect_to_serial("/dev/ttyUSB1", 115200)
while True:
    data = get_gps_data_for_dead_reckoning(gps_ser)
#    
    # print("Heading:", data["heading"])
    if data["ins_status"] is not None:
        print("INS Status:", data["ins_status"])

