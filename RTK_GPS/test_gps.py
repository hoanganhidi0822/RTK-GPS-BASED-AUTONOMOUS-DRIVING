from GPS_module_imu import *
gps_ser = connect_to_serial("/dev/ttyUSB0", 115200)
while True:
    lat, lon, heading, sat_count, rtk_status, speed, age = get_gps_data(gps_ser)
#    
    print(f"Heading: {heading}, status: {rtk_status}, age: {age}")
    # if data["ins_status"] is not None:
    #     print("INS Status:", data["rtk_status"])

