import os
import threading
import matplotlib.pyplot as plt
import time
from Assistance_Astar import islab_assistance_v2
from Assistance_Astar import main_atar_gps as astar
import asyncio
import config as cf

event = threading.Event()  # Tạo event để kiểm soát luồng chạy

cf.cf_destination = "none"

def task1():
    asyncio.set_event_loop(asyncio.new_event_loop())
    car_assistance = islab_assistance_v2.virtual_assistance()
    
    is_run = car_assistance.run()
    if is_run:
        event.set()  # Bật event để Task 2 chạy

def task2(destination = 0):

    if destination:
        astar.run_map(destination)
    else:
        event.wait()  # Chờ Task 1 hoàn thành
        print(f"destination: {cf.cf_destination}")
        astar.run_map(cf.cf_destination)

# # Chạy Task 1 trong luồng riêng
# t1 = threading.Thread(target=task1)
# t1.start()

# # Chạy Task 2 trong luồng riêng
# # t2 = threading.Thread(target=task2)
# # t2.start()

# # Đợi Task 1 và Task 2 hoàn thành
# t1.join()
# # t2.join()
# task2()

# # Chạy plt.show() trong luồng chính
# plt.show()
