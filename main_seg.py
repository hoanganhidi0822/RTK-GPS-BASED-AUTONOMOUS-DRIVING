
import math
import time
import numpy as np
import threading
import config as cf
from RTK_GPS.GPS_module import *
from HD_MAP.HDMAP import *
from CONTROLLER.utils.communication import STM32
from OPTIMAL_TRAJECTORY.frenet_optimal_trajectory import *
camera_index = 0
from OBSTACLES.obstacle_seg import process_depth
import sys
from VISUALIZATION.visualization import *
from VOICE.voice import *
import simpleaudio as sa

# --------------- UART ------------------------- #
gps_ser = connect_to_serial("/dev/ttyUSB0", 115200)
stm32 = STM32(port="/dev/ttyUSB1", baudrate=115200)
# gps_ser = 1

collision_sound = sa.WaveObject.from_wave_file("VISUALIZATION/sound/forward_collision_warning.wav")
destination_sound = sa.WaveObject.from_wave_file("VISUALIZATION/sound/reach.wav")

# ---------------- Config init ------------------ #
cf.latitude = None
cf.longitude = None
cf.heading = None

cf.image = np.zeros((480, 1280, 3))
cf.obstacles = []

cf.tx                 = []
cf.ty                 = []
cf.tyaw               = []
cf.ob                 = []
cf.paths              = []
cf.optimal_path       = []
cf.x                  = 0
cf.y                  = 0
cf.yaw                = 0 
cf.steering_angle     = 0 
cf.lookahead_distance = (0,0) 
cf.lookahead_point    = (0,0) 
cf.projected_point    = (0,0) 
cf.car_speed          = 0 
cf.car_steer          = 0 
cf.area               = 20
cf.MAX_ROAD_WIDTH     = 6
cf.record = 1
cf.rtk_status = "No Fix"
cf.persons = []
cf.speed = 0
cf.camera_error = 0
cf.seg_mode = 0
cf.seg_steer = 0
cf.is_intersection = 0

def update_vis(x,y,yaw,steering_angle,paths,optimal_path, tx, ty,tyaw,ob,gps_speed):
    cf.tx                 = tx
    cf.ty                 = ty
    cf.tyaw               = tyaw
    cf.ob                 = ob
    cf.paths              = paths
    cf.optimal_path       = optimal_path
    cf.x                  = x
    cf.y                  = y
    cf.yaw                = yaw
    cf.steering_angle     = steering_angle 
    cf.lookahead_distance = (0,0) 
    cf.lookahead_point    = (0,0) 
    cf.projected_point    = (0,0) 
    cf.car_speed          = 0 
    cf.car_steer          = 0 
    cf.area               = 20
    cf.MAX_ROAD_WIDTH     = 6
    cf.speed = gps_speed

# ---------- Car State ------------ #
def update_state(ser):
   
    # ------------- GPS ------------- #
    lat, lon, car_heading, sat_count, rtk_status, speed = get_gps_data(ser)
    # rtk_status = "Single"
    # print(f"lat {lat}, lon {lon}, heading {car_heading}")
    # lat, lon, car_heading, rtk_status, speed = 10.8531900250,106.7714968267,185, "RTK Fixed", 15
    # time.sleep(0.1)
    
    
    # -------- Convert data to X Y frame --------- #
    x, y = lat_lon_to_xy(float(lat), float(lon))
    yaw_c = np.deg2rad(convert_yaw(float(car_heading), yaw_offset=90))
    
    return lat, lon, rtk_status, speed, x, y, yaw_c 

# ------ Depth_Obstacle_Position_Estimation ------- #
def depth_thread():
    process_depth()

def run_visualization():
    app = QApplication(sys.argv)
    window = AutonomousCarUI()
    window.show()
    sys.exit(app.exec_())

def slow_down_speed(distance, max_speed):
    """
    Function to slow down the car speed based on the distance to obstacle
    :param distance: The distance from the car to the obstacle
    :param max_speed: The maximum speed of the car
    :return: The speed of the car
    """
    return int(max_speed * (1 / (1 + math.exp(5 - 1 * distance))))  # 5 và 1 là các hệ số điều chỉnh

def get_target_direction(optimal_path, n_points=10):
    if len(optimal_path.x) < n_points:
        return None
    dx = optimal_path.x[n_points - 1] - optimal_path.x[0]
    dy = optimal_path.y[n_points - 1] - optimal_path.y[0]
    angle = np.arctan2(dy, dx)
    return angle  # hướng rẽ tính theo radian

def classify_turn(angle, threshold=7):
    if angle > threshold:
        return "right"
    elif angle < -threshold:
        return "left"
    else:
        return "straight"

# Ngưỡng vùng an toàn phía trước xe
SAFETY_ZONE_X = 3 
SAFETY_ZONE_Z = 5.0  

def is_in_safety_zone(x, z):
    return abs(x) <= SAFETY_ZONE_X and 0 <= z <= SAFETY_ZONE_Z

def main():
    print(__file__ + " start!!")
    cf.record = 1
    fps = 0
    time_start = 0

    # Initial GPS state
    lat, lon, car_heading = 10.85257737,106.7714248783, 185
    x, y = 16.993362287481073, 152.51032455731195
    wx, wy = [],[]

    # Obstacles
    danger_zone = 2.0  # mét

    obstacles = []
    persons = []
    new_persons = []
    new_obstacles = []
    ob = np.array([[0, 0]])
    obs = [[999, 999]]

    # Initial state
    c_speed = 0                     # current speed [m/s]
    c_d     = 0.0                   # current lateral position [m]
    c_d_d   = 0.0                   # current lateral speed [m/s]
    c_d_dd  = 0.0                   # current lateral acceleration [m/s]
    s0      = 0.0                   # current course position
     
    yaw = np.deg2rad(85)            # Convert yaw to radians
    car_speed = 2.8                 # Car speed [m/s]
    car_steer = 0                   # Car steering angle [degree]
 
    # Tracking Algorithm Parameters 
    lookahead_distance = 4.5        # Lookahead distance [m]
    L = 1.8                         # Wheelbase of the vehicle [m]
    count = 0
    
    person_detected_time = None
    stop_triggered = False
    resume_timer = None  # Thời điểm bắt đầu đếm để chạy lại

    # RTK‑status stop/resume timers
    rtk_bad = False
    rtk_bad_start = None
    rtk_resume_start = None

    stm32(angle=int(0), speed=int(0), brake_state=0)

    # Wait Assistance
    while cf.record:
        continue

    # -- Wait GPS --- #
    while True:
        
        lat, lon, rtk_status, gps_speed, x, y, yaw = update_state(gps_ser)
        try:
            lat = float(lat)         
            lon = float(lon)
        
            if not math.isnan(lat):  
                break
        except ValueError:
            pass  

    # Start thread for depth processing
    
    depth_thread_instance = threading.Thread(target=depth_thread)
    depth_thread_instance.daemon = True
    depth_thread_instance.start()

    # --- Khởi tạo thread phát âm thanh --- #
    audio_thread = threading.Thread(target=area_audio_thread_func, args=(lat, lon))
    audio_thread.daemon = True
    audio_thread.start()

    # Read Waypoints
    wx, wy = XY_WAYPOINTS_MAP(input_file)
    # Generate target course 
    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)

    count_none = 0
    target_speed = 9
    speed_filtered = 1
    alpha_speed = 0.93

    steering_filtered = 0  # Đặt ở đầu chương trình, ngoài vòng lặp
    alpha_steering = 0.7  # Hệ số lọc (gần 1: chậm phản ứng; gần 0: nhanh)
    prev_gps_time = 0
    max_delta_speed =0
    send_zero_speed = False
    zero_speed_sent = False
    seg_mode_start_time = None
    turn_type = 'straight'
    still_turning = False
    is_intersection = 0
    log_count = 0
    # --- Main Loop --- #
    while True:
        log_count += 1
        # Update vehicle state using RTK GPS
        lat, lon, rtk_status, gps_speed, x, y, yaw = update_state(gps_ser)
        cf.rtk_status = rtk_status
        cf.latitude   = lat
        cf.longitude  = lon
        if x == None:
            continue

        # --------------------   Update person state  ------------------- #
        new_persons = cf.persons
        # Chỉ cập nhật nếu có dữ liệu mới hợp lệ
        if len(new_persons): 
            persons = []
            persons = new_persons
            new_persons = []

        # --------------------   Update obstacle state  ------------------- #
        new_obstacles =  cf.obstacles
        # Chỉ cập nhật nếu có dữ liệu mới hợp lệ
        if len(new_obstacles): 
            obstacles = []
            obstacles = new_obstacles
            new_obstacles = []
            
        # obstacles = [[-8,12]]
        for obstacle in obstacles:
            obs.append(transform_obstacle_to_global(x, y, yaw, obstacle[1], obstacle[0]))

        ob = np.array(obs)
        # ------------------------- Generate optimal path ------------------------- #
        optimal_path, paths = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)

        # ------------------------- Optimal Path None => Replan
        while optimal_path is None:          
            print("optimal_path is None !!!")
            lat, lon, rtk_status, gps_speed, x, y, yaw = update_state(gps_ser)
   
            new_obstacles =  cf.obstacles
            # Chỉ cập nhật nếu có dữ liệu mới hợp lệ
            if len(new_obstacles): 
                obstacles = []
                obstacles = new_obstacles
                new_obstacles = []
                
            # obstacles = [[-8,12]]
            for obstacle in obstacles:
                obs.append(transform_obstacle_to_global(x, y, yaw, obstacle[1], obstacle[0]))

            ob = np.array(obs)
            s0, c_d, c_d_d, c_d_dd = cartesian_to_frenet(x, y, yaw, csp)
            optimal_path, paths = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)
            # c_d_d = optimal_path.d_d[1]
            # c_d_dd = optimal_path.d_dd[1]
            # c_speed = car_speed    

            count_none += 1
            if count_none == 3:
                pass
                # stm32(angle= int(-5), speed=0, brake_state=0)
            elif count_none > 20:
                print("Replanning failed too many times — entering safe mode.")
                stm32(angle=steering_filtered, speed=1, brake_state=1)
                # break  # hoặc flag lại để tự quay lại vòng điều khiển khác
                obstacles = []
                obs = [] 
                count_none = 0

        #####----------------------------------------------------------------------------------------------------------------------------#####
        if x is not None:
            # Pure Pursuit control: use the optimal path from Frenet
            car_steer,  alpha_pure_pursuit = pure_pursuit_control_frenet(float(lat), float(lon), optimal_path, x, y,yaw, lookahead_distance, L)
            
            s0 , c_d, c_d_d, c_d_dd = cartesian_to_frenet(x, y, yaw, csp)
            c_d_d = optimal_path.d_d[1]
            c_d_dd = optimal_path.d_dd[1]
            c_speed = car_speed    

            ################## SPEED CONTROL ##################################################################################################
            # Giảm tốc độ khi vào cua
            try:
                i = min(5, len(optimal_path.x) - 3)
                dx = optimal_path.x[i+1] - optimal_path.x[i]
                dy = optimal_path.y[i+1] - optimal_path.y[i]
                ddx = optimal_path.x[i+2] - 2 * optimal_path.x[i+1] + optimal_path.x[i]
                ddy = optimal_path.y[i+2] - 2 * optimal_path.y[i+1] + optimal_path.y[i]

                numerator = abs(dx * ddy - dy * ddx)
                denominator = (dx**2 + dy**2)**1.5 + 1e-6  # tránh chia 0
                curvature = numerator / denominator

                # print(f"Curvature: {curvature}")
                if curvature > 0.1:
                    target_speed = min(target_speed, 5)

            except Exception as e:
                print("[WARNING] Curvature calc failed:", e)

            danger_zone = 1.5  # mét

            found_person = False
            for person in persons:
                if abs(person[0]) < danger_zone and abs(person[1]) < 4.0:
                    found_person = True
                    break

            current_time = time.time()

            # Phát hiện người lần đầu
            if found_person and person_detected_time is None:
                person_detected_time = current_time


            # Không còn người trong vùng nguy hiểm
            elif not found_person:
                person_detected_time = None
                if stop_triggered and resume_timer is None:
                    resume_timer = current_time  # Bắt đầu đếm để chạy lại

            # Người vẫn còn sau 1.5 giây => dừng xe
            elif found_person and not stop_triggered and (current_time - person_detected_time) > 0.01:
                stm32(angle=int(steering_filtered), speed=int(1), brake_state=0)

                threading.Thread(target=lambda: collision_sound.play()).start()
                speed_filtered = 1
                target_speed = 1
                stop_triggered = True
                print("\n[ALERT] Người xuất hiện liên tục trong 0.1s – Dừng xe!")

            # Nếu người đã rời đi và đủ thời gian (3 giây) thì cho xe chạy lại
            if stop_triggered and resume_timer is not None and (current_time - resume_timer) >= 1.5:
                stop_triggered = False
                resume_timer = None
                speed_filtered = 4
                target_speed = 4
                print("\n[INFO] Vùng an toàn. Cho xe chạy lại.")

            if 'prev_gps_speed' not in globals():
                prev_gps_speed = gps_speed

            delta_speed = gps_speed - prev_gps_speed
            prev_gps_speed = gps_speed

            # --------- PHÁT HIỆN GIẢM TỐC ĐỘ BẤT THƯỜNG (LỖI PHẦN CỨNG) --------- ###############
            if gps_speed < 1.0 and delta_speed < -0.5:
                send_zero_speed = True
                zero_speed_sent = False

            # ----- TANG TOC DOT NGOT ------####################################################
            if gps_speed < 2 and delta_speed > 0.5:
                speed_filtered = 2

            ######################################################################################
            # --------- GỬI target_speed = 0 ĐỂ RESET MẠCH --------- #
            if send_zero_speed and not zero_speed_sent:
                target_speed = 0
                zero_speed_sent = True
                return target_speed  # Gửi 0 rồi kết thúc sớm

            # --------- TIẾP TỤC CHU TRÌNH SAU KHI ĐÃ GỬI 0 --------- #
            if send_zero_speed and zero_speed_sent:
                send_zero_speed = False  

            #######################################################################################
            # --------- GIAO ĐỘNG TỐC ĐỘ DỰA TRÊN GPS SPEED --------- #
            if gps_speed >= 10:
                target_speed = 4  # Giảm tốc
            elif gps_speed < 8:
                target_speed = 10  # Tăng tốc
            else:
                target_speed = 9  # Duy trì tốc độ ổn định

            # Giới hạn trên
            target_speed = min(target_speed, 10)

            # --------- GIẢM TỐC KHI GẦN ĐÍCH --------- #
            distance_to_goal = np.hypot(tx[-1] - x, ty[-1] - y)

            if distance_to_goal < 8:
                target_speed = min(target_speed, slow_down_speed(distance_to_goal, 8))


            hazard_detected = any(is_in_safety_zone(x, z) for (x, z) in obstacles + persons)


            # --------- GIẢM TỐC KHI CÓ VẬT CẢN --------- #
            if hazard_detected:
                if gps_speed > 7.0:
                    target_speed = 5  # từ từ giảm
                elif gps_speed < 6.0:
                    target_speed = 8  # tăng nhẹ để đạt ~7
                else:
                    target_speed = 7  # đã ổn định
                speed_filtered = target_speed

            # Reset sau khi xử lý
            obstacles = []
            persons = []

            #################### --- STOP --- ###################################
            should_stop = False
            seg_mode = False  # dùng segmentation để điều khiển khi mất GPS
            # 1. Camera lỗi
            if cf.camera_error == 1:
                print("[WARNING] Camera error – Dừng xe!")
                should_stop = True

            # 2. Phát hiện người trong vùng nguy hiểm (đã được xử lý ở phần trước)
            if stop_triggered:
                gps_speed = "SAFE MODE ACTIVATED – A person has been detected. Please monitor the vehicle!"
                print("[INFO] Người trong vùng nguy hiểm – Đang dừng xe.")
                should_stop = True

            # 3) RTK‑status hysteresis
            if cf.rtk_status != "RTK Fixed":

                if not rtk_bad:
                    rtk_bad = True
                    rtk_bad_start = current_time
                    rtk_resume_start = None
                    print("[WARNING] RTK mất Fixed – Dừng xe ngay!")
                gps_speed = "SAFE MODE ACTIVATED – GPS SIGNAL IS WEAK. Please monitor the vehicle!"
                seg_mode = True 

            else:  
                if rtk_bad:
                    # start counting “good” time
                    if rtk_resume_start is None:
                        rtk_resume_start = current_time
                    # if fixed long enough, clear the bad flag
                    elif (current_time - rtk_resume_start) >= 3.0:
                        print("[INFO] RTK Fixed liên tục >3s – Cho chạy lại.")
                        rtk_bad = False
                        rtk_bad_start = None
                        rtk_resume_start = None
                        seg_mode = False  
                    # if still in bad state, keep should_stop True
                    else:
                        gps_speed = "SAFE MODE ACTIVATED – GPS SIGNAL IS WEAK. Please monitor the vehicle!"
                        seg_mode = True
                else:
                    seg_mode = False

            cf.seg_mode = seg_mode

            ########## --- CONTROL COMMAND  ---- #############
 
            # Góc điều khiển ban đầu từ GPS
            steering_angle = car_steer

            # Ghi lại thời gian bắt đầu chế độ segmentation
            if seg_mode and seg_mode_start_time is None:
                seg_mode_start_time = current_time
                speed_filtered = 6
   
            is_intersection = cf.is_intersection
                
            # Cập nhật trạng thái còn đang rẽ
            if is_intersection:
                still_turning = True
            else:  # thêm độ trễ khi thoát cua
                still_turning = False


            # Thời gian đệm trước khi dùng seg_steer (giây)
            seg_delay_cua = 6.0
            seg_delay_thang = 1

            if should_stop:
                target_speed = 1
                speed_filtered = 1
                steering_angle = car_steer

            elif seg_mode:
                delay = seg_delay_cua if still_turning else seg_delay_thang
                alpha_steering = 0.8 if still_turning else 0.6

                if current_time - seg_mode_start_time < delay  or still_turning:
                    steering_angle = car_steer   
                else:
                    steering_angle = cf.seg_steer

                target_speed = 7

            else:
                seg_mode_start_time = None  # reset nếu trở lại chế độ bình thường
                steering_angle = car_steer
                alpha_steering = 0.7

            # Low-pass filter: steering
            steering_filtered = alpha_steering * steering_filtered + (1 - alpha_steering) * steering_angle

            # Low-pass filter: target speed
            speed_filtered = alpha_speed * speed_filtered + (1 - alpha_speed) * target_speed

            count += 1
            if count == 1:
                stm32(angle=int(steering_filtered), speed=int(speed_filtered), brake_state=0)
                count = 0

        ####-----------------------------------------------------------------------------------------------------------#### 
            persons = [] 
        # Check if the goal is reached
        if np.isclose(x, tx[-1], atol = 2.5) and np.isclose(y, ty[-1], atol = 2.5):
            gps_speed = "Goal reached!"
            cf.camera_error = 1
            stm32(angle=0, speed=1, brake_state=1) 
            print("Goal reached!")
            play__ = destination_sound.play() 
            play__.wait_done()
            stm32(angle=steering_filtered, speed=0, brake_state=1) 
            return
        cf.seg_steer = steering_angle
        obstacles = []   
        persons = [] 

        ############## FPS ##########################################################
        alpha = 0.8
        delta_t = time.time() - time_start
        if delta_t > 0:
            fps = (1 - alpha) * fps + alpha * (1 / delta_t)
        time_start = time.time()

        ############# DEBUG #########################################################
        if log_count == 10:
            print(f"\r[INFO] Dir: {turn_type}, Steer: {car_steer} Seg Mode: {seg_mode}, Seg Steer: {cf.seg_steer},GPS Speed: {gps_speed},Target Speed: {round(speed_filtered)} FPS: {fps}", end=" ")
            log_count = 0
        ############--- VISUALIZATION ---############################################
        # right before your update_vis() call:
        if isinstance(gps_speed, (int, float)):
            vis_speed = round(gps_speed * 1.5,1)
        else:
            vis_speed = gps_speed

        # now call update_vis with the right argument:
        update_vis(x, y, yaw,
                    car_steer, paths, optimal_path,
                    tx, ty, tyaw, ob,
                    vis_speed)
        
        obs = []
    
if __name__ == '__main__':
    vis_process = threading.Thread(target=run_visualization)
    vis_process.daemon = True
    vis_process.start()

    while True:    
        main()
    
          