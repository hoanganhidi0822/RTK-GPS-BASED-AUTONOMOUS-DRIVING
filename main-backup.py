"""

Frenet optimal trajectory generator

author: Atsushi Sakai (@Atsushi_twi)

Ref:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame](https://www.youtube.com/watch?v=Cj6tAQe7UCY)

"""

import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import CUBIC_PLANNER.cubic_spline_planner as cubic_spline_planner
import RTK_GPS.GPS_module as GPS_module
from utils.communication import STM32
import time
from OBSTACLES.obstacle import process_depth
import matplotlib.patches as patches
import cv2

stm32 = STM32(port="COM16", baudrate=115200)

# Parameter
MAX_SPEED = 15.0 / 3.6    # maximum speed [m/s]
MAX_ACCEL = 2             # maximum acceleration [m/ss]
MAX_CURVATURE = 30        # maximum curvature (độ cong) [1/m]
MAX_ROAD_WIDTH = 7        # maximum road width [m]
D_ROAD_W = 0.3            # road width sampling length [m]
DT = 0.3                  # time tick [s]
MAXT = 5.0                # max prediction time [m]
MINT = 4.0                # min prediction time [m]
TARGET_SPEED = 8 / 3.6    # target speed [m/s]
D_T_S = 0.1 / 3.6         # target speed sampling length [m/s]
N_S_SAMPLE = 1            # sampling number of target speed
ROBOT_RADIUS = 2.0        # robot radius [m]

# cost weights Trajectory
KJ = 0.1
KT = 0.1
KD = 1.0
KLAT = 2.0
KLON = 1.0

lat, lon, sat_count, alt = None, None, None, None
# Base reference for conversion
base_latitude  = 10.8532570333 # Latitude of the first waypoint
base_longitude = 106.7715131967  # Longitude of the first waypoint
scaling_factor = 100000

# PID control parameters
pre_t = time.time()
error_arr = np.zeros(5)
brake = 0
def PID(error, p, i, d):
    global pre_t, error_arr 
    # Shift and store error history
    error_arr[1:] = error_arr[:-1]
    error_arr[0] = error 
    # Calculate Proportional term
    P = error * p
    # Calculate delta time
    delta_t = time.time() - pre_t
    pre_t = time.time() 
    # Calculate Integral term
    I = np.sum(error_arr) * delta_t * i
    # Calculate Derivative term (if error_arr[1] exists)
    if delta_t > 0:
        D = (error - error_arr[1]) / delta_t * d
    else:
        D = 0
    # Compute the total PID output
    angle = P + I + D
    # Apply output limit
    if abs(angle) > 30:
        angle = np.sign(angle) * 30
    
    return float(angle)

show_animation = True

class quintic_polynomial:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):

        # calc coefficient of quintic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[T**3, T**4, T**5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T**2,
                      vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5
        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4
        return xt
    
    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3
        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t**2
        return xt


class quartic_polynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, T):

        # calc coefficient of quintic polynomial
        self.xs = xs   # Vị trí ban đầu.
        self.vxs = vxs # Vận tốc ban đầu.
        self.axs = axs # Gia tốc ban đầu.
        self.vxe = vxe # Vận tốc tại thời điểm kết thúc.
        self.axe = axe # Gia tốc tại thời điểm kết thúc.
                       # T: Thời gian di chuyển từ điểm đầu đến điểm cuối.

        self.a0 = xs        # Giá trị tại thời điểm ban đầu.
        self.a1 = vxs       # Hệ số của vận tốc ban đầu.
        self.a2 = axs / 2.0 # Hệ số gia tốc ban đầu (gia tốc chia 2 để phù hợp với công thức đa thức).

        A = np.array([[3 * T ** 2,  4 * T ** 3],
                      [  6 * T   , 12 * T ** 2]])       # Ma trận A là ma trận hệ số của phương trình
        
        b = np.array([vxe - self.a1 - 2 * self.a2 * T,
                             axe - 2 * self.a2        ]) # Vector b chứa các giá trị đích
        
        x = np.linalg.solve(A, b) # np.linalg.solve(A, b) giải hệ phương trình để tìm giá trị của a3 và a4.

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        """Tính giá trị vị trí x(t) của đa thức tại thời điểm t."""
        
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4

        return xt

    def calc_first_derivative(self, t):
        """Tính vận tốc v(t), là đạo hàm bậc nhất của x(t)"""
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3
        return xt

    def calc_second_derivative(self, t):
        """Tính gia tốc a(t), là đạo hàm bậc hai của x(t)"""
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2
        return xt

    def calc_third_derivative(self, t):
        """Tính giật j(t) (jerk), là đạo hàm bậc ba của x(t)"""
        xt = 6 * self.a3 + 24 * self.a4 * t
        return xt

class Frenet_path:
    def __init__(self):
        self.t = []     # Danh sách thời gian tương ứng với các điểm trên quỹ đạo.
        self.d = []     # Danh sách giá trị độ lệch ngang (lateral offset) tại mỗi thời điểm t. 
                        # Giá trị này đại diện cho khoảng cách từ quỹ đạo tham chiếu đến quỹ đạo thực tế.
        self.d_d = []   # Vận tốc lệch ngang (lateral velocity) tại mỗi điểm trên quỹ đạo.
        self.d_dd = []  # Gia tốc lệch ngang (lateral acceleration).
        self.d_ddd = [] # Giật lệch ngang (lateral jerk), tức là đạo hàm bậc ba của d theo thời gian.
        
        self.s = []     # Danh sách giá trị dọc theo quỹ đạo tham chiếu tại mỗi thời điểm t. Đây là khoảng cách tích lũy từ điểm đầu quỹ đạo.
        self.s_d = []   # Vận tốc dọc theo quỹ đạo (longitudinal velocity)
        self.s_dd = []  # Gia tốc dọc (longitudinal acceleration)
        self.s_ddd = [] # Giật dọc (longitudinal jerk)
        
        #Cost
        self.cd = 0.0 # Cost liên quan đến chuyển động ngang (lateral motion cost)
        self.cv = 0.0 # Cost liên quan đến chuyển động dọc (longitudinal motion cost)
        self.cf = 0.0 # Tổng chi phí (total cost), được tính dựa trên cd và cv. 
                      # Giá trị này thường được sử dụng để so sánh và chọn quỹ đạo tối ưu

        # Các thông số trong không gian Cartesian (kết quả chuyển đổi từ Frenet)
        self.x = []   # Danh sách tọa độ x trong không gian Cartesian
        self.y = []   # Danh sách tọa độ y trong không gian Cartesian
        self.yaw = [] # Góc phương vị (yaw angle) tại mỗi điểm trong không gian Cartesian
        self.ds = []  # Khoảng cách giữa các điểm liên tiếp trên quỹ đạo trong không gian Cartesian
        self.c = []   # Độ cong (curvature) tại mỗi điểm trên quỹ đạo trong không gian Cartesian

def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0):
    # c_speed: Tốc độ hiện tại của xe dọc theo quỹ đạo tham chiếu (longitudinal speed)
    # c_d    : Độ lệch ngang hiện tại (lateral offset)
    # c_d_d  : Vận tốc lệch ngang hiện tại (lateral velocity)
    # c_d_dd : Gia tốc lệch ngang hiện tại (lateral acceleration)
    # s0     : Vị trí dọc hiện tại trên quỹ đạo tham chiếu (accumulated longitudinal position)
    frenet_paths = []

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH/2, MAX_ROAD_WIDTH/2, D_ROAD_W):
        # For each lateral offset (di) from -MAX_ROAD_WIDTH/2 to MAX_ROAD_WIDTH/2 (step D_ROAD_W)
        # Lề trái đến lề phải, path chính nằm ở tim đường
        # di là các giá trị độ lệch ngang mục tiêu trong phạm vi chiều rộng của đường.
        
        # Lập kế hoạch chuyển động ngang (Lateral motion planning)
        # For each time duration (Ti) from MINT to MAXT (step DT)
        for Ti in np.arange(MINT, MAXT, DT):
            # Ti là các giá trị thời gian (duration) để đạt được mục tiêu di.
            # Create a new Frenet path (fp)
            fp = Frenet_path()

            # Generate lateral trajectory using quintic polynomial
            #    S: start E:end  vitriS,vantocS,giatocS vitriE,vantocE,giatocE, time
            lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            
            # Lưu kết quả vào fp
            # Thời gian t: fp.t
            # print(Ti)
            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            
            # Calculate lateral parameters (fp.d, fp.d_d, fp.d_dd, fp.d_ddd)
            # Độ lệch ngang d: fp.d
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            # print(len(fp.d))
            
            # Vận tốc, gia tốc, và giật ngang: fp.d_d, fp.d_dd, fp.d_ddd
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]
            # print(f"D: {fp.d}")
            ## Lập kế hoạch chuyển động dọc (Longitudinal motion planning)
            # Mục tiêu: Tính toán các quỹ đạo để duy trì hoặc thay đổi tốc độ dọc đến các giá trị mục tiêu khác nhau tv.
            
            # Duyệt qua các giá trị tốc độ mục tiêu tv
            # Loongitudinal motion planning (Velocity keeping)
            # For each target velocity (tv) around TARGET_SPEED
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                # print(D_T_S)
                tfp = copy.deepcopy(fp) # Sao chép quỹ đạo ngang fp
                
                # Tính toán chuyển động dọc bằng phương trình bậc 4
                # print(s0)
                # Generate longitudinal trajectory using quartic polynomial
                # Sử dụng quartic polynomial để mô hình hóa chuyển động từ trạng thái hiện tại (s0,cspeed,0) đến trạng thái mục tiêu (tv,0).
                lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti) 

                # Calculate longitudinal parameters (tfp.s, tfp.s_d, tfp.s_dd, tfp.s_ddd)
                # Lưu kết quả vào tfp: Vị trí dọc, Vận tốc, gia tốc, và giật dọc:tfp.s, tfp.s_d, tfp.s_dd, tfp.s_ddd
                
                
                # print(f"S: {tfp.s}")
                tfp.s     = [ lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d   = [ lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd  = [ lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [ lon_qp.calc_third_derivative(t) for t in fp.t]

                # Compute costs (Jp, Js, ds) 
                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                # ds: Cost do chênh lệch tốc độ với tốc độ mục tiêu.
                ds = (TARGET_SPEED - tfp.s_d[-1])**2

                # Calculate total costs (tfp.cd, tfp.cv, tfp.cf)
                tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1]**2 # Cost do độ lệch ngang cuối cùng.
                tfp.cv = KJ * Js + KT * Ti + KD * ds 
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv

                # Append the trajectory (tfp) to the list
                frenet_paths.append(tfp)
    
    return frenet_paths


def calc_global_paths(fplist, csp):
    
    for fp in fplist:
        # calc global positions
        # Xử lý từng quỹ đạo Frenet để chuyển đổi sang hệ tọa độ toàn cục.
        for i in range(len(fp.s)):
            
            # Tính vị trí toàn cục (ix,iy) tại khoảng cách s[i] dọc theo đường tham chiếu.
            ix, iy = csp.calc_position(fp.s[i])
            
            if ix is None:
                # Nếu không tính được vị trí (ví dụ: ngoài phạm vi đường tham chiếu), thoát khỏi vòng lặp.
                break
            
            # Tính góc hướng (yaw) tại khoảng cách s[i]
            iyaw = csp.calc_yaw(fp.s[i])
            
            # Sử dụng độ lệch ngang di để tính vị trí toàn cục (fx,fy)
            di = fp.d[i]
            fx = ix + di * math.cos(iyaw + math.pi / 2.0)
            fy = iy + di * math.sin(iyaw + math.pi / 2.0)
            
            # Lưu kết quả vào
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        # Tính góc hướng và khoảng cách
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            
            # Sử dụng atan2 để tính góc hướng giữa hai điểm liên tiếp
            # Sử dụng công thức Pythagoras để tính độ dài đoạn thẳng ds giữa hai điểm liên tiếp.
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.sqrt(dx**2 + dy**2))
        
        # Lưu kết quả vào
        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        # Tính độ cong (curvature)
        for i in range(len(fp.yaw) - 1):
            
            # Độ cong: Tính độ thay đổi của góc hướng (yaw) giữa hai điểm liên tiếp chia cho khoảng cách ds
            # Độ cong này biểu diễn mức độ uốn cong của quỹ đạo tại mỗi điểm
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])
    # - Danh sách các quỹ đạo với thông tin toàn cục đầy đủ bao gồm:
    #    + Vị trí toàn cục (x,y).
    #    + Góc hướng yaw.
    #    + Khoảng cách ds.
    #    + Độ cong c.
    return fplist



def check_collision(fp, ob):
    # fp:Một đối tượng thuộc lớp Frenet_path chứa thông tin quỹ đạo, bao gồm danh sách các điểm tọa độ toàn cục (x,y).
    # ob:Mảng NumPy ob chứa tọa độ của các chướng ngại vật: Mỗi hàng đại diện cho một chướng ngại vật với (x,y).
    
    # Duyệt qua từng chướng ngại vật
    # Lặp qua tất cả các chướng ngại vật trong danh sách ob
    for i in range(len(ob[:, 0])):
        # Tính khoảng cách từ từng điểm trên quỹ đạo đến chướng ngại vật
        # Với mỗi chướng ngại vật ob[i]=(xob,yob), tính khoảng cách bình phương từ nó đến từng điểm (x,y) trên quỹ đạo fp:
        d = [((ix - ob[i, 0])**2 + (iy - ob[i, 1])**2)
             for (ix, iy) in zip(fp.x, fp.y)]

        # So sánh mỗi khoảng cách với bán kính va chạm
        collision = any([di <= ROBOT_RADIUS**2 for di in d])

        if collision:
            return False

    return True


def check_paths(fplist, ob):
    """Hàm check_paths được sử dụng để kiểm tra các quỹ đạo (paths)
    trong danh sách fplist dựa trên các ràng buộc động học và tránh va chạm. 
    Chỉ những quỹ đạo hợp lệ (thỏa mãn các điều kiện) mới được giữ lại."""

    # Danh sách các đối tượng thuộc lớp Frenet_path, 
    # mỗi đối tượng đại diện cho một quỹ đạo với các thuộc tính như vận tốc, gia tốc, độ cong, v.v.
    # ob: Mảng NumPy chứa tọa độ của các chướng ngại vật trong môi trường.

    # Mảng này lưu trữ chỉ số của các quỹ đạo hợp lệ trong fplist
    okind = []
    
    # Kiểm tra từng quỹ đạo
    for i in range(len(fplist)):
        
        # Kiểm tra điều kiện động học
        
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            # Giới hạn vận tốc tối đa
            # print("1")
            continue
        elif any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd]):  # Max accel check
            # Giới hạn độ cong tối đa
            # print("2")
            continue
        elif any([abs(c) > MAX_CURVATURE for c in fplist[i].c]):  # Max curvature check
            # Giới hạn độ cong tối đa
            # print("3")
            continue
        elif not check_collision(fplist[i], ob):
            # Nếu quỹ đạo va chạm với bất kỳ chướng ngại vật nào, bỏ qua quỹ đạo.
            # print("4")
            continue
        # Nếu bất kì điều kiện nào không thỏa thì bỏ qua path đó
        # Lưu các quỹ đạo hợp lệ thỏa mãn tất cả điều kiện trên, thêm chỉ số i vào danh sách okind.
        okind.append(i)
    # Trích xuất các quỹ đạo hợp lệ từ danh sách fplist dựa trên chỉ số trong okind
    return [fplist[i] for i in okind]



def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob):
    """ Hàm frenet_optimal_planning thực hiện quá trình lập kế hoạch chuyển động tối ưu trong hệ tọa độ Frenet, 
        dựa trên các thông tin đầu vào về trạng thái xe, đường cong tham chiếu, và các chướng ngại vật. 
        Hàm trả về quỹ đạo tối ưu dựa trên tiêu chí chi phí tối thiểu. """

    # - Các tham số đầu vào
    #     + csp: Đường cong tham chiếu (Cubic Spline Path) trong hệ tọa độ toàn cục.
    #     + Dùng để tính toán vị trí và hướng toàn cục từ tọa độ Frenet.

    #     + s0:Vị trí dọc (longitudinal) ban đầu của xe trên đường tham chiếu.
    #     + c_speed:Vận tốc dọc ban đầu của xe.

    #     + c_d, c_d_d, c_d_dd:
    #     + Các trạng thái ngang (lateral state):
    #     + c_d: vị trí ngang ban đầu.
    #     + c_d_d: vận tốc ngang ban đầu.
    #     + c_d_dd: gia tốc ngang ban đầu.

    #     ob:Mảng NumPy chứa tọa độ của các chướng ngại vật.

    # Tính toán các quỹ đạo trong hệ tọa độ Frenet
    # Hàm calc_frenet_paths tạo ra một danh sách các quỹ đạo ứng viên trong hệ tọa độ Frenet, 
    # bao gồm cả chuyển động ngang (lateral) và dọc (longitudinal).
    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    
    # Chuyển đổi quỹ đạo sang hệ tọa độ toàn cục
    # Sử dụng đường cong tham chiếu csp, hàm calc_global_paths tính toán vị trí toàn cục (x,y), góc lái (yaw), 
    # và độ cong cho từng quỹ đạo trong danh sách.
    fplist = calc_global_paths(fplist, csp)
    
    # Kiểm tra tính hợp lệ của quỹ đạo
    # Loại bỏ các quỹ đạo không hợp lệ (vi phạm giới hạn động học hoặc va chạm với chướng ngại vật) 
    # bằng cách sử dụng hàm check_paths.
    fplist = check_paths(fplist, ob)

    # find minimum cost path
    mincost = float("inf")
    bestpath = None
    for fp in fplist:
        if mincost >= fp.cf:
            mincost = fp.cf
            bestpath = fp
    # print(f"num path: {len(fplist)}")
    return bestpath, fplist

def generate_target_course(x, y):
    
    """Hàm generate_target_course tạo ra đường dẫn tham chiếu (target course) từ một tập hợp các điểm (x,y). 
        Đường dẫn này được biểu diễn dưới dạng spline 2D và bao gồm:
        các thuộc tính như tọa độ (x,y), góc định hướng (yaw), và độ cong (curvature)."""
    
    # cubic_spline_planner.Spline2D tạo spline hai chiều dựa trên các điểm x và y
    # Spline 2D cung cấp các hàm tính toán vị trí, góc yaw, và độ cong cho bất kỳ giá trị dọc s.
    csp = cubic_spline_planner.Spline2D(x, y)
    
    # Tạo danh sách các giá trị dọc s
    # Mỗi giá trị s đại diện cho một vị trí dọc (longitudinal position) dọc theo spline
    # Khoảng cách giữa các giá trị s là 0.1, đảm bảo độ phân giải cao.
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        # Tính tọa độ (x,y) tại vị trí s.
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        # Tính góc định hướng (yaw) tại s
        ryaw.append(csp.calc_yaw(i_s))
        # Tính độ cong (curvature) tại s
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp

# Hàm động học của xe
def update_vehicle_state(car_x, car_y, car_yaw, car_steer, c_speed, delta_t, L):
    """ Cập nhật vị trí và hướng của xe dựa trên động học """
    car_x += c_speed * np.cos(car_yaw) * delta_t
    car_y += c_speed * np.sin(car_yaw) * delta_t
    car_yaw += (c_speed / L) * np.tan(car_steer) * delta_t
    return car_x, car_y, car_yaw

def cartesian_to_frenet(x, y, yaw, csp):
    """
    Chuyển đổi tọa độ Cartesian sang Frenet dựa trên đường spline tham chiếu.

    Args:
        x (float): Tọa độ x của xe.
        y (float): Tọa độ y của xe.
        yaw (float): Góc phương vị của xe (radians).
        csp (CubicSpline2D): Đường spline tham chiếu.

    Returns:
        s (float): Vị trí dọc theo spline.
        d (float): Khoảng cách vuông góc đến spline.
        d_d (float): Tốc độ vuông góc.
        d_dd (float): Gia tốc vuông góc.
    """
    # Tìm giá trị s gần nhất trên spline
    s_min = 0.0
    s_max = csp.s[-1]
    ds = 0.1  # Bước tìm kiếm
    s_best = s_min
    min_dist = float('inf')

    for s_tmp in np.arange(s_min, s_max, ds):
        x_tmp, y_tmp = csp.calc_position(s_tmp)
        dist = np.hypot(x_tmp - x, y_tmp - y)
        if dist < min_dist:
            min_dist = dist
            s_best = s_tmp

    # Tính toán các giá trị dựa trên s_best
    s = s_best
    x_ref, y_ref = csp.calc_position(s)
    yaw_ref = csp.calc_yaw(s)
    d = np.hypot(x - x_ref, y - y_ref)

    # Xác định hướng của d (trái hay phải spline)
    cross_product = (x - x_ref) * -np.sin(yaw_ref) + (y - y_ref) * np.cos(yaw_ref)
    if cross_product < 0:
        d *= -1

    # Tính tốc độ và gia tốc vuông góc
    dx = x - x_ref
    dy = y - y_ref
    v_ref = np.array([np.cos(yaw_ref), np.sin(yaw_ref)])
    v_cartesian = np.array([dx, dy])

    # Tốc độ vuông góc
    d_d = np.dot(v_cartesian, [-v_ref[1], v_ref[0]])

    # Gia tốc vuông góc (giả sử không có dữ liệu gia tốc thêm)
    d_dd = 0.0

    return s, d, d_d, d_dd

# Vehicle parameters
LENGTH = 2.0  # total vehicle length
WIDTH = 1.0   # total vehicle width
BACKTOWHEEL = 0.2  # distance from rear to vehicle center
WHEEL_LEN = 0.3  # wheel length
WHEEL_WIDTH = 0.2  # wheel width
TREAD = 0.6  # width between left and right wheels
WB = 1.8  # wheel base: distance between front and rear axles

def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover
    """
    Plots a car at the given position and orientation.
    :param x: X-coordinate of the car's rear axle center.
    :param y: Y-coordinate of the car's rear axle center.
    :param yaw: Orientation of the car in radians.
    :param steer: Steering angle in radians.
    :param cabcolor: Color of the car body.
    :param truckcolor: Color of the wheels.
    """
    steer = -steer
    # Define the car outline and wheel shapes
    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, -WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    # Rotation matrices
    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    # Transform front wheels based on steering angle
    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    # Apply vehicle yaw rotation
    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T
    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    # Translate to vehicle position
    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    # Plot car body and wheels
    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), cabcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")  # Mark the vehicle's position
def plot_obstacles(obstacles, width=1, height=2):
    """
    Plot obstacles as rectangles.
    """
    ax = plt.gca()
    for obs in obstacles:
        x, y = obs
        rect = patches.Rectangle((x - width / 2, y - height / 2), width, height,
                                  linewidth=1, edgecolor='r', facecolor='none')
        ax.add_patch(rect)

def project_onto_path(x, y, tx, ty):
    """
    Project the point (x, y) onto the closest segment of the target course (tx, ty).
    Returns the projected point (px, py).
    """
    min_dist = float('inf')
    closest_point = None
    
    # Loop over each segment of the target course
    for i in range(len(tx) - 1):
        # Calculate vector for the current segment
        segment_start = np.array([tx[i], ty[i]])
        segment_end = np.array([tx[i+1], ty[i+1]])
        segment_vec = segment_end - segment_start
        
        # Vector from the segment start to the current point
        point_vec = np.array([x, y]) - segment_start
        
        # Project the point onto the segment
        segment_length = np.linalg.norm(segment_vec)
        if segment_length == 0:
            continue
            
        projection = np.dot(point_vec, segment_vec) / segment_length
        projection = np.clip(projection, 0, segment_length)
        
        projected_point = segment_start + (projection / segment_length) * segment_vec
        dist_to_projected = np.linalg.norm(projected_point - np.array([x, y]))
        
        if dist_to_projected < min_dist:
            min_dist = dist_to_projected
            closest_point = projected_point

    return closest_point

def calculate_heading_from_gps(lat1, lon1, lat2, lon2):
    """
    Tính heading giữa 2 điểm GPS trong hệ tọa độ XY.
    Trả về heading với độ chính xác cao.
    """
    # Bán kính Trái Đất (m)
    R = 6371e3  

    # Chuyển lat/lon sang radians
    lat1, lon1 = math.radians(lat1), math.radians(lon1)
    lat2, lon2 = math.radians(lat2), math.radians(lon2)

    # Tính khoảng cách x, y trong hệ tọa độ phẳng (equirectangular projection)
    dx = R * (lon2 - lon1) * math.cos(lat1)
    dy = R * (lat2 - lat1)

    # Tính heading trong hệ XY (theo góc arctan2)
    heading_xy = math.atan2(dy, dx)  # Trả về radian
    heading_xy_deg = math.degrees(heading_xy)

    # Đảm bảo góc trong khoảng [0, 360]
    if heading_xy_deg < 0:
        heading_xy_deg += 180

    return heading_xy_deg

def course_to_waypoint(current_lat, current_long, target_lat, target_long):
    """Calculate the heading to the target waypoint."""
    dlon = math.radians(float(target_long) - float(current_long))
    c_lat = math.radians(float(current_lat))
    t_lat = math.radians(float(target_lat))
    a1 = math.sin(dlon) * math.cos(t_lat)
    a2 = math.sin(c_lat) * math.cos(t_lat) * math.cos(dlon)
    a2 = math.cos(c_lat) * math.sin(t_lat) - a2
    a2 = math.atan2(a1, a2)
    if a2 < 0.0:
        a2 += math.pi * 2
    return math.degrees(a2)

def pure_pursuit_control_frenet(lat, lon, optimal_path, current_idx, x, y,yaw, lookahead_distance, WB):
    """
    Pure Pursuit controller for following the optimal trajectory by finding the
    lookahead point as the point on the path at the lookahead distance.
    """
    try:
        path_x = optimal_path.x
        path_y = optimal_path.y
    except:
        path_x = optimal_path.x
        path_y = optimal_path.y
    # Circle center is the car's current position
    car_pos = np.array([x, y])
    lookahead_point = None

    # Find the closest point on the path (we'll use a simple Euclidean distance)
    min_distance = float('inf')
    closest_idx = current_idx

    for idx in range(current_idx, len(path_x)):
        path_point = np.array([path_x[idx], path_y[idx]])
        distance_to_point = np.linalg.norm(car_pos - path_point)

        if distance_to_point < min_distance:
            min_distance = distance_to_point
            closest_idx = idx

    # Now find the lookahead point from the closest point
    lookahead_point = np.array([path_x[closest_idx], path_y[closest_idx]])

    # Calculate the vector from the car position to the lookahead point
    lookahead_vector = lookahead_point - car_pos
    lookahead_distance_actual = np.linalg.norm(lookahead_vector)

    if lookahead_distance_actual < lookahead_distance:
        # If the closest point is less than the lookahead distance, find the next point
        for idx in range(closest_idx + 1, len(path_x)):
            path_point = np.array([path_x[idx], path_y[idx]])
            distance_to_point = np.linalg.norm(car_pos - path_point)

            if distance_to_point >= lookahead_distance:
                lookahead_point = path_point
                break

    # Calculate the steering angle towards the lookahead point
    lookahead_lat, lookahead_lon = xy_to_lat_lon(lookahead_point[0], lookahead_point[1])
    
    # alpha = course_to_waypoint(lat, lon, lookahead_lat, lookahead_lon) - yaw
    
    alpha = calculate_heading_from_gps(lat, lon, lookahead_lat, lookahead_lon) - np.rad2deg(yaw)
    print("heading: ",calculate_heading_from_gps(lat, lon, lookahead_lat, lookahead_lon))
    
    alpha = - np.arctan2(lookahead_point[1] - y, lookahead_point[0] - x) + yaw
    
    # alpha = (alpha + 180) % 360 - 180  # Normalize to [-180, 180]
    # alpha = math.radians(alpha)
    
    steering_angle = np.arctan2(2.0 * WB * np.sin(alpha), lookahead_distance) * 180/np.pi
    # steering_angle_deg = np.degrees(steering_angle)
    steering_angle = np.clip(steering_angle, -30, 30)
    return steering_angle, closest_idx, lookahead_point

def calculate_speed_at_projected_point(projected_point, nearest_idx, csp):
    """
    Calculate the speed at the projected point on the target course.
    """
    # Speed at the nearest point in the course (csp is the speed profile)
    speed_at_point = csp[nearest_idx]
    return speed_at_point

def lat_lon_to_xy(lat, lon, lat0=10.8532570333, lon0=106.7715131967):
    """
    Convert latitude and longitude to x, y using an equirectangular projection.

    lat0 and lon0 represent the center point (origin) for the projection.
    """
    R = 6371e3  # Radius of the Earth in meters
    x = R * math.radians(lon0 - lon) * math.cos(math.radians(lat0))
    y = R * math.radians(lat0 - lat)
    return x, y

def xy_to_lat_lon(x, y):
    """Convert Cartesian coordinates (X, Y) back to latitude and longitude."""
    lon = (x / scaling_factor) + base_longitude
    lat = (y / scaling_factor) + base_latitude
    return lat, lon

def convert_yaw(yaw_deg_system1, yaw_offset=90):
    """
    Chuyển đổi góc yaw từ hệ tọa độ 1 sang hệ tọa độ 2.
    
    Args:
        yaw_deg_system1 (float): Góc yaw trong hệ tọa độ 1 (độ).
        yaw_offset (float): Độ lệch góc yaw giữa hệ tọa độ 1 và 2 (độ, mặc định là 100).
        
    Returns:
        float: Góc yaw trong hệ tọa độ 2 (độ).
    """
    # Chuyển góc từ độ sang radian
    yaw_rad_system1 = np.deg2rad(yaw_deg_system1)
    yaw_offset_rad = np.deg2rad(yaw_offset)
    
    # Tính góc yaw trong hệ tọa độ 2
    yaw_rad_system2 = yaw_rad_system1 - yaw_offset_rad
    
    # Đảm bảo góc nằm trong khoảng [0, 360) độ
    yaw_deg_system2 = (90 + (90 - np.rad2deg(yaw_rad_system2))) % 360
    
    return yaw_deg_system2

def update_state():
    lat, lon, car_heading, sat_count = GPS_module.get_gps_data(port = "COM17", baudrate  = 115200)
    # lat, lon, car_heading = 10.8532464083,106.7715122883, 186
    x, y= lat_lon_to_xy(float(lat), float(lon))
    # yaw = np.deg2rad(car_heading-90)
    # print("hi: ",convert_yaw(car_heading, yaw_offset=90))
    yaw_c = np.deg2rad(convert_yaw(car_heading, yaw_offset=90))
   
    return x,y,yaw_c

def calculate_heading_from_gps(lat1, lon1, lat2, lon2):
    """
    Tính heading giữa 2 điểm GPS trong hệ tọa độ XY.
    Trả về heading với độ chính xác cao.
    """
    # Bán kính Trái Đất (m)
    R = 6371e3  

    # Chuyển lat/lon sang radians
    lat1, lon1 = math.radians(lat1), math.radians(lon1)
    lat2, lon2 = math.radians(lat2), math.radians(lon2)

    # Tính khoảng cách x, y trong hệ tọa độ phẳng (equirectangular projection)
    dx = R * (lon2 - lon1) * math.cos(lat1)
    dy = R * (lat2 - lat1)

    # Tính heading trong hệ XY (theo góc arctan2)
    heading_xy = math.atan2(dy, dx)  # Trả về radian
    heading_xy_cvt = convert_yaw(heading_xy, yaw_offset=90)
    
    heading_xy_deg = (heading_xy_cvt + 360) % 360 # Đảm bảo trong đoạn [0:360] độ
    
    return heading_xy_deg

def transform_obstacle_to_global(x_vehicle, y_vehicle, heading, x_obstacle, y_obstacle):
    # Ma trận chuyển đổi
    T = np.array([
        [np.cos(heading), -np.sin(heading), x_vehicle],
        [np.sin(heading), np.cos(heading), y_vehicle],
        [0, 0, 1]
    ])
    
    # Tọa độ vật cản trong hệ tọa độ xe
    obstacle_local = np.array([x_obstacle + 1, - y_obstacle , 1])
    
    # Chuyển đổi sang hệ toàn cục
    obstacle_global = np.dot(T, obstacle_local)
    
    # Kết quả tọa độ toàn cục
    return obstacle_global[0], obstacle_global[1]


def main():
    print(__file__ + " start!!")

    # Waypoints
    wx = [0.0, 0.00018565031493520765, 0.0992026796220158, 0.3230748664458098, 0.6765308256629468, 1.042556389933707, 1.3980217386952056, 1.731645899860938, 2.129341186766323, 2.5148381707998655, 2.881944874592221, 3.233227635528738, 3.7490400739779357, 4.006591372980716, 4.328202878485722, 4.709145975616321, 5.240247244451409, 5.744232679106725, 6.20818321237363, 6.429325249972, 6.881612589733192, 7.276938108805592, 7.691014289511622, 7.9844178989740096, 8.347702394781916, 8.743027913854318, 9.093950294950302, 9.516752029594919, 9.942655214094831, 10.37875823271156, 10.745504555111474, 11.119174535351135, 11.424033849529447, 11.755845192255089, 12.125136013565495, 12.378679454268891, 12.758530491088555, 13.037737328924075, 13.404123273035369, 13.749028407839518, 14.163836267464038, 14.500004925988351, 14.889870148161812, 15.317225769709369, 15.734392477929449, 16.060557871889305, 16.46789604507052, 16.76147438561042, 16.76184568624029, 16.758569506859395, 16.754922028400543, 16.79969646700683, 16.802066234839813, 16.80261226499528, 16.82573116321692, 16.87869604639566, 16.88323901406117, 16.915826067481138, 16.993362287481073, 17.187923598713887, 17.49352551415194, 17.937076377633296, 18.317844743686386, 18.724997266552673, 19.07281820089718, 19.460870603078643, 19.899332470290926, 20.26753123284231, 20.53436503672848, 20.918944691528594, 21.30990197246108, 21.60512932270481, 21.22818316463616, 19.348563210588786, 16.13681638281528, 12.319708244035736, 8.344426216952932, 4.701861939487953, 1.1455703567508342, -3.0998107343616064, -7.6857287205863685, -12.02466654410695, -16.305168268312414, -20.626633139273324, -24.97120599077035, -29.15798717227951, -33.221169065319145, -36.91961402395132, -41.73395808545473, -46.69008422532714, -51.25835453257045, -55.404358212983375, -60.67700686209172, -66.46091406119442, -71.5797570570356, -75.85426337342408, -81.17714675648574, -87.60955161473437, -94.45877372215186, -100.89445475822947, -106.75244726997752, -112.23949994915941, -117.36089836470045, -122.2231073923242, -126.34090317824639, -128.85482401981284, -129.82747768492615, -130.38188350229782, -130.94465449687135, -131.52799988920412, -132.1734070223879, -132.51940421595108, -132.8699552979691, -133.5071719834766, -134.20117576014735, -134.99728709519107, -135.73971278892694, -136.51052110667908, -136.98065269652596, -137.44660170009982, -138.13313579032723, -138.8650012685538, -139.39046751096436, -140.04442546856112, -140.87766682469035, -141.676147929119, -142.49446083315092, -143.21431365726872, -143.8895667738577, -144.54789296804248, -145.19130163091936, -145.82815793813822, -146.3032037962805, -146.59459801775046, -147.01976952177995, -147.7409000549717, -148.4314420034985, -149.06483648102156, -149.7007754574551, -150.27465269614143, -151.18488424366706, -151.96280186802196, -152.8315351563222, -153.35389995042934, -152.5428601617364, -150.50326442018311, -146.42333033426485, -141.51742800910836, -136.98811146373185, -132.29208112177272, -127.57073684173265, -122.4946698141908, -117.72745903833733, -114.82877289320776, -114.7661651253051, -115.37516299398065, -115.94321955592751, -116.19584566739748, -113.99716962599302, -109.29566806634558, -103.76475827934274, -98.06784453620088, -92.37202285336997, -86.54989357442277, -80.46567002880728, -74.60786316737416, -69.39090955066999, -64.04200239063567, -59.05693667580215, -54.37326844700302, -48.7799256216231, -44.056582874379906, -40.65317961567652, -37.95488637775666, -33.10122817609782, -27.901572780875803, -24.73569245046761, -21.969680606797333, -19.374947365850065, -15.139394809776483, -10.740950668208532, -7.224879635622847]
    wy = [0.0, 0.016857150891428942  , 1.1814460955783999, 3.2207610502896076, 6.03510464364494,   8.937292229009408, 12.062236610982367, 15.333402321542493, 18.77674225654324, 22.23082362135899, 25.636913255814537, 29.042813858879143, 32.63588888204471, 35.81402891698038, 38.88097402515718, 41.92900487632694, 45.43202312845758, 49.31179203100935, 53.27811506431445, 57.53688075479389, 61.38163437501094, 65.00992483139575, 68.33317424556013, 71.30689352722794, 74.46594139325923, 77.92113470743341, 81.24141521700827, 84.40918076521969, 87.65921943954946, 91.22246086399564, 94.7167614337486, 97.94716308418587, 100.88826889393569, 104.03823213430101, 106.9441225108316, 110.17044330751841, 113.41104153258505, 116.52264252344175, 119.80456078331045, 123.1568987903216, 126.73108179526979, 130.1112185340694, 133.646850258118, 137.34481545537756, 140.90732299324716, 144.4829737713485, 148.19503848548555, 150.67708716021517, 150.66838059733107, 150.6663457302026, 150.70229504999844, 151.3818850831078, 151.48029259323212, 151.48177148587885, 151.73399493791842, 152.08092310911374, 152.25624414984375, 152.51032455731195, 153.0803875877001, 154.65341773695621, 157.12138913379545, 160.34678701251474, 163.59515776300438, 166.44045802412867, 169.39452916221285, 172.32507145404503, 175.21966442588385, 178.6153684543191, 181.99774021093415, 185.3511901671062, 189.0469425855375, 193.1750542372629, 197.13766336010585, 200.59490266070458, 203.1434903795131, 204.88721586056306, 206.20932353852896, 207.1811671974217, 208.325173961064, 209.60855244603547, 210.85708244087843, 212.10450048636298, 213.3115214152105, 214.51483955289189, 215.74150862512548, 216.86791323200364, 217.73412171053477, 218.26340956130647, 218.8186391886356, 219.38665623240172, 219.9409629417633, 220.39816312155244, 221.0236345839291, 221.69655292148033, 222.31664254935714, 222.76531407835708, 223.31388312962162, 224.10484600118966, 224.83391777679958, 225.64581865320702, 226.37247749870903, 226.95607295180193, 227.09042978172207, 225.99293585559573, 223.05572186840294, 218.36588680568767, 212.91363260906544, 208.15040889501321, 203.41888685440506, 198.15213915384612, 192.8612955126648, 189.29749811373694, 185.7972708542985, 180.6450539281787, 174.43499518709206, 167.86708893290023, 161.40461770818555, 155.38990621419438, 150.10647927464134, 145.23447356374058, 139.76349414130473, 134.12146356347782, 128.91661807856815, 123.46324081309389, 117.38792808400531, 111.13394734681536, 104.64850325033034, 98.15768843883905, 91.73043264734359, 85.51723820926365, 79.54198979464596, 73.9575915471294, 68.85374441415169, 65.61557023829144, 62.08364130540891, 57.02056935204766, 51.00047602335912, 44.95517480508482, 38.67340645559997, 32.89627404178275, 26.850238936931873, 20.665577116821943, 15.258534777361323, 10.426748271397772, 6.382955732610247, 3.47317353372014, 2.0474655427185486, 1.5200346471779185, 0.9625700020333732, 0.27538535539838577, -0.25519235662913264, -0.830070127344331, -2.111224713598906, -5.840891584697563, -11.017749306625344, -16.261702047113783, -21.74805972779433, -27.042239216655936, -31.457789753640196, -33.44576601081731, -34.51342633782919, -35.6665177271374, -36.66152328938894, -37.86131654780157, -38.76421935219863, -39.73661898577061, -40.44086093418154, -41.06002764409084, -41.69550664990344, -42.207192343911096, -42.773908406927845, -43.30171736525035, -43.94367903523014, -44.33842102488832, -44.90940697324399, -45.543040143121566, -45.88385259324046, -46.138110912606024, -46.389044503587336, -46.988752101501476, -47.54602771525508, -46.927594891724844]

    # Obstacles
    ob = np.array([[0, 0]])

    # Initial state
    c_speed = 0  # current speed [m/s]
    c_d = 0.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position

    # Generate target course (assuming `generate_target_course` is defined elsewhere)
    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)

    # Initial state
    # lat, lon, car_heading, sat_count = GPS_module.get_gps_data(port = "COM23", baudrate  = 115200)
    lat, lon, car_heading = 10.8532568817,106.7715131950, 185
    x, y = 16.993362287481073, 152.51032455731195
    
    
    
    # yaw = np.deg2rad(car_heading-90)  # Convert yaw to radians
    yaw = np.deg2rad(85)  # Convert yaw to radians
    dyaw = np.deg2rad(85)
    
    car_speed = 3 # Car speed [m/s]
    car_steer = 0  # Car steering angle [degree]
    lookahead_distance = 3.0  # Lookahead distance [m]
    L = 1.8  # Wheelbase of the vehicle [m]
    current_idx = 0  # Initial target point index
    
    count = 0
    obs = [[999, 999]]
    show_animation = True  # Enable animation
    area = 20.0  # Animation area length [m]
    while 1:
        # Update vehicle state using Real-Time-Kinematic GPS
        x, y, yaw = update_state()
        obstacles, image = process_depth()
        # obstacles = [[5,3]]
        for obstacle in obstacles:
            obs.append(transform_obstacle_to_global(x, y, yaw, obstacle[1], obstacle[0]))
        
        ob = np.array(obs)
        
        print(ob)
        
        # Project the current position onto the target course
        # projected_point = project_onto_path(x, y, tx, ty)
        # s0 , c_d, c_d_d, c_d_dd = cartesian_to_frenet(projected_point[0], projected_point[1], yaw, csp)
        
        optimal_path, paths = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)
        
        while optimal_path is None:
            
            
            print("optimal_path is None !!!")
            x, y, yaw = update_state()
        
            # Project the current position onto the target course
            projected_point = project_onto_path(x, y, tx, ty)
            s0 , c_d, c_d_d, c_d_dd = cartesian_to_frenet(projected_point[0], projected_point[1], yaw, csp)
            
            optimal_path, paths = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)
            # continue

        # print(x,y)
        if x is not None:
            # Pure Pursuit control: use the optimal path from Frenet
            car_steer, current_idx, lookahead_point = pure_pursuit_control_frenet(lat, lon,
                optimal_path, current_idx, x, y,yaw, lookahead_distance, L)

            # Project the current position onto the target course
            projected_point = project_onto_path(x, y, tx, ty)
            # s0 , _, __, ___ = cartesian_to_frenet(projected_point[0], projected_point[1], yaw, csp)
            s0 , _, __, ___ = cartesian_to_frenet(x, y, yaw, csp)
            
            c_d = optimal_path.d[1]
            c_d_d = optimal_path.d_d[1]
            c_d_dd = optimal_path.d_dd[1]
            c_speed = car_speed    
            
            # ----------- Control Block ------------- #
            steering_angle = car_steer
            
            count += 1
            if count == 1:
                stm32(angle= int(steering_angle), speed=0, brake_state=0)
                count = 0    

            print(f"Steering Angle: {steering_angle:.2f}")
            

            # Check if the goal is reached
            if np.isclose(x, tx[-1], atol=1.0) and np.isclose(y, ty[-1], atol=1.0):
                print("Goal reached!")
                break

            # Visualization
            if show_animation:
                plt.cla()
                plt.plot(tx, ty, "--y", label="Target Course" )

                tyaw = np.array(tyaw)
                # Tính toán đường song song
                left_x = tx + (MAX_ROAD_WIDTH/2 +0.5) * np.cos(tyaw + np.pi / 2)  # Dịch chuyển 3m sang bên trái
                left_y = ty + (MAX_ROAD_WIDTH/2 +0.5) * np.sin(tyaw + np.pi / 2)
                right_x = tx +( MAX_ROAD_WIDTH/2+0.5) * np.cos(tyaw - np.pi / 2)  # Dịch chuyển 3m sang bên phải
                right_y = ty +( MAX_ROAD_WIDTH/2+0.5) * np.sin(tyaw - np.pi / 2)

                # Vẽ đường biên trái và phải
                plt.plot(left_x, left_y, "-b", label="Left Boundary")
                plt.plot(right_x, right_y, "-b", label="Right Boundary")
                
                plt.plot(ob[:, 0], ob[:, 1], "xb", label="Obstacles")

                # Plot
                for traj in paths:
                    plt.plot(traj.x, traj.y, "black", alpha=0.05)

                plt.plot(optimal_path.x[1:], optimal_path.y[1:], "-g",alpha=0.95, label="Optimal Trajectory")
                circle = plt.Circle((x, y), lookahead_distance, color='b', fill=False, linestyle='--')
                plt.gca().add_artist(circle)
                plt.scatter(lookahead_point[0], lookahead_point[1], color='red', s=20, label="Lookahead Point")
                
                plot_car(x, y, yaw, np.deg2rad(steering_angle), cabcolor="-r")
                plt.scatter(x, y, color='blue', label="Current Position")

                # Plot the projected point
                if projected_point is not None:
                    plt.scatter(projected_point[0], projected_point[1], color='green', s=20, label="Projected Point")

                
                plt.xlim(x - area, x + area)
                plt.ylim(y - area, y + area)
                plt.title(f"Speed: {car_speed * 3.6:.2f} km/h | Steering Angle: {(car_steer):.2f}°")
                plt.grid(True)
                plt.pause(1/100)
                obstacles.clear()
                obs = [[999, 999]]
            else:
                obs = [[999, 999]]
                
        cv2.imshow("img", image)
        cv2.waitKey(1)
    
    if show_animation:
        plt.show()

if __name__ == '__main__':
    main()
