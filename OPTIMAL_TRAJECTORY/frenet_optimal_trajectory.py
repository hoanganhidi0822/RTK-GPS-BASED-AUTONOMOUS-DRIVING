import  numpy as  np
import math
from OPTIMAL_TRAJECTORY.CUBIC_PLANNER import cubic_spline_planner 

# Parameter 
MAX_SPEED = 20.0 / 3.6    # maximum speed [m/s]
MAX_ACCEL = 3             # maximum acceleration [m/ss]
MAX_CURVATURE = 10        # maximum curvature (độ cong) [1/m]
MAX_ROAD_WIDTH = 6        # maximum road width [m]
D_ROAD_W = 0.6           # road width sampling length [m]
DT = 0.3                  # time tick [s]
MAXT = 5.0               # max prediction time [m]
MINT = 4.0                # min prediction time [m]
TARGET_SPEED = 18 / 3.6   # target speed [m/s]
D_T_S = 0.1 / 3.6         # target speed sampling length [m/s]
N_S_SAMPLE = 1            # sampling number of target speed
ROBOT_RADIUS = 2.2       # robot radius [m]

# cost weights Trajectory 
KJ = 0.1
KT = 0.1
KD = 1.0
KLAT = 2.0
KLON = 1.0

# Base reference for conversion
base_latitude  = 10.8532570333 # Latitude of the first waypoint
base_longitude = 106.7715131967  # Longitude of the first waypoint
scaling_factor = 100000

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
    frenet_paths = []

    # Create an array of lateral offsets and time durations
    di_values = np.arange(-MAX_ROAD_WIDTH/4, MAX_ROAD_WIDTH*0.75, D_ROAD_W) # Do xe chạy bên làn phải
    Ti_values = np.arange(MINT, MAXT, DT)
    
    # Loop through each lateral offset (di) and time duration (Ti)
    for di in di_values:
        for Ti in Ti_values:
            # Create lateral trajectory using quintic polynomial (for all time steps)
            lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            t_values = np.arange(0.0, Ti, DT)

            # Calculate lateral parameters (fp.d, fp.d_d, fp.d_dd, fp.d_ddd) using vectorized operations
            fp_d = lat_qp.calc_point(t_values)
            fp_d_d = lat_qp.calc_first_derivative(t_values)
            fp_d_dd = lat_qp.calc_second_derivative(t_values)
            fp_d_ddd = lat_qp.calc_third_derivative(t_values)

            # Create longitudinal trajectories for each target speed tv
            tv_values = np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S)

            # Use broadcasting to calculate the longitudinal trajectories in parallel
            for tv in tv_values:
                tfp = Frenet_path()

                # Generate longitudinal trajectory using quartic polynomial
                lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                # Calculate longitudinal parameters (tfp.s, tfp.s_d, tfp.s_dd, tfp.s_ddd) using vectorized operations
                tfp_s = lon_qp.calc_point(t_values)
                tfp_s_d = lon_qp.calc_first_derivative(t_values)
                tfp_s_dd = lon_qp.calc_second_derivative(t_values)
                tfp_s_ddd = lon_qp.calc_third_derivative(t_values)

                # Compute costs (Jp, Js, ds)
                Jp = np.sum(np.power(fp_d_ddd, 2))  # Use fp_d_ddd for lateral jerk
                Js = np.sum(np.power(tfp_s_ddd, 2))  # Use tfp_s_ddd for longitudinal jerk
                ds = np.square(TARGET_SPEED - tfp_s_d[-1])

                # Calculate total costs (tfp.cd, tfp.cv, tfp.cf) using vectorized operations
                tfp_cd = KJ * Jp + KT * Ti + KD * np.square(fp_d[-1])  # Cost do độ lệch ngang cuối cùng.
                tfp_cv = KJ * Js + KT * Ti + KD * ds
                tfp_cf = KLAT * tfp_cd + KLON * tfp_cv

                # Assign the calculated values to tfp
                tfp.t = t_values
                tfp.d = fp_d
                tfp.d_d = fp_d_d
                tfp.d_dd = fp_d_dd
                tfp.d_ddd = fp_d_ddd
                tfp.s = tfp_s
                tfp.s_d = tfp_s_d
                tfp.s_dd = tfp_s_dd
                tfp.s_ddd = tfp_s_ddd
                tfp.cd = tfp_cd
                tfp.cv = tfp_cv
                tfp.cf = tfp_cf

                # Append the trajectory to the list
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

# def check_collision(fp, ob):
#     # fp:Một đối tượng thuộc lớp Frenet_path chứa thông tin quỹ đạo, bao gồm danh sách các điểm tọa độ toàn cục (x,y).
#     # ob:Mảng NumPy ob chứa tọa độ của các chướng ngại vật: Mỗi hàng đại diện cho một chướng ngại vật với (x,y).
    
#     # Duyệt qua từng chướng ngại vật
#     # Lặp qua tất cả các chướng ngại vật trong danh sách ob
#     for i in range(len(ob[:, 0])):
#         print(f"len fp.x: {(i)}")
#         # Tính khoảng cách từ từng điểm trên quỹ đạo đến chướng ngại vật
#         # Với mỗi chướng ngại vật ob[i]=(xob,yob), tính khoảng cách bình phương từ nó đến từng điểm (x,y) trên quỹ đạo fp:
#         d = [((ix - ob[i, 0])**2 + (iy - ob[i, 1])**2)
#              for (ix, iy) in zip(fp.x, fp.y)]

#         # So sánh mỗi khoảng cách với bán kính va chạm
#         collision = any([di <= ROBOT_RADIUS**2 for di in d])

#         if collision:
#             return False

#     return True

def check_collision(fp, ob):
    """
    Kiểm tra va chạm nhưng chỉ xét 3/4 quỹ đạo phía xa nhất của xe.

    Parameters:
        fp: Đối tượng Frenet_path chứa danh sách tọa độ (x, y).
        ob: Mảng NumPy chứa danh sách tọa độ (x, y) của các chướng ngại vật.

    Returns:
        True nếu 3/4 quỹ đạo phía xa an toàn, False nếu có va chạm.
    """
    path_length = len(fp.x)
    three_fourths_index = path_length // 4  # Lấy 1/4 đầu bỏ đi, lấy 3/4 sau
    
    # Chỉ lấy 3/4 quỹ đạo phía xa
    x_far = fp.x[three_fourths_index:]  
    y_far = fp.y[three_fourths_index:]

    for i in range(ob.shape[0]):  # Duyệt qua từng chướng ngại vật
        x_ob, y_ob = ob[i]  # Tọa độ của vật cản

        # Kiểm tra va chạm chỉ với 3/4 quỹ đạo phía xa
        collision = any((ix - x_ob) ** 2 + (iy - y_ob) ** 2 <= ROBOT_RADIUS**2 
                        for ix, iy in zip(x_far, y_far))

        if collision:
            return False  # Có va chạm, quỹ đạo không an toàn

    return True  # Không có va chạm, quỹ đạo an toàn

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
            # print("Max speed check")
            continue
        elif any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd]):  # Max accel check
            # Giới hạn độ cong tối đa
            # print("Max accel check")
            continue
        elif any([abs(c) > MAX_CURVATURE for c in fplist[i].c]):  # Max curvature check
            # Giới hạn độ cong tối đa
            # print("Max curvature check")
            continue
        elif not check_collision(fplist[i], ob):
            # Nếu quỹ đạo va chạm với bất kỳ chướng ngại vật nào, bỏ qua quỹ đạo.
            # print("check_collision")
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
    # Tạo danh sách giá trị s
    s_min = 0.0
    s_max = csp.s[-1]
    ds = 0.1  # Bước tìm kiếm
    s_values = np.arange(s_min, s_max, ds)

    # Tính toán vị trí (x, y) trên spline cho tất cả giá trị s
    positions = np.array([csp.calc_position(s) for s in s_values])
    x_spline, y_spline = positions[:, 0], positions[:, 1]

    # Tính khoảng cách từ điểm (x, y) đến tất cả các điểm trên spline
    distances = np.hypot(x_spline - x, y_spline - y)

    # Tìm giá trị s tương ứng với khoảng cách nhỏ nhất
    min_idx = np.argmin(distances)
    s_best = s_values[min_idx]

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

def project_onto_path(x, y, tx, ty):
    """
    Project the point (x, y) onto the closest segment of the target course (tx, ty).
    Returns the projected point (px, py).
    """
    # Chuyển dữ liệu thành mảng NumPy
    segments_start = np.stack([tx[:-1], ty[:-1]], axis=1)
    segments_end = np.stack([tx[1:], ty[1:]], axis=1)
    segment_vecs = segments_end - segments_start

    # Tính vector từ mỗi điểm bắt đầu đoạn thẳng đến điểm cần chiếu
    point_vecs = np.array([x, y]) - segments_start

    # Tính độ dài của các đoạn thẳng
    segment_lengths = np.linalg.norm(segment_vecs, axis=1)
    nonzero_mask = segment_lengths > 0  # Tránh chia cho 0

    # Tính tỉ lệ chiếu của điểm lên các đoạn thẳng
    projections = np.einsum('ij,ij->i', point_vecs, segment_vecs)  # Dot product
    projections = np.divide(projections, segment_lengths, where=nonzero_mask)
    projections = np.clip(projections, 0, segment_lengths)  # Clip vào đoạn thẳng

    # Tính các điểm chiếu
    projected_points = segments_start + (projections[:, None] / segment_lengths[:, None]) * segment_vecs

    # Tính khoảng cách từ điểm gốc đến các điểm chiếu
    distances = np.linalg.norm(projected_points - np.array([x, y]), axis=1)

    # Lấy điểm chiếu gần nhất
    min_idx = np.argmin(distances)
    closest_point = projected_points[min_idx]

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


def find_lookahead_point(x, y, lookahead_distance, path_x, path_y):
    """
    Tìm giao điểm giữa đường tròn (tâm xe, bán kính lookahead_distance) 
    và đoạn thẳng nối hai waypoint gần nhất.
    """
    for i in range(len(path_x) - 1):
        x1, y1 = path_x[i], path_y[i]
        x2, y2 = path_x[i + 1], path_y[i + 1]

        # Hệ số của phương trình đường thẳng y = ax + b
        if x2 - x1 == 0:
            continue  # Bỏ qua nếu đường thẳng thẳng đứng

        a = (y2 - y1) / (x2 - x1)
        b = y1 - a * x1

        # Phương trình đường tròn: (x - x0)^2 + (y - y0)^2 = r^2
        # Kết hợp với y = ax + b, ta có phương trình bậc 2 Ax^2 + Bx + C = 0
        A = 1 + a**2
        B = 2 * (a * (b - y) - x)
        C = x**2 + (b - y)**2 - lookahead_distance**2

        # Giải phương trình bậc 2
        delta = B**2 - 4 * A * C
        if delta < 0:
            continue  # Không có giao điểm thực

        sqrt_delta = np.sqrt(delta)
        x_sol1 = (-B + sqrt_delta) / (2 * A)
        x_sol2 = (-B - sqrt_delta) / (2 * A)
        y_sol1 = a * x_sol1 + b
        y_sol2 = a * x_sol2 + b

        # Chọn điểm phía trước xe trên đường đi
        if x1 <= x_sol1 <= x2 or x1 >= x_sol1 >= x2:
            return np.array([x_sol1, y_sol1])
        if x1 <= x_sol2 <= x2 or x1 >= x_sol2 >= x2:
            return np.array([x_sol2, y_sol2])

    return np.array([path_x[-1], path_y[-1]])  # Nếu không tìm thấy, chọn điểm cuối

def pure_pursuit_control_frenet(lat, lon, optimal_path, x, y, yaw, lookahead_distance, WB):
    path_x, path_y = optimal_path.x, optimal_path.y

    # Tìm điểm lookahead dựa vào giao điểm giữa đường tròn và đường thẳng
    lookahead_point = find_lookahead_point(x, y, lookahead_distance, path_x, path_y)

    # Tính góc alpha giữa hướng xe và lookahead point
    alpha = np.arctan2(lookahead_point[1] - y, lookahead_point[0] - x) - yaw

    # Tính góc lái bằng công thức Pure Pursuit
    steering_angle = -np.arctan2(2.0 * WB * np.sin(alpha), lookahead_distance) * 180/np.pi
    steering_angle = np.clip(steering_angle, -35, 35)

    return steering_angle, lookahead_point

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
    R = 6371e3  # Bán kính Trái Đất (mét)
    x = R * math.radians(lon0 - lon) * math.cos(math.radians(lat0))
    y = R * math.radians(lat0 - lat)
    return x, y

def xy_to_lat_lon(x, y):
    """Convert Cartesian coordinates (X, Y) back to latitude and longitude."""
    lon = (x / scaling_factor) + base_longitude
    lat = (y / scaling_factor) + base_latitude
    return lat, lon

def convert_yaw(yaw_deg_system1, yaw_offset = 92):
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
    yaw_offset_rad  = np.deg2rad(yaw_offset)
    
    # Tính góc yaw trong hệ tọa độ 2
    yaw_rad_system2 = yaw_rad_system1 - yaw_offset_rad
    
    # Đảm bảo góc nằm trong khoảng [0, 360) độ
    yaw_deg_system2 = (90 + (90 - np.rad2deg(yaw_rad_system2))) % 360
    
    return yaw_deg_system2


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
        [np.sin(heading),  np.cos(heading), y_vehicle],
        [     0,                0,              1    ]
    ])
    
    # Tọa độ vật cản trong hệ tọa độ xe
    obstacle_local = np.array([x_obstacle, -y_obstacle , 1])
    
    # Chuyển đổi sang hệ toàn cục
    obstacle_global = np.dot(T, obstacle_local)
    
    # Kết quả tọa độ toàn cục
    return obstacle_global[0], obstacle_global[1]

