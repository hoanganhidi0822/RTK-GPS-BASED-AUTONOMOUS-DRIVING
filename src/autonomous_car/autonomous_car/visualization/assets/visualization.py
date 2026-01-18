import math
import os
import threading
import cv2
import numpy as np
import simpleaudio as sa
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.transforms import Affine2D
from PyQt5.QtMultimedia import QMediaPlayer
from PyQt5.QtCore import Qt, QTimer, QSize, QPoint, pyqtSlot
from PyQt5.QtGui import QIcon, QPainter, QColor, QPixmap, QImage
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QFrame, QPushButton,
    QSizePolicy, QLabel, QDialog, QStackedLayout, QSplitter
)
from autonomous_car.visualization.assets.FACE_DETECTION.face import FaceRecognition
from autonomous_car.Fot_node.Frenet import lat_lon_to_xy, convert_yaw
from std_msgs.msg import String, Float32, Int32, Float32MultiArray
import config as cf
from pydub.playback import play
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import MarkerArray
#Ros2
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor

pkg_share = get_package_share_directory('autonomous_car')
image_global = None

wave_obj = sa.WaveObject.from_wave_file(f"{pkg_share}/assets/sound/output.wav")
micro_sound = sa.WaveObject.from_wave_file(f"{pkg_share}/assets/sound/micro_sound.wav")

EARTH_RADIUS_M = 6371000.0
DEFAULT_ORIGIN_LAT = float(os.environ.get("ORIGIN_LAT", "10.8532570333"))
DEFAULT_ORIGIN_LON = float(os.environ.get("ORIGIN_LON", "106.7715131967"))


def latlon_from_xy(x: float, y: float, lat0: float, lon0: float):
    lat = lat0 - math.degrees(y / EARTH_RADIUS_M)
    lon = lon0 - math.degrees(x / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon


from scipy.ndimage import rotate
import numpy as np

def rotate_image(img, angle_rad):
    """
    Xoay ảnh theo góc radian và đảm bảo giá trị RGB trong khoảng hợp lệ.
    """
    angle_deg = -np.degrees(angle_rad)
    rotated = rotate(img, angle_deg, reshape=True)
    
    # Clip về [0, 1] nếu là float, hoặc [0, 255] nếu là int
    if rotated.dtype == np.float32 or rotated.dtype == np.float64:
        rotated = np.clip(rotated, 0.0, 1.0)
    else:
        rotated = np.clip(rotated, 0, 255)
    
    return rotated
def plot_car(x, y, yaw, ax, icon_path = f"{pkg_share}/assets/icon/car.png", zoom=0.12):
    """
    Vẽ xe bằng icon PNG tại vị trí (x, y) với góc yaw.
    """
    car_img = mpimg.imread(icon_path)
    rotated_img = rotate_image(car_img, yaw)
    imagebox = OffsetImage(rotated_img, zoom=zoom)
    ab = AnnotationBbox(imagebox, (x, y-1), frameon=False)
    ax.add_artist(ab)

    ax.plot(x, y, "*", color="yellow")


class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = plt.figure(figsize=(30, 10))
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.updateGeometry()

        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 25)
        self.ax.set_facecolor('black')

        self.vehicle_pos = (0, 0)
        self.vehicle_yaw = np.deg2rad(90)
        self.filtered_inv_yaw = np.deg2rad(90)
        self.obstacles = np.array([[999, 999]])
        self.paths = []
        self.optimal_path = []
        self.tx = []
        self.ty = []
        self.tyaw = []

        self.left_x = []
        self.left_y = []
        self.mid_x = []
        self.mid_y = []
        self.right_x = []
        self.right_y = []

        self.path_lines = []
        self.obstacle_artists = []
        self._closing = False
        self._layout_done = False

        self.obstacle_icon = mpimg.imread(f"{pkg_share}/assets/icon/obstacle.png")
        self.obstacle_img_raw = OffsetImage(self.obstacle_icon, zoom=0.15)
        self.vehicle_icon = mpimg.imread(f"{pkg_share}/assets/icon/car.png")
        self.vehicle_icon = rotate(self.vehicle_icon, -90, reshape=True) 
        self.vehicle_img = OffsetImage(self.vehicle_icon, zoom=0.15)
        self.vehicle_box = AnnotationBbox(
            self.vehicle_img,
            (0.5, 0.1),
            xycoords='axes fraction',
            frameon=False,
            zorder=100
        )
        self.ax.add_artist(self.vehicle_box)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(33)

    def update_plot(self):
        if self._closing:
            return
        for line in self.path_lines:
            line.remove()
        self.path_lines.clear()
        for artist in self.obstacle_artists:
            artist.remove()
        self.obstacle_artists.clear()
        self.ax.set_xlim(self.vehicle_pos[0] - 5, self.vehicle_pos[0] + 5)
        self.ax.set_ylim(self.vehicle_pos[1] - 5, self.vehicle_pos[1] + 25)
        self.ax.set_facecolor('black')
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_xticklabels([])
        self.ax.set_yticklabels([])
        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        self.ax.spines['left'].set_visible(False)
        self.ax.spines['bottom'].set_visible(False)

        def rotate(x, y, theta):
            R = np.array([[np.cos(theta), -np.sin(theta)],
                        [np.sin(theta),  np.cos(theta)]])
            return np.dot(R, np.array([x, y]))

        def rotate_array(x_arr, y_arr, theta):
            dx = np.array(x_arr) - self.vehicle_pos[0]
            dy = np.array(y_arr) - self.vehicle_pos[1]
            x_rot = dx * np.cos(theta) - dy * np.sin(theta)
            y_rot = dx * np.sin(theta) + dy * np.cos(theta)
            return x_rot + self.vehicle_pos[0], y_rot + self.vehicle_pos[1]
        
        alpha = 0.2
        self.filtered_inv_yaw = alpha * (np.deg2rad(90) - self.vehicle_yaw) + (1 - alpha) * self.filtered_inv_yaw
        inv_yaw =  self.filtered_inv_yaw
        
        tx_rot, ty_rot = rotate_array(self.tx, self.ty, inv_yaw)
        self.path_lines.append(self.ax.plot(tx_rot[1:], ty_rot[1:], "white", linewidth=0.5)[0])

        if not isinstance(self.tyaw, np.ndarray):
            self.tyaw = np.array(self.tyaw)
        if len(self.left_x) != len(self.tx):
            MAX_ROAD_WIDTH = 6.0
            self.left_x  = self.tx + (MAX_ROAD_WIDTH * 0.67 ) * np.cos(self.tyaw + np.pi / 2)
            self.left_y  = self.ty + (MAX_ROAD_WIDTH * 0.67 ) * np.sin(self.tyaw + np.pi / 2)
            self.mid_x   = self.tx + (MAX_ROAD_WIDTH * 0.125) * np.cos(self.tyaw + np.pi / 2)
            self.mid_y   = self.ty + (MAX_ROAD_WIDTH * 0.125) * np.sin(self.tyaw + np.pi / 2)
            self.right_x = self.tx + (MAX_ROAD_WIDTH * 0.33 ) * np.cos(self.tyaw - np.pi / 2)
            self.right_y = self.ty + (MAX_ROAD_WIDTH * 0.33 ) * np.sin(self.tyaw - np.pi / 2)
            
        left_x_rot, left_y_rot   = rotate_array(self.left_x, self.left_y, inv_yaw)
        mid_x_rot, mid_y_rot     = rotate_array(self.mid_x, self.mid_y, inv_yaw)
        right_x_rot, right_y_rot = rotate_array(self.right_x, self.right_y, inv_yaw)

        self.path_lines.append(self.ax.plot(mid_x_rot, mid_y_rot, "--y", linewidth=3)[0])
        self.path_lines.append(self.ax.plot(left_x_rot, left_y_rot, "whitesmoke", linewidth=2)[0])
        self.path_lines.append(self.ax.plot(right_x_rot, right_y_rot, "whitesmoke", linewidth=2)[0])

        if self.paths:
            max_len = max(len(traj.x) for traj in self.paths if traj.x)
            all_x = [traj.x + [np.nan] * (max_len - len(traj.x)) for traj in self.paths]
            all_y = [traj.y + [np.nan] * (max_len - len(traj.y)) for traj in self.paths]
            all_x = np.array(all_x)
            all_y = np.array(all_y)
            for i in range(all_x.shape[0]):
                x_rot, y_rot = rotate_array(all_x[i, :], all_y[i, :], inv_yaw)
                self.path_lines.append(self.ax.plot(x_rot, y_rot, "white", alpha=0.4, linewidth=0.5)[0])

            optimal_x, optimal_y = rotate_array(self.optimal_path.x, self.optimal_path.y, inv_yaw)
            self.path_lines.append(self.ax.plot(optimal_x[1:], optimal_y[1:], "deepskyblue", alpha=0.9, linewidth=25)[0])

        for obs in self.obstacles:
            dx = obs[0] - self.vehicle_pos[0]
            dy = obs[1] - self.vehicle_pos[1]
            if not (-5 <= dx <= 5) or not (-25 <= dy <= 25):
                continue
            x_rot, y_rot = rotate(dx, dy, self.filtered_inv_yaw)
            x_plot = x_rot + self.vehicle_pos[0]
            y_plot = y_rot + self.vehicle_pos[1] + 1
            trans_data = Affine2D().rotate_around(x_plot, y_plot, self.filtered_inv_yaw) + self.ax.transData
            obstacle_img_rotated = OffsetImage(self.obstacle_icon, zoom=0.15)
            ab = AnnotationBbox(obstacle_img_rotated, (x_plot, y_plot), frameon=False)
            ab.set_transform(trans_data)
            self.ax.add_artist(ab)
            self.obstacle_artists.append(ab)

        self.ax.set_xlim(self.vehicle_pos[0] - 6, self.vehicle_pos[0] + 6)
        self.ax.set_ylim(self.vehicle_pos[1] - 5, self.vehicle_pos[1] + 30)
        if not self._layout_done:
            try:
                self.fig.tight_layout()
            except Exception:
                pass
            self._layout_done = True
        self.ax.figure.canvas.draw_idle()

    def shutdown(self):
        self._closing = True
        if getattr(self, "timer", None) is not None:
            self.timer.stop()

    def update_vehicle_position(self, x, y, yaw, paths, optimal_path, tx, ty, tyaw):
        self.vehicle_pos = (x, y)
        self.vehicle_yaw = yaw
        self.paths = paths or []
        self.optimal_path = optimal_path
        self.tx = tx
        self.ty = ty
        self.tyaw = tyaw

    def update_obstacles(self, new_obstacles):
        self.obstacles = new_obstacles

class VehicleStatusFrame(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setStyleSheet("""
            background-color: white;
            border-radius: 20px;
        """)
        self.setFixedHeight(200)

        # Layout chính của Frame
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(10)

        # Layout chứa GPS icon và trạng thái
        gps_layout = QHBoxLayout()
        gps_layout.setAlignment(Qt.AlignRight)

        self.gps_icon = QLabel()
        pixmap = QPixmap(f"{pkg_share}/assets/icon/gps.png").scaled(30, 30, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.gps_icon.setPixmap(pixmap)
        gps_layout.addWidget(self.gps_icon)

        self.gps_status = QLabel("Khởi tạo...")
        self.gps_status.setStyleSheet("font-size: 14px; color: black; margin-left: 6px;")
        gps_layout.addWidget(self.gps_status)

        main_layout.addLayout(gps_layout)
        # ==== Spacer phía trên ====
        main_layout.addStretch(1)

        # ==== Container căn giữa label ====
        status_container = QWidget()
        status_layout = QVBoxLayout(status_container)
        status_layout.addStretch(1)

        self.vehicle_status_label = QLabel("Trạng thái xe: Đang khởi tạo")
        self.vehicle_status_label.setAlignment(Qt.AlignCenter)
        self.vehicle_status_label.setWordWrap(True)  # Cho phép xuống dòng nếu cần
        self.vehicle_status_label.setMaximumWidth(7000)  # Giới hạn chiều rộng
        
        self.vehicle_status_label.setStyleSheet("""
            font-size: 70px;
            font-weight: bold;
            color: #333;
        """)
        status_layout.addWidget(self.vehicle_status_label, alignment=Qt.AlignCenter)

        status_layout.addStretch(1)
        main_layout.addWidget(status_container)

        # ==== Spacer phía dưới ====
        main_layout.addStretch(4)
        self.setLayout(main_layout)

    def update_gps_status(self, status_text):
        self.gps_status.setText(status_text)
        if "Fixed" in status_text:
            color = "lightgreen"
        elif "RTK INS Fusion" in status_text:
            color = "orange"
        else:
            color = "red"

        self.gps_status.setStyleSheet(
            f"font-size: 24px; color: {color}; margin-left: 6px;font-weight: bold;"
        )

    def update_vehicle_status(self, status_text):

        if isinstance(status_text, (int, float)):
            # format 1 chữ số thập phân
            text = f"{status_text:.1f} km/h"

            self.vehicle_status_label.setStyleSheet("""
                font-size: 70px;
                font-weight: bold;
                color: #333;
            """)
        else:
            text = str(status_text)
            self.vehicle_status_label.setStyleSheet("""
                font-size: 20px;
                font-weight: bold;
                color: red;
            """)

        self.vehicle_status_label.setText(text)


class SimplePath:
    def __init__(self, x, y, yaw=None):
        self.x = x
        self.y = y

        if yaw is None:
            # tự tính yaw nếu không truyền vào
            self.yaw = self._compute_yaw_from_xy(x, y)
        else:
            self.yaw = yaw

    @staticmethod
    def _compute_yaw_from_xy(x, y):
        import math
        n = len(x)
        if n < 2:
            return [0.0] * n

        yaws = []
        for i in range(n - 1):
            dx = x[i+1] - x[i]
            dy = y[i+1] - y[i]
            yaws.append(math.atan2(dy, dx))
        yaws.append(yaws[-1])  # yaw điểm cuối cùng = yaw điểm trước
        return yaws


class VehicleDisplayFrame(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)  # Loại bỏ padding bên ngoài
        layout.setSpacing(2)  # Loại bỏ khoảng cách giữa các widget

        self.plot_canvas = PlotCanvas(self)

        # QFrame bo tròn góc
        self.round_frame = QFrame(self)
        self.round_frame.setStyleSheet("""
            background-color: white;
            border-radius: 20px; /* Bo tròn góc */
            padding: 5px;
        """)

        # Đảm bảo chiều cao tự động điều chỉnh theo kích thước cửa sổ
        self.round_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        round_layout = QVBoxLayout(self.round_frame)
        round_layout.setContentsMargins(2, 2, 2, 2)
        round_layout.addWidget(self.plot_canvas)

        layout.addWidget(self.round_frame)

# Widget Animation "Listening"
class ListeningDialog(QDialog):
    def __init__(self, map_frame, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setModal(True)

        # Lấy kích thước map_frame
        x, y = map_frame.mapToGlobal(QPoint(0, 0)).x(), map_frame.mapToGlobal(QPoint(0, 0)).y()
        width, height = 1920 , 1080 
        self.setGeometry(x, y, width, height)

        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignRight)

        # Animation state
        self.base_radius = 100
        self.pulse_amplitude = 10  # Biên độ co giãn
        self.angle = 0  # Góc dùng cho sin

        # Timer update animation mượt hơn (~60 FPS)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_animation)
        self.timer.start(50)  # 60 FPS

        # Tự động đóng sau 6 giây
        self.auto_close_timer = QTimer(self)
        self.auto_close_timer.setSingleShot(True)
        self.auto_close_timer.timeout.connect(self.close)
        self.auto_close_timer.start(4000)

    def update_animation(self):
        self.angle += 0.3
        if self.angle > 2 * math.pi:
            self.angle = 0
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Animation: dùng sin để tạo hiệu ứng nhịp nhàng
        pulse = self.pulse_amplitude * math.sin(self.angle)
        radius = self.base_radius + pulse

        center = self.rect().center()
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(255, 255, 255, 160))  # Trắng mờ
        painter.drawEllipse(center, radius, radius)

class MapDisplayFrame(QFrame):
    def __init__(self, parent=None, on_microphone_click=None, route_pub=None):
        super().__init__(parent)
        # Prefer ROS camera so perception + UI share the same device.
        self.use_ros_camera = True
        self.face_ready = False
        self.latest_ros_frame = None
        self._ros_lock = threading.Lock()
        self.cap = None
        if not self.use_ros_camera:
            self.cap = cv2.VideoCapture(0)   # 0 = webcam, CSI có thể là 1
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.setStyleSheet("background-color: white; border-radius: 20px;")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # ==== Tạo stacked layout chứa 2 trang ====
        self.stacked_layout = QStackedLayout(self)
        self.route_pub = route_pub     

        # === Trang chính ===
        self.main_widget = QWidget()
        main_layout = QVBoxLayout(self.main_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(0)

        main_layout.addStretch(1)

        self.mic_button = QPushButton()
        self.mic_button.setIcon(QIcon(f"{pkg_share}/assets/icon/micro.png"))
        self.mic_button.setIconSize(QSize(50, 50))
        self.mic_button.setFixedSize(100, 100)
        self.mic_button.setStyleSheet("""
            QPushButton {
                background-color: #eee;
                border-radius: 50px;
                border: none;
            }
            QPushButton:hover {
                background-color: #ddd;
            }
        """)
        main_layout.addWidget(self.mic_button, alignment=Qt.AlignHCenter)

        if on_microphone_click:
            self.mic_button.clicked.connect(on_microphone_click)

        main_layout.addStretch(1)

        bottom_widget = QWidget()
        bottom_layout = QHBoxLayout(bottom_widget)
        bottom_layout.setContentsMargins(0, 0, 0, 0)
        bottom_layout.setSpacing(50)
        bottom_layout.setAlignment(Qt.AlignCenter)

        self.route_buttons = []
        buttons = [
            ("Khu C", self.khu_c_clicked),
            ("Khu D", self.khu_d_clicked),
            ("Tòa Trung Tâm", self.trung_tam_clicked),
            ("Tòa Việt Đức", self.viet_duc_clicked),
            ("Xưởng Gỗ", self.xuong_go_clicked),
            ("Maker_Space", self.ms_clicked)
        ]

        for text, callback in buttons:
            btn = QPushButton(text)
            btn.setFixedSize(120, 60)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #f5f5f5;
                    border: none;
                    border-radius: 15px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #eee;
                }
            """)
            btn.clicked.connect(callback)
            bottom_layout.addWidget(btn)
            self.route_buttons.append(btn)

        main_layout.addWidget(bottom_widget)

        # ==== Thêm nút chuyển sang trang camera ====
        self.camera_btn = QPushButton("Camera View")
        self.camera_btn.setFixedHeight(60)
        self.camera_btn.setFixedSize(120, 60)
        self.camera_btn.setStyleSheet("""
            QPushButton {
                background-color: #f5f5f5;
                color: black;
                border-radius: 15px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #eee;
            }
        """)
        self.camera_btn.clicked.connect(self.show_camera_view)
        main_layout.addWidget(self.camera_btn, alignment=Qt.AlignHCenter)

        self.stacked_layout.addWidget(self.main_widget)

        # === Trang camera view ===
        self.camera_widget = QWidget()
        cam_layout = QVBoxLayout(self.camera_widget)
        cam_layout.setAlignment(Qt.AlignCenter)

        self.camera_label = QLabel("Đang chờ ảnh từ camera...")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setFixedSize(1080, 720)
        self.camera_label.setStyleSheet("border: 5px solid #fff; border-radius: 10px;")

        cam_layout.addWidget(self.camera_label)  # <-- Sửa dòng này

        self.back_button = QPushButton("Quay lại")
        self.back_button.setFixedSize(140, 60)
        self.back_button.setStyleSheet("""
            QPushButton {
                background-color: #f5f5f5;
                color: black;
                border-radius: 15px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #eee;
            }
        """)
        self.back_button.clicked.connect(self.show_main_view)
        cam_layout.addWidget(self.back_button, alignment=Qt.AlignCenter)
        self.stacked_layout.addWidget(self.camera_widget)
        self.play_obj = None
        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_camera_image)
        self.camera_timer.start(60)  # cập nhật mỗi 50ms (20 fps)

        self.init_face_detection()

    def send_route_request(self, checkpoint_code: str):
        """
        Gửi yêu cầu lập route đến checkpoint_code (ví dụ: 'C', 'D', 'MS', 'G', ...)
        """
        if not self.face_ready:
            print("[MapDisplayFrame] Face not recognized yet, route request blocked")
            return
        if self.route_pub is None:
            print("[MapDisplayFrame] route_pub is None, không gửi được request")
            return
        msg = String()
        msg.data = checkpoint_code
        self.route_pub.publish(msg)
        print(f"[MapDisplayFrame] 📨 Sent route_plan request: {checkpoint_code}")

    def init_face_detection(self):
        mydict = ['Co Nguyet', 'Thay Giang', 'Thay Ha', 'Thay Hai','Thay Phong', 'Thay Thanh', 'Thay Trung']  # Ví dụ thêm tên
        try:
            self.face_detector = FaceRecognition(
                face_detection_model  =f'{pkg_share}/assets/FACE_DETECTION/model2/face_detection_yunet_2023mar.onnx',
                face_recognition_model=f'{pkg_share}/assets/FACE_DETECTION/model2/face_recognition_sface_2021dec.onnx',
                svc_path              =f'{pkg_share}/assets/FACE_DETECTION/model2/svc_model.pkl',
                mydict=mydict
            )
        except Exception as exc:
            self.face_detector = None
            self.face_timer = None
            print(f"[MapDisplayFrame] Face detection disabled: {exc}")
            self._set_route_buttons_enabled(True)
            self.face_ready = True
            return

        self.face_detect_frame_count = 0
        self.face_detect_triggered = False
        
        self.player = QMediaPlayer()

        # self.cap = cv2.VideoCapture(0)
        # self.cap.set(cv2.CAP_PROP_FPS, 60)  # Điều chỉnh FPS
        
        # Ngay khi khởi động thì hiển thị giao diện camera
        self.show_camera_view()

        # Bắt đầu phát hiện khuôn mặt
        self.face_timer = QTimer(self)
        self.face_timer.timeout.connect(self.process_face_detection)
        self.face_timer.start(200)  # 20 fps
        self._set_route_buttons_enabled(False)

    def process_face_detection(self):
        if self.face_detector is None:
            return
        if self.use_ros_camera:
            with self._ros_lock:
                frame = None if self.latest_ros_frame is None else self.latest_ros_frame.copy()
            if frame is None:
                return
        else:
            if self.cap is None or not self.cap.isOpened():
                return
            ret, frame = self.cap.read()
            if not ret:
                return
            frame = frame.copy()
        results = self.face_detector.detect_and_recognize(frame)
        if results:
            self.face_detect_frame_count += 1
            names_detected = []
            for coords, name in results:
                x, y, w_box, h_box = map(int, coords[:4])
                cv2.rectangle(frame, (x, y), (x + w_box, y + h_box), (0, 255, 0), 2)
                cv2.putText(frame, name, (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                names_detected.append(name)

            # ... logic đếm frame + play_audio ...
    
            if self.face_detect_frame_count >= 20 and not self.face_detect_triggered:
                self.face_detect_triggered = True
                self.face_timer.stop()
                self.camera_timer.start(120) 
                self.stacked_layout.setCurrentWidget(self.main_widget)
                self.show_main_view()
                self.face_ready = True
                self._set_route_buttons_enabled(True)

                # Ưu tiên người quen
                if "Thay Giang" in names_detected:
                    self.play_audio("Thay Giang")
                elif "Thay Hai" in names_detected:
                    self.play_audio("Thay Hai")
                elif "Thay Thanh" in names_detected:
                    self.play_audio("Thay Thanh")
                elif "Thay Ha" in names_detected:
                    self.play_audio("Thay Ha")
                elif "Co Nguyet" in names_detected:
                    self.play_audio("Co Nguyet")
                elif "Thay Trung" in names_detected:
                    self.play_audio("Thay Trung")
                elif "Thay Phong" in names_detected:
                    self.play_audio("Thay Phong")
                else:
                    self.play_audio("default")
                    print("_____________________________________________")  # âm thanh chào chung

        else:
            self.face_detect_frame_count = 0

        # Hiển thị ảnh camera lên QLabel
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (1080, 720))
        qimg = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.shape[1] * 3, QImage.Format_RGB888)
        self.camera_label.setPixmap(QPixmap.fromImage(qimg))

    def stop_audio(self):
        if self.play_obj and self.play_obj.is_playing():
            self.play_obj.stop()
            self.play_obj = None
    def play_audio(self, name):
        def play_and_then():
            if name == "Thay Giang":
                file_path = f"{pkg_share}/assets/voice/hieugiang.wav"
            elif name == "Thay Hai":
                file_path = f"{pkg_share}/assets/voice/thanhhai.wav"
            elif name == "Thay Thanh":
                file_path = f"{pkg_share}/assets/voice/dinhthanh.wav"
            elif name == "Thay Ha":
                file_path = f"{pkg_share}/assets/voice/myha.wav"
            elif name == "Co Nguyet":
                file_path = f"{pkg_share}/assets/voice/coNguyet.wav"
            elif name == "Thay Phong":
                file_path = f"{pkg_share}/assets/voice/thayPhong.wav"
            elif name == "Thay Trung":
                file_path = f"{pkg_share}/assets/voice/thayTrung.wav"
            else:
                file_path = f"{pkg_share}/assets/sound/output.wav"

            wave_obj = sa.WaveObject.from_wave_file(file_path)
            self.play_obj = wave_obj.play()
            self.play_obj.wait_done()

        threading.Thread(target=play_and_then, daemon=True).start()
        
    def show_camera_view(self):
        self.stacked_layout.setCurrentWidget(self.camera_widget)

    def show_main_view(self):
        self.stacked_layout.setCurrentWidget(self.main_widget)

    def update_camera_image(self):
        if self.use_ros_camera:
            with self._ros_lock:
                img = None if self.latest_ros_frame is None else self.latest_ros_frame.copy()
            if img is None:
                self.camera_label.setText("Đang chờ ảnh từ ROS /camera/image_raw ...")
                return
        else:
            if self.cap is None or not self.cap.isOpened():
                self.camera_label.setText("Camera trực tiếp chưa sẵn sàng")
                return
            ret, img = self.cap.read()
            if not ret:
                self.camera_label.setText("Không lấy được ảnh từ camera trực tiếp")
                return

        img = cv2.resize(img, (1080, 720))
        rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        qimg = QImage(rgb_img.data, w, h, ch * w, QImage.Format_RGB888)
        self.camera_label.setPixmap(QPixmap.fromImage(qimg))


        
    # ===== Dummy callback functions =====
    def khu_c_clicked(self):
        self.stop_audio()
        self.switch_to_ros_camera() 
        self.send_route_request("C")
        threading.Thread(target=self.run_task, args=("khu_c",), daemon=True).start()

    def khu_d_clicked(self):
        self.stop_audio()
        self.switch_to_ros_camera() 
        self.send_route_request("D")
        threading.Thread(target=self.run_task, args=("khu_d",), daemon=True).start()

    def trung_tam_clicked(self):
        self.stop_audio()
        self.switch_to_ros_camera() 
        self.send_route_request("TTT")   # hoặc "STT" tùy CHECKPOINTS
        threading.Thread(target=self.run_task, args=("trung_tam_truoc",), daemon=True).start()

    def viet_duc_clicked(self):
        self.stop_audio()
        self.switch_to_ros_camera() 
        self.send_route_request("VD")
        threading.Thread(target=self.run_task, args=("viet_duc",), daemon=True).start()

    def xuong_go_clicked(self):
        self.stop_audio()
        self.switch_to_ros_camera() 
        self.send_route_request("G")
        threading.Thread(target=self.run_task, args=("go",), daemon=True).start()

    def ms_clicked(self):
        self.stop_audio()
        self.switch_to_ros_camera() 
        self.send_route_request("MS")
        threading.Thread(target=self.run_task, args=("maker_space",), daemon=True).start()

    def run_task(self, location):
        play_obj = wave_obj.play()
        self.show_camera_view()
        cf.record = 0

    def closeEvent(self, event):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        event.accept()

    def set_ros_frame(self, frame_bgr: np.ndarray):
        with self._ros_lock:
            self.latest_ros_frame = frame_bgr

    def switch_to_ros_camera(self):
        # chuyển sang hiển thị ROS image
        self.use_ros_camera = True

        # release cam trực tiếp để ROS node được quyền mở /dev/video*
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
            self.cap = None

    def switch_to_direct_camera(self):
        # (nếu sau này cần quay lại cam trực tiếp)
        self.use_ros_camera = False
        if self.cap is None or (hasattr(self.cap, "isOpened") and not self.cap.isOpened()):
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            self.cap.set(cv2.CAP_PROP_FPS, 30)

    def _set_route_buttons_enabled(self, enabled: bool):
        for btn in self.route_buttons:
            btn.setEnabled(enabled)

    def shutdown(self):
        if getattr(self, "camera_timer", None) is not None:
            self.camera_timer.stop()
        if getattr(self, "face_timer", None) is not None and self.face_timer.isActive():
            self.face_timer.stop()
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()

class AutonomousCarUI(QWidget):
    def __init__(self):
        super().__init__()
        self._closing = False
        self.current_lat = None
        self.current_lon = None
        self.current_heading_deg = None
        self.current_speed = 0.0
        self.current_rtk_status = "No Fix"
        self.obstacles = np.array([[999, 999]])
        self.tx = []    
        self.ty = []     
        self.tyaw = []  
        self.paths = []        
        self.optimal_path = None 
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # rad
        # ===== Node ROS2 + publisher trước =====
        self.node = Node('visualization')
        self.node.declare_parameter("origin_lat", DEFAULT_ORIGIN_LAT)
        self.node.declare_parameter("origin_lon", DEFAULT_ORIGIN_LON)
        self.origin_lat = self.node.get_parameter("origin_lat").value
        self.origin_lon = self.node.get_parameter("origin_lon").value
        self.bridge = CvBridge()
        self.subscription = self.node.create_subscription(
            Image,
            '/perception/image',          # <-- ảnh đã vẽ bbox/overlay
            self.listener_callback,
            10)

        self.data_timer = QTimer()
        self.data_timer.timeout.connect(self.ros_spin_callback)
        self.data_timer.start(50)

        self.route_pub = self.node.create_publisher(
            String,
            '/route_plan/request',
            10
        )
        self.paths = []
        self.optimal_path = None

        # Nhận quỹ đạo tối ưu từ Frenet planner
        self.optimal_path_sub = self.node.create_subscription(
            Path,
            '/frenet/optimal_path',
            self.optimal_path_callback,
            10
        )

        # Nhận toàn bộ candidate paths để vẽ mờ mờ phía dưới
        self.candidate_paths_sub = self.node.create_subscription(
            MarkerArray,
            '/frenet/candidate_paths',
            self.candidate_paths_callback,
            10
        )

        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/odometry',
            self.odometry_callback,
            10
        )

        self.gps_rtk_sub = self.node.create_subscription(
            String,
            '/gps/rtk_status',
            self.gps_rtk_status_callback,
            10
        )
        self.ref_path_sub = self.node.create_subscription(
            Path,
            '/frenet/reference_path',
            self.ref_path_callback,
            10
        )
        # ===== Perception outputs =====
        self.perc_obs_sub = self.node.create_subscription(
            Float32MultiArray,
            '/perception/obstacles',
            self.perception_obstacles_callback,
            10
        )

        self.perc_person_sub = self.node.create_subscription(
            Float32MultiArray,
            '/perception/persons',
            self.perception_persons_callback,
            10
        )

        # (optional) debug seg
        self.perc_seg_steer_sub = self.node.create_subscription(
            Float32,
            '/perception/seg_steer',
            self.perception_seg_steer_callback,
            10
        )

        self.perc_intersection_sub = self.node.create_subscription(
            Int32,
            '/perception/is_intersection',
            self.perception_intersection_callback,
            10
        )

        self.initUI()

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.right_frame.set_ros_frame(frame)

        
    def ref_path_callback(self, msg: Path):
        self.tx = [p.pose.position.x for p in msg.poses]
        self.ty = [p.pose.position.y for p in msg.poses]

        # Tính tyaw từ đạo hàm quỹ đạo
        self.tyaw = []
        for i in range(len(self.tx) - 1):
            dx = self.tx[i+1] - self.tx[i]
            dy = self.ty[i+1] - self.ty[i]
            self.tyaw.append(math.atan2(dy, dx))
        if len(self.tx) > 1:
            self.tyaw.append(self.tyaw[-1])  # điểm cuối cùng
        else:
            self.tyaw = [0.0]


    def initUI(self):
        self.setWindowTitle("ISLAB DRIVE TEAM")

        # Lấy kích thước màn hình đúng cách
        screen_geometry = QApplication.primaryScreen().geometry()
        screen_width, screen_height = screen_geometry.width(), screen_geometry.height()
        print(screen_width, screen_height)
        self.setGeometry(100, 100, screen_width, screen_height)

        self.setStyleSheet("background-color: dimgray;")

        splitter = QSplitter(Qt.Horizontal)

        # Tạo khung bên trái (xe và vật cản)
        self.left_frame = QFrame()
        left_layout = QVBoxLayout(self.left_frame)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(0)

        self.status_frame    = VehicleStatusFrame(self.left_frame)
        self.vehicle_display = VehicleDisplayFrame(self.left_frame)

        left_layout.addWidget(self.status_frame)
        left_layout.addWidget(self.vehicle_display)

        # Tạo khung bên phải (bản đồ GPS)
        self.right_frame = MapDisplayFrame(self, self.show_listening, route_pub=self.route_pub)
        
        # Cập nhật lại tỷ lệ phân chia nếu cần thiết
        splitter.addWidget(self.left_frame)
        splitter.addWidget(self.right_frame)
        splitter.setSizes([screen_width // 3, screen_width * 2 // 3])

        # Bố cục chính
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(splitter)
        self.setLayout(layout)

    def show_listening(self):
        threading.Thread(target=lambda: micro_sound.play().wait_done(), daemon=True).start()
        self.right_frame.stop_audio()
        self.listening_dialog = ListeningDialog(self.right_frame, self)
        self.listening_dialog.show()


        # done1 = threading.Event()
        # done2 = threading.Event()

        # def task1_wrapper():
        #     task1()
        #     done1.set()

        # def task2_wrapper():
        #     task2()
        #     done2.set()

        # Chạy 2 task song song
        # threading.Thread(target= task1_wrapper, daemon=True).start()
        # threading.Thread(target= task2_wrapper, daemon=True).start()

        # def check_if_done():
            
        #     if done1.is_set() and done2.is_set():
        #         cf.record = 0
        #         QTimer.singleShot(0, self.close_listening_and_show_map)
        #         self.right_frame.show_camera_view()
                
        #     else:
        #         QTimer.singleShot(10, check_if_done)

        # check_if_done()

    @pyqtSlot()
    def close_listening_and_show_map(self):
        self.listening_dialog.close()
        self.show_map()

    def show_map(self):
        self.right_frame.show()
    
    def ros_spin_callback(self):
        if self._closing or not rclpy.ok():
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0)
        except Exception:
            self._closing = True
            if getattr(self, "data_timer", None) is not None:
                self.data_timer.stop()
            return
        # self.count += 1
        self.update_data()

    def update_data(self):
        # Pose đã convert từ GPS
        x   = self.x
        y   = self.y
        yaw = self.yaw

        # Vẽ
        self.vehicle_display.plot_canvas.update_vehicle_position(
            x, y, yaw,
            paths=self.paths,
            optimal_path=self.optimal_path,
            tx=self.tx,
            ty=self.ty,
            tyaw=self.tyaw,
        )
        self.vehicle_display.plot_canvas.update_obstacles(self.obstacles)

        # Cập nhật panel status
        self.status_frame.update_gps_status(self.current_rtk_status)
        self.status_frame.update_vehicle_status(self.current_speed )

    
    def closeEvent(self, event):
        self._closing = True
        if getattr(self, "data_timer", None) is not None:
            self.data_timer.stop()
        if getattr(self, "right_frame", None) is not None:
            self.right_frame.shutdown()
        if getattr(self, "vehicle_display", None) is not None:
            if getattr(self.vehicle_display, "plot_canvas", None) is not None:
                self.vehicle_display.plot_canvas.shutdown()
        self.node.destroy_node()
        self.node.get_logger().info('ROS 2 Node destroyed.')
        event.accept()

    def optimal_path_callback(self, msg: Path):
        xs = [p.pose.position.x for p in msg.poses]
        ys = [p.pose.position.y for p in msg.poses]

        # Tính yaw từ đạo hàm quỹ đạo
        yaws = []
        n = len(xs)
        if n < 2:
            yaws = [0.0] * n
        else:
            for i in range(n - 1):
                dx = xs[i+1] - xs[i]
                dy = ys[i+1] - ys[i]
                yaws.append(math.atan2(dy, dx))
            yaws.append(yaws[-1])

        # Lưu lại thành SimplePath
        self.optimal_path = SimplePath(xs, ys, yaw=yaws)


    def candidate_paths_callback(self, msg: MarkerArray):
        paths = []
        for m in msg.markers:
            xs = [p.x for p in m.points]
            ys = [p.y for p in m.points]
            paths.append(SimplePath(xs, ys))
        self.paths = paths

    # ========== ODOMETRY CALLBACK ==========

    def odometry_callback(self, msg: Odometry):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.current_heading_deg = math.degrees(self.yaw) + 90.0
        self.current_speed = float(msg.twist.twist.linear.x) * 3.6
        self.current_lat, self.current_lon = latlon_from_xy(
            self.x, self.y, self.origin_lat, self.origin_lon
        )

    def gps_rtk_status_callback(self, msg: String):
        self.current_rtk_status = msg.data

    def update_xy_yaw_from_gps(self):
        """
        Backward-compatible helper for GPS-only mode.
        """
        try:
            x, y = lat_lon_to_xy(float(self.current_lat), float(self.current_lon))
            yaw_deg_map = convert_yaw(float(self.current_heading_deg), yaw_offset=90.0)
            yaw = math.radians(yaw_deg_map)
            self.x = x
            self.y = y
            self.yaw = yaw
        except Exception as e:
            self.node.get_logger().warn(f"update_xy_yaw_from_gps error: {e}")
            
    def vehicle_to_map(self, x_v, z_v):
        """
        x_v: right(+), z_v: forward(+)
        -> map (x,y) using current pose (self.x,self.y,self.yaw)
        """
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)

        x_map = self.x + z_v * cy + x_v * sy
        y_map = self.y + z_v * sy - x_v * cy
        return x_map, y_map


    def _decode_pairs(self, data):
        """data: [x1,z1,x2,z2,...] -> list[(x,z)]"""
        if data is None:
            return []
        n = len(data) // 2
        pairs = []
        for i in range(n):
            pairs.append((float(data[2*i]), float(data[2*i+1])))
        return pairs

    def perception_obstacles_callback(self, msg: Float32MultiArray):
        pairs_v = self._decode_pairs(msg.data)  # vehicle frame (x,z)
        # nếu chưa có pose GPS thì bỏ qua để tránh vẽ sai
        if self.x is None or self.y is None:
            return

        obs_map = []
        for x_v, z_v in pairs_v:
            x_m, y_m = self.vehicle_to_map(x_v, z_v)
            obs_map.append([x_m, y_m])

        if len(obs_map) == 0:
            self.obstacles = np.array([[999, 999]], dtype=np.float32)
        else:
            self.obstacles = np.array(obs_map, dtype=np.float32)

    def perception_persons_callback(self, msg: Float32MultiArray):
        # nếu bạn muốn vẽ persons riêng thì tạo self.persons và vẽ thêm.
        # tạm thời mình gộp persons vào obstacles để thấy trên map.
        pairs_v = self._decode_pairs(msg.data)
        if self.x is None or self.y is None:
            return

        prs_map = []
        for x_v, z_v in pairs_v:
            x_m, y_m = self.vehicle_to_map(x_v, z_v)
            prs_map.append([x_m, y_m])

        # nếu bạn muốn gộp vào obstacles:
        if len(prs_map) > 0:
            if self.obstacles is None or len(self.obstacles) == 0 or (self.obstacles.shape == (1,2) and self.obstacles[0,0] == 999):
                self.obstacles = np.array(prs_map, dtype=np.float32)
            else:
                self.obstacles = np.vstack([self.obstacles, np.array(prs_map, dtype=np.float32)])

    def perception_seg_steer_callback(self, msg: Float32):
        # optional: hiển thị lên status_frame nếu muốn
        self.seg_steer = float(msg.data)

    def perception_intersection_callback(self, msg: Int32):
        # optional: hiển thị/logic UI nếu muốn
        self.is_intersection = int(msg.data)
