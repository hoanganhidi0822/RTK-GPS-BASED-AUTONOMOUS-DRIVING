import sys
import random
import math
import threading
import cv2
import numpy as np
import simpleaudio as sa
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.transforms import Affine2D
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5.QtCore import Qt, QTimer, QSize, QRect, QPoint, QMetaObject, pyqtSlot
from PyQt5.QtGui import QIcon, QPainter, QColor, QFont, QPixmap, QImage
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QFrame, QPushButton,
    QSizePolicy, QLabel, QDialog, QStackedLayout, QSplitter
)

from FACE_DETECTION.face import FaceRecognition
from Assistance_Astar.main_assistance import *
# from frenet_optimal_trajectory import *
import config as cf
from pydub import AudioSegment
from pydub.playback import play

wave_obj = sa.WaveObject.from_wave_file("VISUALIZATION/sound/click.wav")
micro_sound = sa.WaveObject.from_wave_file("VISUALIZATION/sound/micro_sound.wav")

cf.ob  = np.array([[3,3]])
ob =np.array([[]])

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
cf.speed = 0

cf.det_image = np.zeros((480, 640, 3))
cf.depth_image = np.zeros((480, 640, 3))


MAX_ROAD_WIDTH = 6
# Vehicle parameters
LENGTH = 2.2  # total vehicle length
WIDTH = 0.6 # total vehicle width
BACKTOWHEEL = 0.15  # distance from rear to vehicle center
WHEEL_LEN = 0.3  # wheel length
WHEEL_WIDTH = 0.2  # wheel width
TREAD = 0.7 # width between left and right wheels
WB = 1.8  # wheel base: distance between front and rear axles

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
def plot_car(x, y, yaw, ax, icon_path="VISUALIZATION/icon/car.png", zoom=0.12):
    """
    Vẽ xe bằng icon PNG tại vị trí (x, y) với góc yaw.
    """
    # Load và xoay icon
    car_img = mpimg.imread(icon_path)
    rotated_img = rotate_image(car_img, yaw)
    
    # Tạo OffsetImage từ ảnh đã xoay
    imagebox = OffsetImage(rotated_img, zoom=zoom)

    # Tạo AnnotationBbox gắn ảnh vào tọa độ (x, y)
    ab = AnnotationBbox(imagebox, (x, y-1), frameon=False)
    ax.add_artist(ab)

    # Tùy chọn vẽ vị trí tâm
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

        self.obstacle_icon = mpimg.imread("VISUALIZATION/icon/obstacle.png")
        self.obstacle_img_raw = OffsetImage(self.obstacle_icon, zoom=0.15)
        self.vehicle_icon = mpimg.imread("VISUALIZATION/icon/car.png")
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
        self.timer.start(200)

    def update_plot(self):
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
        inv_yaw = self.filtered_inv_yaw

        tx_rot, ty_rot = rotate_array(self.tx, self.ty, inv_yaw)
        self.path_lines.append(self.ax.plot(tx_rot[1:], ty_rot[1:], "white", linewidth=0.5)[0])

        if not isinstance(self.tyaw, np.ndarray):
            self.tyaw = np.array(self.tyaw)

        if len(self.left_x) != len(self.tx):
            MAX_ROAD_WIDTH = 6.0
            self.left_x  = self.tx + (MAX_ROAD_WIDTH * 0.75 + 0.5) * np.cos(self.tyaw + np.pi / 2)
            self.left_y  = self.ty + (MAX_ROAD_WIDTH * 0.75 + 0.5) * np.sin(self.tyaw + np.pi / 2)
            self.mid_x   = self.tx + (MAX_ROAD_WIDTH * 0.25)       * np.cos(self.tyaw + np.pi / 2)
            self.mid_y   = self.ty + (MAX_ROAD_WIDTH * 0.25)       * np.sin(self.tyaw + np.pi / 2)
            self.right_x = self.tx + (MAX_ROAD_WIDTH / 4 + 0.5)    * np.cos(self.tyaw - np.pi / 2)
            self.right_y = self.ty + (MAX_ROAD_WIDTH / 4 + 0.5)    * np.sin(self.tyaw - np.pi / 2)

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
        self.fig.tight_layout()
        self.ax.figure.canvas.draw_idle()

    def update_vehicle_position(self, x, y, yaw):
        self.vehicle_pos = (x, y)
        self.vehicle_yaw = yaw
        self.paths = cf.paths
        self.tx = cf.tx
        self.ty = cf.ty
        self.tyaw = cf.tyaw
        self.optimal_path = cf.optimal_path

    def update_obstacles(self, new_obstacles):
        self.obstacles = cf.ob
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
        pixmap = QPixmap("VISUALIZATION/icon/gps.png").scaled(30, 30, Qt.KeepAspectRatio, Qt.SmoothTransformation)
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

        # Timer cập nhật thông tin
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status_from_cf)
        self.status_timer.start(1000)  # cập nhật mỗi 1s

    def update_status_from_cf(self):
        # Cập nhật trạng thái GPS
        self.update_gps_status(cf.rtk_status)

        # Cập nhật trạng thái xe
        self.update_vehicle_status(cf.speed)

    def update_gps_status(self, status_text):
        self.gps_status.setText(status_text)

        if "Fixed" in status_text:
            color = "lightgreen"
        elif "Float" in status_text:
            color = "orange"
        else:
            color = "red"

        self.gps_status.setStyleSheet(
            f"font-size: 24px; color: {color}; margin-left: 6px;font-weight: bold;"
        )

    def update_vehicle_status(self, status_text):

        if isinstance(status_text, (int, float)):

            self.vehicle_status_label.setStyleSheet("""
                font-size: 70px;
                font-weight: bold;
                color: #333;
            """)
        else:
            self.vehicle_status_label.setStyleSheet("""
                font-size: 20px;
                font-weight: bold;
                color: red;
            """)

        self.vehicle_status_label.setText(f"{status_text}")



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
    def __init__(self, parent=None, on_microphone_click=None):
        super().__init__(parent)
        self.setStyleSheet("background-color: white; border-radius: 20px;")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # ==== Tạo stacked layout chứa 2 trang ====
        self.stacked_layout = QStackedLayout(self)

        # === Trang chính ===
        self.main_widget = QWidget()
        main_layout = QVBoxLayout(self.main_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(0)

        main_layout.addStretch(1)

        self.mic_button = QPushButton()
        self.mic_button.setIcon(QIcon("VISUALIZATION/icon/micro.png"))
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

        buttons = [
            ("Khu C", self.khu_c_clicked),
            ("Khu D", self.khu_d_clicked),
            ("Tòa Trung Tâm", self.trung_tam_clicked),
            ("Tòa Việt Đức", self.viet_duc_clicked),
            ("Xưởng Gỗ", self.xuong_go_clicked),
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


        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_camera_image)
        # self.camera_timer.start(60)  # cập nhật mỗi 50ms (20 fps)

        self.init_face_detection()
    def init_face_detection(self):
        mydict = ['Thay Giang', 'Thay Ha', 'Thay Hai', 'Thay Thanh', 'Hoang Anh', 'Quoc Kha']  # Ví dụ thêm tên
        self.face_detector = FaceRecognition(
            face_detection_model  ='FACE_DETECTION/model1/face_detection_yunet_2023mar.onnx',
            face_recognition_model='FACE_DETECTION/model1/face_recognition_sface_2021dec.onnx',
            svc_path              ='FACE_DETECTION/model1/svc_model.pkl',
            mydict=mydict
        )

        self.face_detect_frame_count = 0
        self.face_detect_triggered = False
        
        self.player = QMediaPlayer()

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FPS, 60)  # Điều chỉnh FPS
        
        # Ngay khi khởi động thì hiển thị giao diện camera
        self.show_camera_view()

        # Bắt đầu phát hiện khuôn mặt
        self.face_timer = QTimer(self)
        self.face_timer.timeout.connect(self.process_face_detection)
        self.face_timer.start(200)  # 20 fps

    def process_face_detection(self):
        if not self.cap.isOpened() or self.face_detect_triggered:
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        results = self.face_detector.detect_and_recognize(frame)

        if results:
            self.face_detect_frame_count += 1
            names_detected = []

            for coords, name in results:
                x, y, w, h = coords[0], coords[1], coords[2], coords[3]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                names_detected.append(name)

            if self.face_detect_frame_count >= 20 and not self.face_detect_triggered:
                self.face_detect_triggered = True
                self.face_timer.stop()
                self.camera_timer.start(100) 
                self.stacked_layout.setCurrentWidget(self.main_widget)
                self.show_main_view()

                # Ưu tiên người quen
                if "Thay Giang" in names_detected:
                    self.play_audio("Thay Giang")
                elif "Thay Hai" in names_detected:
                    self.play_audio("Thay Hai")
                elif "Thay Thanh" in names_detected:
                    self.play_audio("Thay Thanh")
                elif "Thay Ha" in names_detected:
                    self.play_audio("Thay Ha")
                else:
                    self.play_audio("default")  # âm thanh chào chung

        else:
            self.face_detect_frame_count = 0

        # Hiển thị ảnh camera lên QLabel
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (1080, 720))
        qimg = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.shape[1] * 3, QImage.Format_RGB888)
        self.camera_label.setPixmap(QPixmap.fromImage(qimg))

    def play_audio(self, name):
        if self.cap.isOpened():
            self.cap.release()

        def play_and_then():
            if name == "Thay Giang":
                file_path = "VISUALIZATION/voice/hieugiang.mp3"
            elif name == "Thay Hai":
                file_path = "VISUALIZATION/voice/thanhhai.mp3"
            elif name == "Thay Thanh":
                file_path = "VISUALIZATION/voice/dinhthanh.mp3"
            elif name == "Thay Ha":
                file_path = "VISUALIZATION/voice/myha.mp3"
            else:
                file_path = "test/output.mp3"  # Âm thanh chung

            sound = AudioSegment.from_mp3(file_path)
            sound = sound.apply_gain(6) 
            play(sound)

        threading.Thread(target=play_and_then).start()

    def show_camera_view(self):

        self.stacked_layout.setCurrentWidget(self.camera_widget)

    def show_main_view(self):
        self.stacked_layout.setCurrentWidget(self.main_widget)

    def update_camera_image(self):
        if hasattr(cf, "image") and cf.image is not None:
            img = cf.image

            # Chuyển kiểu dữ liệu nếu cần
            if img.dtype != np.uint8:
                img = np.clip(img, 0, 255)
                img = img.astype(np.uint8)

            # Resize ảnh và chuyển sang RGB
            img = cv2.resize(img, (1080, 720))
            rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            h, w, ch = rgb_img.shape
            bytes_per_line = ch * w
            qimg = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.camera_label.setPixmap(QPixmap.fromImage(qimg))
        else:
            self.camera_label.setText("Không có ảnh từ camera.")
        


    # ===== Dummy callback functions =====
    def khu_c_clicked(self):
        threading.Thread(target=self.run_task, args=("khu_c",), daemon=True).start()

    def khu_d_clicked(self):
        threading.Thread(target=self.run_task, args=("khu_d",), daemon=True).start()

    def trung_tam_clicked(self):
        threading.Thread(target=self.run_task, args=("trung_tam_truoc",), daemon=True).start()

    def viet_duc_clicked(self):
        threading.Thread(target=self.run_task, args=("viet_duc",), daemon=True).start()

    def xuong_go_clicked(self):
        threading.Thread(target=self.run_task, args=("go",), daemon=True).start()

    def run_task(self, location):
        
        play_obj = wave_obj.play()
        play_obj.wait_done()
        task2(location)
        self.show_camera_view()
        cf.record = 0

class AutonomousCarUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
    
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
        self.right_frame = MapDisplayFrame(self, self.show_listening)
        
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

        # Tạo QTimer để cập nhật dữ liệu liên tục
        self.data_timer = QTimer()
        self.data_timer.timeout.connect(self.update_data)
        self.data_timer.start(200)

    def show_listening(self):
        # Hiển thị dialog NGAY LẬP TỨC
        play_micro = micro_sound.play()
        play_micro.wait_done()
        self.listening_dialog = ListeningDialog(self.right_frame, self)
        self.listening_dialog.show()

        done1 = threading.Event()
        done2 = threading.Event()

        def task1_wrapper():
            task1()
            done1.set()

        def task2_wrapper():
            task2()
            done2.set()

        # Chạy 2 task song song
        threading.Thread(target= task1_wrapper, daemon=True).start()
        threading.Thread(target= task2_wrapper, daemon=True).start()

        def check_if_done():
            
            if done1.is_set() and done2.is_set():
                cf.record = 0
                QTimer.singleShot(0, self.close_listening_and_show_map)
                self.right_frame.show_camera_view()
                
            else:
                QTimer.singleShot(10, check_if_done)

        check_if_done()

    @pyqtSlot()
    def close_listening_and_show_map(self):
        # Ẩn dialog và chuyển sang bản đồ
        self.listening_dialog.close()
        self.show_map()

    def show_map(self):
        """Ẩn ListeningDialog và hiển thị lại MapDisplayFrame"""
        self.right_frame.show()

    def update_data(self):

        ob =cf.ob
        x  = cf.x
        y  = cf.y
        yaw = cf.yaw
        self.tx   = cf.tx
        self.ty   = cf.ty 
        self.tyaw = cf.tyaw
        
        self.vehicle_display.plot_canvas.update_vehicle_position(x, y, yaw)
        self.vehicle_display.plot_canvas.update_obstacles(ob)

