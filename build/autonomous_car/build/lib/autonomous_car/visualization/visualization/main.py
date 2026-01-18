import rclpy
from rclpy.node import Node # Có thể không cần nếu không dùng Executor
import sys
from autonomous_car.visualization.assets.visualization import AutonomousCarUI

from PyQt5.QtWidgets import QApplication

# Loại bỏ class VisualizationNode

def main():
    # 1. Khởi tạo ROS 2
    rclpy.init() 
    
    # 2. Khởi tạo QApplication (cần trước khi tạo cửa sổ)
    app = QApplication(sys.argv)
    
    # 3. Khởi tạo cửa sổ GUI (và Node ROS 2 bên trong nó)
    window = AutonomousCarUI()
    window.show()
    
    # 4. Chạy vòng lặp GUI (Blocking call)
    # Vòng lặp này sẽ chạy mãi cho đến khi cửa sổ đóng.
    # Trong khi chạy, QTimer bên trong window sẽ gọi ros_spin_callback.
    exit_code = app.exec_()
    
    # 5. Đóng ROS 2 sau khi GUI đóng
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()