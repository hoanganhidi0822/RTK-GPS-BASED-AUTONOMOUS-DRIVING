#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

# ===================== CONFIG =====================
SIM_MODE = True
STM32_PORT = "/dev/ttyUSB0"
STM32_BAUD = 115200

LOOKAHEAD_DISTANCE = 4.5
WHEELBASE_L = 1.8
FIXED_SPEED_CMD = 0
BRAKE_STATE = 0

CONTROL_HZ = 10.0
STEER_LPF_ALPHA = 0.7

PRINT_EVERY = 20   # in mỗi 10 vòng (~0.5s nếu 20Hz)
# ================================================

# ====== IMPORT ĐÚNG THEO PACKAGE CỦA BẠN ======
# Nếu file control_node.py nằm trong cùng package với Frenet.py -> dùng: from .Frenet import ...
# Nếu bạn đang thử "from Fot_node.Frenet import ..." thường sẽ SAI trong ROS2 package.
from autonomous_car.Fot_node.Frenet import pure_pursuit_control_frenet


class _OptimalPathAdapter:
    def __init__(self, xs, ys):
        self.x = xs
        self.y = ys


class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")
        self.last_path_time = None
        self.last_pose_time = None
        self.last_yaw_time  = None

        # -------- SUBS --------
        self.path_sub = self.create_subscription(Path, "/frenet/optimal_path", self.path_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, "/frenet/car_pose", self.pose_cb, 10)
        self.yaw_sub  = self.create_subscription(Float32, "/frenet/yaw", self.yaw_cb, 10)

        # -------- PUB (debug) --------

        # -------- STM32 --------
        self.stm32 = None
        if not SIM_MODE:
            from .communication import STM32
            self.stm32 = STM32(port=STM32_PORT, baudrate=STM32_BAUD)
            self.get_logger().info(f"✅ STM32 connected: {STM32_PORT} @ {STM32_BAUD}")
        else:
            self.get_logger().warn("🧪 SIM_MODE=True -> KHÔNG connect STM32, chỉ tính góc lái")

        # -------- STATE --------
        self.have_path = False
        self.have_pose = False
        self.have_yaw  = False

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # rad

        self.opt_path = None
        self.steer_filtered = 0.0

        self._tick = 0

        dt = 1.0 / float(CONTROL_HZ)
        self.timer = self.create_timer(dt, self.control_loop)

        self.get_logger().info(
            f"ControlNode started | Ld={LOOKAHEAD_DISTANCE}m | L={WHEELBASE_L}m | "
            f"SIM_MODE={SIM_MODE} | speed_cmd={FIXED_SPEED_CMD}"
        )


    def pose_cb(self, msg: PoseStamped):
        self.x = float(msg.pose.position.x)
        self.y = float(msg.pose.position.y)
        self.have_pose = True


    def yaw_cb(self, msg: Float32):
        self.yaw = float(msg.data)
        self.have_yaw = True


    def path_cb(self, msg: Path):
        if len(msg.poses) < 3:
            self.have_path = False
            self.opt_path = None
            return
        xs = [float(ps.pose.position.x) for ps in msg.poses]
        ys = [float(ps.pose.position.y) for ps in msg.poses]
        self.opt_path = _OptimalPathAdapter(xs, ys)
        self.have_path = True

    # ---------------- main loop ----------------
    def control_loop(self):
        self._tick += 1

        if not (self.have_path and self.have_pose and self.have_yaw):
            if self._tick % PRINT_EVERY == 0:
                self.get_logger().warn(
                    f"Waiting topics... path={self.have_path} pose={self.have_pose} yaw={self.have_yaw}"
                )
            return

        try:
            steer_deg, alpha = pure_pursuit_control_frenet(
                0.0, 0.0,
                self.opt_path,
                self.x, self.y, self.yaw,
                LOOKAHEAD_DISTANCE, WHEELBASE_L
            )

            self.steer_filtered = (
                STEER_LPF_ALPHA * float(self.steer_filtered) +
                (1.0 - STEER_LPF_ALPHA) * float(steer_deg)
            )

            self.get_logger().info(
                    f"steer_raw={steer_deg:.2f} -> steer_int={steer_deg} -> steer_out={self.steer_filtered} | "
                    f"alpha(rad)={alpha:.3f} | x={self.x:.2f} y={self.y:.2f} yaw={self.yaw:.3f} | "
                    f"path_pts={len(self.opt_path.x)}"
                )
            if (not SIM_MODE) and (self.stm32 is not None):
                self.stm32(
                    angle=int(round(self.steer_filtered)),                 # <-- truyền số nguyên
                    speed=int(FIXED_SPEED_CMD),
                    brake_state=int(BRAKE_STATE)
                )

        except Exception as e:
            self.get_logger().warn(f"[Control] error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
