#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String, Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

# ====== YOUR FRENET MODULE ======
from .Frenet import (
    XY_WAYPOINTS_MAP,
    generate_target_course,
    lat_lon_to_xy,
    convert_yaw,
    cartesian_to_frenet,
    frenet_optimal_planning,
)

WAYPOINT_FILE = "/home/minh_tan/ros2_ws/src/autonomous_car/way/merged_waypoints.txt"
YAW_OFFSET_DEG = 90.0
SPEED_UNIT = "mps"    
TIMER_DT = 1/20     
DBG_EVERY = 10
MAX_RETRY = 1          # khuyên để 0~1 để giữ Hz ổn định

# ================== TICK-BASED SCHED ==================
# 100Hz / PLAN_EVERY = tần số planning + optimal_path
PLAN_EVERY = 1         
REF_EVERY  = 100      
VIS_EVERY  = 1       

# ================== YAW SMOOTH ==================
yaw_buffer = deque(maxlen=5)

def smoothed_yaw_deg(new_heading_deg: float) -> float:
    yaw_buffer.append(float(new_heading_deg))
    return sum(yaw_buffer) / len(yaw_buffer)

def yaw_to_quat(yaw_rad: float):
    half = yaw_rad * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class Fot_node(Node):
    def __init__(self):
        super().__init__("frenet_planner_node")

        # -------- SUBS --------
        self.fix_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_fix_callback, 10)
        self.heading_sub = self.create_subscription(Float32, "/gps/heading", self.heading_callback, 10)
        self.speed_sub = self.create_subscription(Float32, "/gps/speed", self.speed_callback, 10)
        self.route_done_sub = self.create_subscription(String, "/route_plan/done", self.route_done_callback, 10)

        self.obstacles = np.empty((0, 2), dtype=float)
        self.persons = np.empty((0, 2), dtype=float)

        self.obs_sub = self.create_subscription(
            Float32MultiArray, "/perception/obstacles", self.obstacles_callback, 10
        )
        self.person_sub = self.create_subscription(
            Float32MultiArray, "/perception/persons", self.persons_callback, 10
        )

        # -------- PUBS --------
        self.ref_path_pub = self.create_publisher(Path, "/frenet/reference_path", 10)
        self.opt_path_pub = self.create_publisher(Path, "/frenet/optimal_path", 10)

        self.paths_marker_pub = self.create_publisher(MarkerArray, "/frenet/candidate_paths", 10)
        self.ref_marker_pub = self.create_publisher(Marker, "/frenet/reference_marker", 10)
        self.opt_marker_pub = self.create_publisher(Marker, "/frenet/optimal_marker", 10)

        self.car_pose_pub = self.create_publisher(PoseStamped, "/frenet/car_pose", 10)
        self.yaw_pub = self.create_publisher(Float32, "/frenet/yaw", 10)

        # -------- STATE --------
        self.have_fix = False
        self.current_lat = None
        self.current_lon = None
        self.current_heading_deg = None

        # NOTE: raw/3.6 => raw là km/h, biến này là m/s
        self.current_speed_mps = 0.0

        # Frenet state (dùng state vòng trước để plan)
        self.c_speed = 0.0
        self.c_d = 0.0
        self.c_d_d = 0.0
        self.c_d_dd = 0.0
        self.s0 = 0.0

        # ref path
        self.tx = []
        self.ty = []
        self.tyaw = []
        self.csp = None

        self.ready = False
        self._dbg_count = 0
        self._tick = 0

        # ====== OPTIMAL PATH HZ LOG ======
        self._opt_last_pub_t = None
        self._opt_last_log_t = None
        self._opt_hz_ema = None
        self._opt_pub_count = 0

        # -------- ONLY ONE TIMER --------
        self.timer = self.create_timer(TIMER_DT, self.timer_callback)

        self.get_logger().info("✅ FrenetPlannerNode started: đợi /route_plan/done để load merged_waypoints.txt")
        self.get_logger().info(f"CFG: {WAYPOINT_FILE} | yaw_offset={YAW_OFFSET_DEG} | timer={1/TIMER_DT:.1f}Hz")
        self.get_logger().info(
            f"SCHED: plan~{(1/TIMER_DT)/PLAN_EVERY:.1f}Hz | "
            f"ref~{(1/TIMER_DT)/REF_EVERY:.2f}Hz | vis~{(1/TIMER_DT)/VIS_EVERY:.1f}Hz"
        )

    # ================= ROUTE DONE =================
    def route_done_callback(self, msg: String):
        self.get_logger().info(f"📩 /route_plan/done = {msg.data} -> load {WAYPOINT_FILE}")

        if not os.path.exists(WAYPOINT_FILE):
            self.get_logger().error(f"❌ waypoint file không tồn tại: {WAYPOINT_FILE}")
            self.ready = False
            return

        ok = self.load_waypoints_from_file(WAYPOINT_FILE)
        self.ready = bool(ok)
        if self.ready:
            self.get_logger().info("✅ Ready=True: bắt đầu Frenet planning + publish RViz")

    def load_waypoints_from_file(self, path: str) -> bool:
        try:
            wx, wy = XY_WAYPOINTS_MAP(path)
        except Exception as e:
            self.get_logger().error(f"❌ XY_WAYPOINTS_MAP lỗi: {e}")
            return False

        if wx is None or wy is None or len(wx) < 2:
            self.get_logger().error("❌ merged_waypoints.txt quá ít điểm")
            return False

        try:
            self.tx, self.ty, self.tyaw, _, self.csp = generate_target_course(wx, wy)
        except Exception as e:
            self.get_logger().error(f"❌ generate_target_course lỗi: {e}")
            return False

        self.c_speed = 0.0
        self.c_d = 0.0
        self.c_d_d = 0.0
        self.c_d_dd = 0.0
        self.s0 = 0.0

        self.get_logger().info(
            f"Loaded ref path: {len(self.tx)} pts | csp_len={len(self.csp.s) if self.csp else -1}"
        )
        return True

    # ================= GPS CALLBACKS =================
    def gps_fix_callback(self, msg: NavSatFix):
        self.current_lat = float(msg.latitude)
        self.current_lon = float(msg.longitude)
        self.have_fix = True

    def heading_callback(self, msg: Float32):
        self.current_heading_deg = float(msg.data)

    def speed_callback(self, msg: Float32):
        raw = float(msg.data)
        self.current_speed_mps = raw / 3.6

    # ================= OBSTACLES CALLBACKS =================
    def _parse_xz_array(self, data_list):
        # data_list = [x1,z1,x2,z2,...]
        if data_list is None:
            return np.empty((0, 2), dtype=float)

        n = len(data_list)
        if n < 2:
            return np.empty((0, 2), dtype=float)

        if n % 2 != 0:
            data_list = data_list[:-1]
            n -= 1

        arr = np.array(data_list, dtype=float).reshape(-1, 2)
        mask = np.isfinite(arr).all(axis=1)
        return arr[mask]

    def obstacles_callback(self, msg: Float32MultiArray):
        self.obstacles = self._parse_xz_array(msg.data)

    def persons_callback(self, msg: Float32MultiArray):
        self.persons = self._parse_xz_array(msg.data)

    # ================= OPTIMAL PATH HZ LOG =================
    def log_optimal_path_hz(self):
        now = self.get_clock().now()

        if self._opt_last_pub_t is None:
            self._opt_last_pub_t = now
            self._opt_last_log_t = now
            return

        dt = (now - self._opt_last_pub_t).nanoseconds / 1e9
        if dt <= 0.0:
            return

        inst_hz = 1.0 / dt

        # EMA smoothing cho Hz (mượt)
        ema_alpha = 0.2
        if self._opt_hz_ema is None:
            self._opt_hz_ema = inst_hz
        else:
            self._opt_hz_ema = ema_alpha * inst_hz + (1.0 - ema_alpha) * self._opt_hz_ema

        self._opt_last_pub_t = now
        self._opt_pub_count += 1

        # log mỗi ~1s
        if self._opt_last_log_t is None:
            self._opt_last_log_t = now

        if (now - self._opt_last_log_t).nanoseconds / 1e9 >= 1.0:
            self.get_logger().info(
                f"[HZ] /frenet/optimal_path ~ {self._opt_hz_ema:.1f} Hz "
                f"(inst {inst_hz:.1f}) | PLAN_EVERY={PLAN_EVERY} | pub_count={self._opt_pub_count}"
            )
            self._opt_last_log_t = now

    # ================= Publish helpers =================
    def publish_car_pose_and_yaw(self, x, y, yaw):
        now = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(float(yaw))
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)

        self.car_pose_pub.publish(pose)

        yy = Float32()
        yy.data = float(yaw)
        self.yaw_pub.publish(yy)

    def publish_reference_path(self):
        if (not self.ready) or (self.csp is None) or (not self.tx):
            return

        now = self.get_clock().now().to_msg()

        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = "map"
        for xx, yy in zip(self.tx, self.ty):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(xx)
            pose.pose.position.y = float(yy)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.ref_path_pub.publish(path_msg)

        m = Marker()
        m.header.stamp = now
        m.header.frame_id = "map"
        m.ns = "reference"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.07
        m.color.a = 1.0
        m.color.r = 0.0
        m.color.g = 0.6
        m.color.b = 1.0
        for xx, yy in zip(self.tx, self.ty):
            p = Point()
            p.x = float(xx); p.y = float(yy); p.z = 0.0
            m.points.append(p)
        self.ref_marker_pub.publish(m)

    def publish_optimal_path(self, fp):
        now = self.get_clock().now().to_msg()

        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = "map"

        for xx, yy in zip(fp.x, fp.y):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(xx)
            pose.pose.position.y = float(yy)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.opt_path_pub.publish(path_msg)

    def publish_optimal_marker(self, fp):
        now = self.get_clock().now().to_msg()

        m = Marker()
        m.header.stamp = now
        m.header.frame_id = "map"
        m.ns = "optimal"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.10
        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0

        for xx, yy in zip(fp.x, fp.y):
            p = Point()
            p.x = float(xx); p.y = float(yy); p.z = 0.0
            m.points.append(p)

        self.opt_marker_pub.publish(m)

    def publish_candidate_paths_markers(self, paths):
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        for i, fp in enumerate(paths):
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = "map"
            marker.ns = "frenet_paths"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.scale.x = 0.05
            marker.color.a = 0.35
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker.lifetime = Duration(sec=0, nanosec=int(0.3 * 1e9))

            for xx, yy in zip(fp.x, fp.y):
                p = Point()
                p.x = float(xx); p.y = float(yy); p.z = 0.0
                marker.points.append(p)

            marker_array.markers.append(marker)

        self.paths_marker_pub.publish(marker_array)

    # ================= MAIN TIMER (ONE TIMER) =================
    def timer_callback(self):
        if (not self.ready) or (self.csp is None):
            return
        if (not self.have_fix) or (self.current_heading_deg is None):
            return

        self._tick += 1

        # 1) heading smooth
        heading_deg = smoothed_yaw_deg(self.current_heading_deg)

        # 2) lat/lon -> xy
        try:
            x, y = lat_lon_to_xy(float(self.current_lat), float(self.current_lon))
        except Exception as e:
            self.get_logger().warn(f"lat_lon_to_xy error: {e}")
            return

        # 3) heading -> yaw
        try:
            yaw = np.deg2rad(convert_yaw(float(heading_deg), yaw_offset=YAW_OFFSET_DEG))
        except Exception as e:
            self.get_logger().warn(f"convert_yaw error: {e}")
            return

        # 4) publish pose+yaw mỗi tick
        self.publish_car_pose_and_yaw(x, y, yaw)

        # 5) publish reference path chậm
        if (self._tick % REF_EVERY) == 0:
            self.publish_reference_path()

        # 6) gom obstacles + persons, rồi transform sang map
        ob_cam = self.obstacles
        if ob_cam.size > 0:
            right = ob_cam[:, 0]
            forward = ob_cam[:, 1]
            ob = np.zeros((ob_cam.shape[0], 2), dtype=float)
            ob[:, 0] = x + forward * math.cos(yaw) + right * math.sin(yaw)
            ob[:, 1] = y + forward * math.sin(yaw) - right * math.cos(yaw)
        else:
            ob = np.empty((0, 2), dtype=float)

        

        optimal_path, paths = None, None
        try:
            optimal_path, paths = frenet_optimal_planning(
                self.csp,
                self.s0,
                2.8,
                self.c_d, self.c_d_d, self.c_d_dd,
                ob)  
        except Exception as e:
            self.get_logger().warn(f"frenet_optimal_planning error: {e}")
            optimal_path, paths = None, None

        retry = 0
        while optimal_path is None and retry < MAX_RETRY:
            retry += 1
            self.get_logger().warn(f"[WARNING] optimal_path is None — retry {retry}/{MAX_RETRY}")

            try:
                # update frenet state (chỉ khi retry, giống code 1)
                self.s0, self.c_d, self.c_d_d, self.c_d_dd = cartesian_to_frenet(x, y, yaw, self.csp)

                optimal_path, paths = frenet_optimal_planning(
                    self.csp,
                    self.s0,
                    2.8,
                    self.c_d, self.c_d_d, self.c_d_dd,
                    ob
                )
            except Exception as e:
                self.get_logger().warn(f"retry planning error: {e}")
                optimal_path, paths = None, None

        if optimal_path is None:
            self.get_logger().warn("⚠️ optimal_path vẫn None -> skip publish")
            return

        if paths is None:
            paths = []
        try:
            self.s0, self.c_d, self.c_d_d, self.c_d_dd = cartesian_to_frenet(x, y, yaw, self.csp)
            self.c_d_d = float(optimal_path.d_d[1])
            self.c_d_dd = float(optimal_path.d_dd[1])
        except Exception:
            pass
        

        # publish optimal path + marker
        self.publish_optimal_path(optimal_path)
        self.publish_optimal_marker(optimal_path)
        self.log_optimal_path_hz()

        # candidate markers chậm hơn
        if (self._tick % VIS_EVERY) == 0:
            self.publish_candidate_paths_markers(paths)

        self.c_speed = float(self.current_speed_mps)


def main(args=None):
    rclpy.init(args=args)
    node = Fot_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
