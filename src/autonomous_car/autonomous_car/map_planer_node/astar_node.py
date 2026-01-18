#!/usr/bin/env python3
import os
import math
import heapq
from collections import defaultdict
import csv
import rclpy
import matplotlib.pyplot as plt
import contextily as ctx
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from pyproj import Transformer
from .astar_config import *

EARTH_RADIUS_M = 6371000.0
DEFAULT_ORIGIN_LAT = float(os.environ.get("ORIGIN_LAT", "10.8532570333"))
DEFAULT_ORIGIN_LON = float(os.environ.get("ORIGIN_LON", "106.7715131967"))


def latlon_from_xy(x: float, y: float, lat0: float, lon0: float):
    lat = lat0 - math.degrees(y / EARTH_RADIUS_M)
    lon = lon0 - math.degrees(x / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon


class RoutePlannerNode(Node):
    def __init__(self):
        super().__init__('route_planner_node')

        # Biến destination (lat, lon) – sẽ set lúc lập plan
        self.destination = None

        self.declare_parameter("origin_lat", DEFAULT_ORIGIN_LAT)
        self.declare_parameter("origin_lon", DEFAULT_ORIGIN_LON)
        self.origin_lat = self.get_parameter("origin_lat").get_parameter_value().double_value
        self.origin_lon = self.get_parameter("origin_lon").get_parameter_value().double_value

        # Current position from /odometry
        self.have_fix = False
        self.lat_cur = None
        self.lon_cur = None

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry',
            self.odom_callback,
            10
        )

        # Subscribed request lập route từ visualization
        # data = "MS", "C", "VD", ...
        self.plan_sub = self.create_subscription(
            String,
            '/route_plan/request',
            self.plan_request_callback,
            10
        )
        self.route_done_pub = self.create_publisher(
            String,
            '/route_plan/done',
            10
        )

        self.get_logger().info(
            "✅ RoutePlannerNode started, waiting for /odometry và /route_plan/request ..."
        )

    # --- Nhận odometry liên tục ---
    def odom_callback(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self.lat_cur, self.lon_cur = latlon_from_xy(x, y, self.origin_lat, self.origin_lon)
        self.have_fix = True

    # --- Nhận tín hiệu bấm nút từ visualization ---
    def plan_request_callback(self, msg: String):
        dest_cp = msg.data.strip()
        self.get_logger().info(f"📩 Nhận request lập route tới checkpoint: {dest_cp}")

        if not self.have_fix:
            self.get_logger().warn("❌ Chưa có GPS fix nào, chưa thể lập route.")
            return

        if dest_cp not in CHECKPOINTS:
            self.get_logger().error(
                f"❌ destination_checkpoint='{dest_cp}' không tồn tại trong CHECKPOINTS"
            )
            return

        # Dùng GPS hiện tại tại thời điểm bấm nút
        lat_start = self.lat_cur
        lon_start = self.lon_cur

        # Lấy đích
        lat_dest, lon_dest = CHECKPOINTS[dest_cp]
        self.destination = (lat_dest, lon_dest)

        self.get_logger().info(
            f"Start: lat={lat_start:.7f}, lon={lon_start:.7f}"
        )
        self.get_logger().info(
            f"Dest ({dest_cp}): lat={lat_dest:.7f}, lon={lon_dest:.7f}"
        )

        # ===== Chọn group THUẬN / NGHỊCH =====
        cp_start = nearest_checkpoint_name(lat_start, lon_start)
        cp_dest = dest_cp

        self.get_logger().info(f"Checkpoint gần start: {cp_start}")
        self.get_logger().info(f"Checkpoint dest:      {cp_dest}")

        nghich_dir = os.path.join(ROOT_DIR, "NGHICH")
        thuan_dir  = os.path.join(ROOT_DIR, "THUAN")

        chosen_group = choose_group_by_order(cp_start, cp_dest)
        self.get_logger().info(f"==> Chọn ROUTE_GROUP = {chosen_group}")

        if chosen_group == "THUAN":
            csv_path = os.path.join(ROOT_DIR, "THUAN", "THUAN_densified.csv")
        else:
            csv_path = os.path.join(ROOT_DIR, "NGHICH", "NGHICH_densified.csv")

        nodes, edges = build_graph_from_file(csv_path)


        # ===== Xây graph từ folder đã chọn =====
        nodes, edges = build_graph_from_file(csv_path, join_thresh_m=5.0)
        self.get_logger().info(f"Graph có {len(nodes)} node.")

        # ===== Tìm node gần nhất start & dest =====
        start_idx = find_nearest_node(nodes, lat_start, lon_start)
        goal_idx  = find_nearest_node(nodes, lat_dest,  lon_dest)
        self.get_logger().info(f"Nearest node start: {start_idx}, goal: {goal_idx}")

        # ===== A* =====
        route_indices = astar(nodes, edges, start_idx, goal_idx)
        if not route_indices:
            self.get_logger().error("❌ Không tìm được route A*")
            return

        # ===== Lưu file =====
        out_file = os.path.join(path_dir, "merged_waypoints.txt")
        save_waypoints_segment_from_nodes(nodes, route_indices, out_file)
        done_msg = String()
        done_msg.data = dest_cp   # hoặc "OK", tuỳ bạn muốn báo gì
        self.route_done_pub.publish(done_msg)
        self.get_logger().info(f"✅ Route A* xong, đã lưu {len(route_indices)} điểm và báo /route_plan/done = {dest_cp}")

        # Log số lượng waypoint trong route

        # ===== Vẽ BẢN ĐỒ DEBUG =====
        try:
            plot_route_debug(
                nodes,
                route_indices,
                lat_start,
                lon_start,
                self.destination,
                dest_cp,
                chosen_group
            )
        except Exception as e:
            self.get_logger().warn(f"[WARN] Không vẽ được debug map: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RoutePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
