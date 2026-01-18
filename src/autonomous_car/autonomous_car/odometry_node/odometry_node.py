#!/usr/bin/env python3

import math
import os
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

EARTH_RADIUS_M = 6371000.0
DEFAULT_ORIGIN_LAT = float(os.environ.get("ORIGIN_LAT", "10.8532570333"))
DEFAULT_ORIGIN_LON = float(os.environ.get("ORIGIN_LON", "106.7715131967"))


def xy_from_latlon(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    # Match the projection used in Fot_node.Frenet.lat_lon_to_xy.
    x = EARTH_RADIUS_M * math.radians(lon0 - lon) * math.cos(math.radians(lat0))
    y = EARTH_RADIUS_M * math.radians(lat0 - lat)
    return x, y


def quat_from_yaw(yaw_rad: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw_rad / 2.0), w=math.cos(yaw_rad / 2.0))


@dataclass
class GpsState:
    lat: Optional[float] = None
    lon: Optional[float] = None
    heading_deg: Optional[float] = None
    speed_mps: Optional[float] = None


class OdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("odometry_node")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("origin_lat", DEFAULT_ORIGIN_LAT)
        self.declare_parameter("origin_lon", DEFAULT_ORIGIN_LON)

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.child_frame_id = self.get_parameter("child_frame_id").get_parameter_value().string_value
        self.origin_lat = self.get_parameter("origin_lat").get_parameter_value().double_value
        self.origin_lon = self.get_parameter("origin_lon").get_parameter_value().double_value

        self.state = GpsState()
        self.odom_pub = self.create_publisher(Odometry, "/odometry", 10)

        self.create_subscription(NavSatFix, "/gps/fix", self._fix_cb, 10)
        self.create_subscription(Float32, "/gps/heading", self._heading_cb, 10)
        self.create_subscription(Float32, "/gps/speed", self._speed_cb, 10)

    def _fix_cb(self, msg: NavSatFix) -> None:
        self.state.lat = msg.latitude
        self.state.lon = msg.longitude

        self._publish_odom(msg.header.stamp)

    def _heading_cb(self, msg: Float32) -> None:
        self.state.heading_deg = float(msg.data)

    def _speed_cb(self, msg: Float32) -> None:
        # GPS speed is typically km/h; convert to m/s for Odometry.
        self.state.speed_mps = float(msg.data) / 3.6

    def _publish_odom(self, stamp) -> None:
        if self.state.lat is None or self.state.lon is None:
            return

        x, y = xy_from_latlon(self.state.lat, self.state.lon, self.origin_lat, self.origin_lon)

        # Heading is degrees from North. Convert to map yaw (matching convert_yaw(..., 90)).
        if self.state.heading_deg is not None:
            yaw_deg = (270.0 - self.state.heading_deg) % 360.0
            yaw = math.radians(yaw_deg)
        else:
            yaw = 0.0

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat_from_yaw(yaw)

        if self.state.speed_mps is not None:
            odom.twist.twist.linear.x = float(self.state.speed_mps)

        self.odom_pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
