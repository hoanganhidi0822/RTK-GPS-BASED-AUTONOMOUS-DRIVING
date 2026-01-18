#!/usr/bin/env python3
# gps_node_threaded.py

import time
import threading
import csv
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32, Int32, String

# ================== CỜ CHỌN CHẾ ĐỘ ==================
USE_REAL_GPS = True   # True: đọc GPS thật qua serial | False: fake từ CSV
# =====================================================

if USE_REAL_GPS:
    from .GPS_module import get_gps_data_for_dead_reckoning


# ===================== CONFIG =====================
PUBLISH_HZ = 20           # tần số publish ROS topic (ổn định)
STALE_TIMEOUT = 0.5         # quá lâu không có data mới thì coi như "stale"

# ---- CSV FAKE CONFIG ----
CSV_PATH = "/home/minh_tan/ros2_ws/src/autonomous_car/autonomous_car/GPS_node/test.csv"  # đổi đường dẫn đúng file bạn
PLAYBACK_SPEED = 1.0        # 1.0 = realtime theo timestamp; 2.0 = nhanh gấp đôi; 0.5 = chậm 1/2
LOOP_CSV = True             # hết file thì quay lại từ đầu
USE_CSV_STAMP = False       # True: dùng timestamp trong CSV làm header.stamp (SYSTEM_TIME)
# ================================================


class GPSNode(Node):
    def __init__(self):
        super().__init__("gps_node")

        # -------- Publishers --------
        self.fix_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.heading_pub = self.create_publisher(Float32, "/gps/heading", 10)
        self.speed_pub = self.create_publisher(Float32, "/gps/speed", 10)
        self.rtk_status_pub = self.create_publisher(String, "/gps/rtk_status", 10)
        self.sat_count_pub = self.create_publisher(Int32, "/gps/sat_count", 10)

        # -------- Serial handle --------
        self.ser = None

        # -------- Shared latest data (thread-safe) --------
        self._lock = threading.Lock()
        self._latest = {
            "lat": None,
            "lon": None,
            "heading": None,
            "sat_count": None,
            "rtk_status": "No Fix",
            "speed": None,
            "age": None,
            "stamp": None,
            "rx_wall_time": 0.0,
        }
        self._stop_evt = threading.Event()

        # ---- CSV buffer (for fake) ----
        self._csv_rows = []
        self._csv_idx = 0

        if USE_REAL_GPS:
            self.reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
            self.reader_thread.start()
            mode = "REAL(serial thread)"
        else:
            self._load_csv(CSV_PATH)
            self.reader_thread = threading.Thread(target=self._fake_reader_loop_csv, daemon=True)
            self.reader_thread.start()
            mode = f"FAKE(CSV) path={CSV_PATH} speed={PLAYBACK_SPEED}x"

        # -------- Publish timer (NON-BLOCKING) --------
        self.dt = 1.0 / float(PUBLISH_HZ)
        self.timer = self.create_timer(self.dt, self._publish_timer_cb)

        self.get_logger().info(f"✅ GPSNode started | mode={mode} | publish={PUBLISH_HZ} Hz")

    # ======================= CSV load =======================
    def _load_csv(self, path: str):
        if not os.path.exists(path):
            self.get_logger().error(f"[CSV] File not found: {path}")
            self._csv_rows = []
            return

        rows = []
        with open(path, "r", newline="") as f:
            reader = csv.DictReader(f)
            required = {"timestamp", "latitude", "longitude", "heading", "sat_count", "rtk_status", "speed", "age"}
            if not required.issubset(set(reader.fieldnames or [])):
                self.get_logger().error(f"[CSV] Header thiếu cột. Cần: {sorted(required)} | Có: {reader.fieldnames}")
                self._csv_rows = []
                return

            for i, r in enumerate(reader):
                try:
                    rows.append({
                        "timestamp": float(r["timestamp"]),
                        "lat": float(r["latitude"]),
                        "lon": float(r["longitude"]),
                        "heading": float(r["heading"]),
                        "sat_count": int(float(r["sat_count"])),
                        "rtk_status": str(r["rtk_status"]),
                        "speed": float(r["speed"]),
                        "age": float(r["age"]),
                    })
                except Exception as e:
                    self.get_logger().warn(f"[CSV] Skip line {i+2} parse error: {e}")

        rows.sort(key=lambda x: x["timestamp"])
        self._csv_rows = rows
        self._csv_idx = 0
        self.get_logger().info(f"[CSV] Loaded {len(rows)} rows")

    # ======================= Reader loops =======================
    def _serial_reader_loop(self):
        while rclpy.ok() and (not self._stop_evt.is_set()):
            try:
                lat, lon, heading, sat_count, rtk_status, speed, age, self.ser = \
                    get_gps_data_for_dead_reckoning(self.ser)

                stamp = self.get_clock().now().to_msg()
                rx_wall = time.time()

                with self._lock:
                    self._latest.update({
                        "lat": float(lat) if lat is not None else None,
                        "lon": float(lon) if lon is not None else None,
                        "heading": float(heading) if heading is not None else None,
                        "sat_count": int(sat_count) if sat_count is not None else None,
                        "rtk_status": str(rtk_status) if rtk_status is not None else "No Fix",
                        "speed": float(speed) if speed is not None else None,
                        "age": float(age) if age is not None else None,
                        "stamp": stamp,
                        "rx_wall_time": rx_wall,
                    })

            except Exception as e:
                self.get_logger().warn(f"[GPS reader] error: {e}")
                time.sleep(0.05)

    def _fake_reader_loop_csv(self):
        """
        Thread fake từ CSV:
        - Mỗi dòng CSV là 1 sample GPS
        - Sleep theo delta timestamp / PLAYBACK_SPEED để phát lại "giống thật"
        """
        if not self._csv_rows:
            self.get_logger().error("[CSV] No data rows. Fake thread stopped.")
            return

        def make_stamp_from_unix(ts: float):
            sec = int(ts)
            nsec = int((ts - sec) * 1e9)
            # builtin_interfaces/Time msg
            from builtin_interfaces.msg import Time as RosTimeMsg
            m = RosTimeMsg()
            m.sec = sec
            m.nanosec = nsec
            return m

        while rclpy.ok() and (not self._stop_evt.is_set()):
            row = self._csv_rows[self._csv_idx]

            # --- stamp ---
            if USE_CSV_STAMP:
                stamp = make_stamp_from_unix(row["timestamp"])
            else:
                stamp = self.get_clock().now().to_msg()

            rx_wall = time.time()
            with self._lock:
                self._latest.update({
                    "lat": row["lat"],
                    "lon": row["lon"],
                    "heading": row["heading"],
                    "sat_count": row["sat_count"],
                    "rtk_status": row["rtk_status"],
                    "speed": row["speed"],
                    "age": row["age"],
                    "stamp": stamp,
                    "rx_wall_time": rx_wall,
                })

            # --- compute sleep based on next timestamp ---
            next_idx = self._csv_idx + 1
            if next_idx >= len(self._csv_rows):
                if LOOP_CSV:
                    next_idx = 0
                else:
                    self.get_logger().info("[CSV] End of file. Fake thread stopped.")
                    return

            t_now = row["timestamp"]
            t_next = self._csv_rows[next_idx]["timestamp"]
            dt = max(0.0, t_next - t_now)

            # playback speed
            if PLAYBACK_SPEED <= 0:
                sleep_s = 0.01
            else:
                sleep_s = dt / float(PLAYBACK_SPEED)

            # clamp để tránh sleep quá bé gây busy-loop
            sleep_s = max(0.001, min(sleep_s, 0.5))

            self._csv_idx = next_idx
            time.sleep(sleep_s)

    # ======================= Publisher timer =======================
    def _publish_timer_cb(self):
        """
        Timer publish: tuyệt đối KHÔNG đọc serial ở đây.
        Chỉ lấy data mới nhất rồi publish -> tần số publish ổn định.
        """
        with self._lock:
            data = dict(self._latest)

        if data["lat"] is None or data["lon"] is None or data["stamp"] is None:
            return

        age_sec = time.time() - float(data["rx_wall_time"])
        is_stale = age_sec > STALE_TIMEOUT

        fix_msg = NavSatFix()
        fix_msg.header.stamp = data["stamp"]
        fix_msg.header.frame_id = "gps"
        fix_msg.latitude = float(data["lat"])
        fix_msg.longitude = float(data["lon"])
        fix_msg.altitude = 0.0

        status = NavSatStatus()
        if (not is_stale) and ("Fixed" in str(data["rtk_status"])):
            status.status = NavSatStatus.STATUS_FIX
        else:
            status.status = NavSatStatus.STATUS_NO_FIX
        status.service = NavSatStatus.SERVICE_GPS
        fix_msg.status = status

        self.fix_pub.publish(fix_msg)

        if data["heading"] is not None:
            hdg_msg = Float32()
            hdg_msg.data = float(data["heading"])
            self.heading_pub.publish(hdg_msg)

        if data["speed"] is not None:
            spd_msg = Float32()
            spd_msg.data = float(data["speed"])
            self.speed_pub.publish(spd_msg)

        st_msg = String()
        st_msg.data = str(data["rtk_status"]) if not is_stale else "STALE"
        self.rtk_status_pub.publish(st_msg)

        if data["sat_count"] is not None:
            sat_msg = Int32()
            sat_msg.data = int(data["sat_count"])
            self.sat_count_pub.publish(sat_msg)

    # ======================= Shutdown =======================
    def destroy_node(self):
        try:
            self._stop_evt.set()
            if hasattr(self, "reader_thread") and self.reader_thread.is_alive():
                self.reader_thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            if USE_REAL_GPS and self.ser is not None and getattr(self.ser, "is_open", False):
                self.ser.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
