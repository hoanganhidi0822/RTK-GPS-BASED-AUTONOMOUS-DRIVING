import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge

from autonomous_car.perception_node.perception import PerceptionCore


class PerceptionNode(Node):
    """
    Minimal node:
    SUB: /gps/rtk_status (String)
    PUB: /perception/image, /perception/obstacles, /perception/persons
    """

    def __init__(self):
        super().__init__("perception_node")
        
        self._tick = 0

        # ===== PARAMS =====
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_fps", 30.0)
        self.declare_parameter("loop_hz",30.0)
        self.declare_parameter("max_retries", 5)

        self.declare_parameter("rtk_topic", "/gps/rtk_status")
        self.declare_parameter("rtk_fix_keywords", "FIX,RTK")
        self.declare_parameter("seg_mode_default", 1)  # chưa có RTK -> cho seg chạy

        # read
        self.camera_index = int(self.get_parameter("camera_index").value)
        self.camera_fps = float(self.get_parameter("camera_fps").value)
        self.loop_hz = float(self.get_parameter("loop_hz").value)
        self.max_retries = int(self.get_parameter("max_retries").value)

        self.rtk_topic = str(self.get_parameter("rtk_topic").value)
        kw = str(self.get_parameter("rtk_fix_keywords").value)
        self.rtk_keywords = [k.strip().upper() for k in kw.split(",") if k.strip()]
        self.seg_mode = int(self.get_parameter("seg_mode_default").value)

        # state
        self.rtk_ok = False

        # pub/sub
        self.bridge = CvBridge()
        self.pub_img = self.create_publisher(Image, "/perception/image", 10)
        self.pub_obs = self.create_publisher(Float32MultiArray, "/perception/obstacles", 10)
        self.pub_person = self.create_publisher(Float32MultiArray, "/perception/persons", 10)

        self.sub_rtk = self.create_subscription(String, self.rtk_topic, self.cb_rtk, 10)

        # core
        self.core = PerceptionCore()

        # camera
        self.cap = None
        self._open_camera_with_retry()

        # timer
        dt = 1.0 / 30
        self.timer = self.create_timer(dt, self.loop)

        self.get_logger().info(f"✅ PerceptionNode minimal started | cam={self.camera_index}")

    def cb_rtk(self, msg: String):
        s = msg.data.upper().strip()
        if "NO FIX" in s or "NOFIX" in s:
            self.rtk_ok = False
            return
        self.rtk_ok = any(k in s for k in self.rtk_keywords)

    def decide_seg_mode(self) -> int:
        # RTK tốt -> không chạy seg
        return 0 if self.rtk_ok else 1

    def _open_camera_with_retry(self):
        if getattr(self, "cap", None) is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

        for attempt in range(self.max_retries):
            cap = cv2.VideoCapture(self.camera_index)
            cap.set(cv2.CAP_PROP_FPS, self.camera_fps)

            if cap.isOpened():
                self.cap = cap
                self.get_logger().info("✅ Camera opened")
                return

            self.get_logger().warn(f"Retry camera... ({attempt+1}/{self.max_retries})")
            time.sleep(1.0)

        self.get_logger().error("❌ Cannot open camera after retries")

    def loop(self):
        self._tick += 1

        if self.cap is None or (not self.cap.isOpened()):
            self._open_camera_with_retry()
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self._open_camera_with_retry()
            return

        self.seg_mode = self.decide_seg_mode()

        out_img, _depth_u16, obstacles, persons, _seg_steer, _is_int, avg_fps, fps_inst = \
            self.core.process_frame(frame, self.seg_mode)


        if out_img is not None:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(out_img, encoding="bgr8"))

        mo = Float32MultiArray()
        mo.data = [v for (x, z) in obstacles for v in (float(x), float(z))]
        self.pub_obs.publish(mo)

        mp = Float32MultiArray()
        mp.data = [v for (x, z) in persons for v in (float(x), float(z))]
        self.pub_person.publish(mp)

        # ===== log FPS infer (core) =====
        if fps_inst is not None and self._tick % 10 == 0:
            self.get_logger().info(
                f"seg_mode={self.seg_mode} rtk_ok={self.rtk_ok} | fps_inst={fps_inst:.2f} avg={avg_fps:.2f}"
            )


    def destroy_node(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
