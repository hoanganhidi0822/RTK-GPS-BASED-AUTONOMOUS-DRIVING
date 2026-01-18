import time
from collections import deque
from contextlib import nullcontext

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as R

from autonomous_car.perception_node.depth_anything_v2.dpt import DepthAnythingV2
from autonomous_car.perception_node.Segformer.road_segmentation import get_steering_angle


def optimize_for_gpu(model):
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = model.to(dev)

    if dev.type == "cuda":
        torch.backends.cudnn.benchmark = True
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.set_float32_matmul_precision("high")

        if hasattr(torch.backends.cuda, "enable_flash_sdp"):
            torch.backends.cuda.enable_flash_sdp(True)
            torch.backends.cuda.enable_mem_efficient_sdp(True)

        if hasattr(model, "gradient_checkpointing_enable"):
            model.gradient_checkpointing_enable()

        if torch.__version__ >= "2.0":
            try:
                model = torch.compile(model, mode="max-autotune")
            except Exception:
                pass

    return model


class PerceptionCore:
    """
    Core xử lý 1 frame:
    input: raw_frame (BGR)
    output: out_img (BGR), depth_u16 (u16) + lists + seg info + avg_fps (infer fps)
    """

    def __init__(
        self,
        encoder="vits",
        input_size=518,
        yolo_weights="yolo11n.pt",
        depth_ckpt=None,
        camera_matrix_path="/home/minh_tan/ros2_ws/src/autonomous_car/autonomous_car/perception_node/camera_param/camera_matrix.txt",
        dist_coeffs_path="/home/minh_tan/ros2_ws/src/autonomous_car/autonomous_car/perception_node/camera_param/distortion_coefficients.txt",
        use_undistort=True,
        width=640,
        height=480,
        depth_scale_factor=1.25,
        max_depth=30.0,
        max_x=8.0,
        fx=267.0, fy=267.0, cx=320.0, cy=245.0,
        skip_seg0=3,
        skip_seg1=4,
    ):
        self.W = int(width)
        self.H = int(height)
        self.use_undistort = bool(use_undistort)

        self.depth_scale_factor = float(depth_scale_factor)
        self.max_depth = float(max_depth)
        self.max_x = float(max_x)

        self.skip_seg0 = int(skip_seg0)
        self.skip_seg1 = int(skip_seg1)

        # device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # YOLO
        self.detector = YOLO(yolo_weights).to(self.device)

        # DepthAnything
        model_configs = {
            "vits": {"encoder": "vits", "features": 64,  "out_channels": [48, 96, 192, 384]},
            "vitb": {"encoder": "vitb", "features": 128, "out_channels": [96, 192, 384, 768]},
            "vitl": {"encoder": "vitl", "features": 256, "out_channels": [256, 512, 1024, 1024]},
            "vitg": {"encoder": "vitg", "features": 384, "out_channels": [1536, 1536, 1536, 1536]},
        }
        if depth_ckpt is None:
            depth_ckpt = f"/home/minh_tan/ros2_ws/src/autonomous_car/autonomous_car/perception_node/checkpoints/depth_anything_v2_{encoder}.pth"

        self.depth_anything = DepthAnythingV2(**model_configs[encoder])
        self.depth_anything.load_state_dict(
            torch.load(depth_ckpt, map_location="cuda" if self.device.type == "cuda" else "cpu")
        )
        self.depth_anything = optimize_for_gpu(self.depth_anything).eval()

        self.input_size = int(input_size)

        # undistort map
        self.map1, self.map2 = None, None
        if self.use_undistort:
            camera_matrix = np.loadtxt(camera_matrix_path, dtype=np.float32)
            dist_coeffs = np.loadtxt(dist_coeffs_path, dtype=np.float32)
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                camera_matrix, dist_coeffs, None, camera_matrix, (self.W, self.H), cv2.CV_16SC2
            )

        # inv_K
        self.inv_K = np.linalg.inv(np.array([
            [fx,  0.0, cx],
            [0.0, fy,  cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32))

        # cache
        self.count_frame = 2
        self.results_cache = []
        self.depth_cache_u16 = None

        # fps window: chỉ tính khi infer thật
        self.fps_window = deque(maxlen=30)

        self.last_seg_steer = 0.0
        self.last_is_intersection = 0

    def _undistort(self, frame):
        if self.use_undistort and self.map1 is not None and self.map2 is not None:
            return cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        return frame

    def process_frame(self, raw_frame_bgr, seg_mode: int):
        """
        Returns:
          out_img_bgr, depth_u16_or_None, obstacles[(x,z)], persons[(x,z)],
          seg_steer, is_intersection, avg_fps_or_None  (avg_fps = infer FPS)
        """
        if raw_frame_bgr is None:
            return None, None, [], [], self.last_seg_steer, self.last_is_intersection, None

        frame = raw_frame_bgr
        if frame.shape[1] != self.W or frame.shape[0] != self.H:
            frame = cv2.resize(frame, (self.W, self.H), interpolation=cv2.INTER_LINEAR)

        self.count_frame += 1
        frame = self._undistort(frame)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        overlay = None
        if seg_mode == 1:
            try:
                angle, overlay, is_intersection = get_steering_angle(rgb, debug=1)
                self.last_seg_steer = float(angle)
                self.last_is_intersection = int(is_intersection)
            except Exception:
                overlay = None

        # frame skipping
        if seg_mode == 1:
            do_infer = (self.count_frame % max(self.skip_seg1, 1) == 0)
        else:
            do_infer = (self.count_frame % max(self.skip_seg0, 1) == 0)

        infer_t0 = None

        # ===== infer (chỉ khi do_infer) =====
        if do_infer:
            try:
                # sync để timing không ảo (cuda async)
                if self.device.type == "cuda":
                    torch.cuda.synchronize()
                infer_t0 = time.perf_counter()

                amp_ctx = torch.amp.autocast("cuda") if self.device.type == "cuda" else nullcontext()
                with torch.no_grad():
                    with amp_ctx:
                        self.results_cache = self.detector(
                            frame, verbose=False, device=self.device, classes=[0, 1, 2, 3, 4, 7]
                        )
                        depth_map = self.depth_anything.infer_image(frame, self.input_size)

                depth_map = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min() + 1e-9) * 65535.0
                self.depth_cache_u16 = depth_map.astype(np.uint16)

                if self.device.type == "cuda":
                    torch.cuda.synchronize()

            except Exception:
                self.results_cache = []
                self.depth_cache_u16 = None

        # ===== postprocess =====
        obstacles, persons = [], []
        frame_draw = frame.copy()

        if self.depth_cache_u16 is not None and self.results_cache:
            depth_u16 = self.depth_cache_u16

            for predictions in self.results_cache:
                for bbox in predictions.boxes:
                    class_id = int(bbox.cls.cpu().numpy()[0])
                    if class_id not in [0, 1, 2, 3, 4, 7]:
                        continue

                    xmin, ymin, xmax, ymax = bbox.xyxy[0].cpu().numpy()
                    bw, bh = xmax - xmin, ymax - ymin
                    if bw < 10 or bh < 10 or bw > 400 or bh > 400:
                        continue

                    x1 = int(max(xmin, 0)); y1 = int(max(ymin, 0))
                    x2 = int(min(xmax, self.W - 1)); y2 = int(min(ymax, self.H - 1))
                    if x2 <= x1 or y2 <= y1:
                        continue

                    patch = depth_u16[y1:y2, x1:x2]
                    if patch.size == 0:
                        continue

                    depth_value = float(np.median(patch))
                    if depth_value <= 1e-3:
                        continue

                    z_camera = (65535.0 / depth_value) * self.depth_scale_factor

                    cxp = (xmin + xmax) / 2.0
                    cyp = (ymin + ymax) / 2.0
                    pixel = np.array([cxp, cyp, 1.0], dtype=np.float32)

                    cam_xyz = self.inv_K @ (pixel * z_camera)
                    rot = R.from_euler("x", 0, degrees=True).as_matrix()
                    real = rot @ cam_xyz
                    x_real, z_real = float(real[0]), float(real[2])

                    if z_real < self.max_depth and abs(x_real) <= self.max_x:
                        if class_id == 0:
                            persons.append((x_real, z_real))
                        elif class_id in [2, 7]:
                            obstacles.append((x_real, z_real))

                        cv2.rectangle(frame_draw, (x1, y1), (x2, y2), (0, 0, 255), 1)
                        cv2.putText(frame_draw, f"X:{x_real:.2f} Z:{z_real:.2f}",
                                    (x1, min(y2 + 15, self.H - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        out_img = overlay if (seg_mode == 1 and overlay is not None) else frame_draw

        # ===== FPS giống code cũ: chỉ tính khi infer thật =====
        # ===== FPS: trả cả fps_inst để log ngay =====
        avg_fps = None
        fps_inst = None

        if do_infer and (infer_t0 is not None):
            dt = time.perf_counter() - infer_t0
            fps_inst = 1.0 / dt if dt > 1e-6 else 0.0
            self.fps_window.append(fps_inst)
            # cho ra avg_fps sớm hơn (không cần đủ 30)
            avg_fps = float(sum(self.fps_window) / len(self.fps_window))

        return (
            out_img,
            self.depth_cache_u16,
            obstacles,
            persons,
            self.last_seg_steer,
            self.last_is_intersection,
            avg_fps,
            fps_inst
        )
