
# RTK‑GPS‑BASED‑AUTONOMOUS‑DRIVING

> Outdoor autonomous golf‑cart navigation using **RTK‑GPS**, **AI perception** (YOLOv11 + DepthAnything‑V2), and **Frenet Optimal Trajectory** with **Pure Pursuit** control.  
> Author: **Phan Văn Hoàng Anh** (HCMUTE, Intelligent Systems Lab)

![Platform](assets/golfcart_hcmute.jpg)

---

## 1) Project Overview

This repository contains a real‑time navigation stack for an outdoor autonomous vehicle (golf‑cart platform). The stack integrates high‑precision localization, monocular perception, local planning, and low‑level control with a PyQt5 GUI for monitoring and tele‑op.

**Highlights**
- Centimeter‑level localization with **RTK‑GPS (Sino M900)**.
- Obstacle detection (**YOLOv11**) and monocular **depth estimation (DepthAnything‑V2)**.
- **Frenet Optimal Trajectory (FOT)** local planner and **Pure Pursuit** tracking.
- PyQt5‑based GUI: GPS map, obstacles, vehicle state, and live camera.
- Measured performance (outdoor campus):
  - Localization accuracy: ± **10 cm** (RTK fix)
  - Obstacle 3D position error: < **0.2 m** (longitudinal), < **0.3 m** (lateral) @ 3–8 m
  - Trajectory tracking error: **~0.20 m**
  - End‑to‑end throughput: **~20 FPS** (RTX 3050 Ti)

---

## 2) Hardware & Wiring Overview

![Hardware Stack](assets/hardware_stack.png)

**Modules**
- **a. On‑board PC** (Ubuntu, CUDA, PyTorch)
- **b. Monocular Camera** (USB IMX335)
- **c. RTK‑GPS** (Sino GNSS M900, survey antenna)
- **d. STM32‑based control board** (steering / brake I/O, PWM, ADC)
- **e1. Steering Actuator** (servo/gearbox)
- **e2. Encoder / feedback sensor**
- **f1–f2. Power & motor drivers**
- **g. Golf‑cart platform** (ISLAB‑C102)

---

## 3) System Architecture

![System Architecture](assets/system_architecture.png)

**Data flow (simplified)**
1. **Sensors** → camera & RTK‑GPS.
2. **Perception** → YOLOv11 (objects), DepthAnything‑V2 (dense depth). 3D obstacle coordinates are computed in the camera frame and transformed to the global frame using extrinsics + GPS pose.
3. **Local Planner** → Frenet Optimal Trajectory samples and scores candidate paths (smoothness, collision, curvature, goal progress).
4. **Controller** → Pure Pursuit outputs steering; throttle/brake commands are regulated by simple speed PID.
5. **GUI** → PyQt5 map + overlays (objects, path, states).

---

## 4) Demos

**Perception → Depth → Planning → Control**

![Demo 1](assets/demo_citylane_1.gif)  
![Demo 2](assets/demo_citylane_2.gif)

**Perception Snapshots** (detection, depth projection, road segmentation & centerline)

![Perception Examples](assets/perception_examples.png)

---

## 5) Installation

### 5.1 Prerequisites
- Ubuntu 22.04/24.04, Python **3.9+**
- NVIDIA GPU with CUDA (tested: **RTX 3050 Ti**)
- PyTorch 2.x (CUDA build), OpenCV‑Python, NumPy, PyQt5, Matplotlib
- Model weights: **YOLOv11** (e.g., `yolo11n.pt`) and **DepthAnything‑V2**

### 5.2 Setup
```bash
git clone https://github.com/<your-username>/RTK-GPS-BASED-AUTONOMOUS-DRIVING.git
cd RTK-GPS-BASED-AUTONOMOUS-DRIVING
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Put model weights here:
#   - weights/yolo11n.pt
#   - weights/depthanything_v2_vit_small.pth
```

### 5.3 Quick Start
```bash
# Launch full GUI (map + camera + planner)
python main.py

# Test perception only with a USB camera:
python perception/run_detection.py --camera 0

# Run local planner unit test (Frenet + Pure Pursuit):
python planning/test_fot_pure_pursuit.py --viz
```

---

## 6) Repository Layout (suggested)

```
RTK-GPS-BASED-AUTONOMOUS-DRIVING/
├── gui/                     # PyQt5 UI and widgets
├── perception/              # YOLOv11, DepthAnything-V2 wrappers
├── planning/                # Frenet Optimal Trajectory, path utils
├── control/                 # Pure Pursuit, speed PID
├── localization/            # GPS parsing, (future) IMU/VO fusion
├── config/                  # calibration, map, planner params
├── weights/                 # *.pt, *.pth (gitignored)
├── scripts/                 # tools: data log, playback, calibration
└── main.py
```

---

## 7) Key Parameters

- `config/camera.yaml` → intrinsics/extrinsics (camera ↔ vehicle ↔ GPS antenna)
- `config/fot.yaml` → sampling ranges, costs (collision, jerk, curvature)
- `config/controller.yaml` → lookahead distance, wheelbase, speed PID
- `config/map.yaml` → waypoints, HD‑map reference, RTK base settings

---

## 8) Notes & Tips

- Calibrate camera intrinsics, then extrinsics (PnP) against a known board; verify reprojection error < 0.5 px.
- When GPS RTK drops to float/SPS, switch to road‑segmentation‑only fallback with speed limit.
- Use moving‑average filters to stabilize depth before projecting to world.
- Vectorize portions of **Frenet sampling** and **cost evaluation** for speed.

---

## 9) Roadmap

- Sensor fusion (RTK‑GPS + IMU + VO) via EKF/IEKF or `robot_localization` (ROS2).
- HD‑map and global routing (A* / Lanelet2).
- Add LiDAR option and 3D multi‑object tracking.
- Port to **ROS2** (Rolling/Jazzy) with modular nodes.

---

## 10) References

- Werling et al., “Optimal Trajectory Generation for Dynamic Street Scenarios…” (Frenet)
- Coulter, “Implementation of the Pure Pursuit Path Tracking Algorithm.”
- DepthAnything‑V2 (Huang et al.) and YOLOv11.
