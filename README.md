
# RTK-GPS-BASED-AUTONOMOUS-DRIVING

A research project on **outdoor autonomous vehicle navigation** using **RTK-GPS**, **AI-based perception** (YOLOv11 + DepthAnything-V2), and **Frenet Optimal Trajectory Planning** with **Pure Pursuit Control**.  

## Overview  

This project implements an **autonomous golf-cart platform** tested at HCMUTE campus. The navigation stack combines centimeter-level localization, deep-learning-based perception, local planning, and low-level control.  

<p align="center">
  <img src="assets/golfcart_hcmute.jpg" width="70%">
</p>  

---

## System Architecture  

<p align="center">
  <img src="assets/system_architecture.png" width="80%">
</p>  

**Pipeline**  
1. **Sensor Data**: RTK-GPS + Monocular Camera.  
2. **Perception**: YOLOv11 (object detection) + DepthAnything-V2 (depth estimation).  
3. **Planner**: Frenet Optimal Trajectory (local path generation).  
4. **Control**: Pure Pursuit (trajectory tracking).  
5. **Visualization**: PyQt5 GUI (map, obstacles, vehicle state).  

---

## Hardware Setup  

<p align="center">
  <img src="assets/hardware_stack.png" width="90%">
</p>  

- **Onboard PC**: Ubuntu, CUDA, PyTorch.  
- **Camera**: IMX335 USB.  
- **RTK-GPS**: Sino GNSS M900.  
- **Controller Board**: STM32 for steering/brake actuation.  
- **Actuators**: Steering servo (gearbox), brake motor.  
- **Platform**: Golf Cart.  

---

## Features  

- **Localization**: RTK-GPS with ±10 cm accuracy.  
- **Perception**: YOLOv11 (objects), DepthAnything-V2 (monocular depth), SegFormer (road segmentation).  
- **Planning**: Frenet Optimal Trajectory, smooth and collision-free.  
- **Control**: Pure Pursuit for stable path following.  
- **GUI**: PyQt5 interface with GPS map + live camera + planner view.  

---

## Demo  

**Full-stack demo (perception → planning → control):**  

<p align="center">
  <img src="assets/demo_citylane_1.gif" width="48%">
  <img src="assets/demo_citylane_2.gif" width="48%">
</p>  

**Perception Snapshots:**  

<p align="center">
  <img src="assets/perception_examples.png" width="95%">
</p>  

---

## Repository Structure  

```
RTK-GPS-BASED-AUTONOMOUS-DRIVING/
├── Visualization/       # PyQt5 GUI
├── Obstacles/           # YOLOv11 + DepthAnything-V2 + Segformer
├── RTK_GPS/             # Localization
├── HD_MAP/              # GPS map
├── Optimal_trajectory/  # Quintic Polynomial
└── run.sh               # Run
```

---

## Results  

- **Localization**: ±10 cm (RTK fix).  
- **Obstacle depth error**: <0.2 m (longitudinal), <0.3 m (lateral).  
- **Trajectory tracking**: ~0.20 m average.  
- **Throughput**: ~20 FPS (RTX 3050 Ti).  

---

## Roadmap  

- Sensor fusion (RTK-GPS + IMU).  
- HD-map + global path planning (A*).  
- LiDAR integration + multi-object tracking.  

---

## Author  

**Phan Văn Hoàng Anh**  
Final-year student @ HCMUTE  
Focus: AI Perception · Sensor Fusion · Autonomous Driving  
