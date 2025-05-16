# RTK-GPS-BASED-AUTONOMOUS-DRIVING

🚗 A private research project on autonomous vehicle navigation using RTK-GPS, obstacle detection, and Frenet optimal trajectory planning.

## 📌 Project Overview

This project integrates high-precision **RTK-GPS** positioning with real-time **obstacle detection** and **path planning algorithms** (Pure Pursuit and Frenet Optimal Trajectory) to navigate an autonomous vehicle in outdoor environments.

### 🔧 Components

- **RTK-GPS**: For centimeter-level positioning accuracy.
- **YOLOv8 / DepthAnything**: For obstacle detection and depth estimation.
- **Frenet Optimal Trajectory**: For smooth and safe path planning around obstacles.
- **Pure Pursuit Controller**: For trajectory tracking.
- **PyQt5 GUI**: Visual interface showing GPS map, obstacle positions, and live vehicle state.

## 🗂️ Project Structure
RTK-GPS-BASED-AUTONOMOUS-DRIVING/
│
├── OBSTACLES/ # Obstacle detection models, outputs
│ ├── checkpoints/ # Trained model weights (.pt, .pth)
│ ├── image*/ # Captured/testing images
│ └── output_video*.mp4 # Output videos (ignored in Git)
│
├── FACE_DETECTION/ # Face detection for human interaction
│ └── model/ # .onnx models
│
├── GPS/ # GPS-related data and tools
├── frenet-optimal-trajectory/ # Frenet planner core code
├── GUI/ # PyQt5 interface files
├── utils/ # Common utilities
└── main.py # Entry point for testing/demo

