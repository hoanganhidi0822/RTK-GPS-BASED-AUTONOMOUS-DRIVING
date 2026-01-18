#!/bin/bash

# --- CONFIG ---
# 1. Source ROS 2 environment (IMPORTANT: change 'humble' to your ROS distro if different)
source /opt/ros/humble/setup.bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_NAME="autonomous_car"
LAUNCH_FILE="bringup.launch.py"

# 2. Move to workspace root
echo "Di chuyển đến workspace: $WORKSPACE_DIR"
cd "$WORKSPACE_DIR" || { echo "Không tìm thấy thư mục $WORKSPACE_DIR"; exit 1; }

# 3. Source workspace setup.bash (to register the built package)
echo "source install/setup.bash"
source install/setup.bash

# 4. Run launch file
echo "Đang khởi động: $LAUNCH_FILE"
ros2 launch "$PACKAGE_NAME" "$LAUNCH_FILE"
