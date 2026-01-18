#!/bin/bash

# --- CONFIG ---
# 1. Source ROS 2 environment (IMPORTANT: change 'humble' to your ROS distro if different)
source /opt/ros/humble/setup.bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_NAME="autonomous_car"

# 2. Move to workspace root
echo "Di chuyển đến workspace: $WORKSPACE_DIR"
cd "$WORKSPACE_DIR" || { echo "Không tìm thấy thư mục $WORKSPACE_DIR"; exit 1; }

# 3. Build package
echo "Đang build package '$PACKAGE_NAME'..."
colcon build --symlink-install --packages-select "$PACKAGE_NAME"

# Check build result
if [ $? -eq 0 ]; then
    echo "Build THÀNH CÔNG!"
else
    echo "Build THẤT BẠI! Dừng script."
    exit 1
fi
