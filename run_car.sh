#!/bin/bash

# --- CẤU HÌNH ---
# 1. Nạp môi trường ROS 2 gốc (QUAN TRỌNG: Sửa 'humble' thành phiên bản của bạn nếu khác)
source /opt/ros/humble/setup.bash

WORKSPACE_DIR=~/ros2_ws
PACKAGE_NAME="autonomous_car"
LAUNCH_FILE="bringup.launch.py"

# 2. Di chuyển về thư mục gốc Workspace
echo "📂 Di chuyển đến workspace: $WORKSPACE_DIR"
cd $WORKSPACE_DIR || { echo "❌ Không tìm thấy thư mục $WORKSPACE_DIR"; exit 1; }

# 3. Build package
echo "🔨 Đang build package '$PACKAGE_NAME'..."
colcon build --symlink-install --packages-select $PACKAGE_NAME

# Kiểm tra build
if [ $? -eq 0 ]; then
    echo "✅ Build THÀNH CÔNG!"
else
    echo "❌ Build THẤT BẠI! Dừng script."
    exit 1
fi

# 4. Source setup.bash của workspace (Để nhận gói autonomous_car vừa build)
echo "source install/setup.bash"
source install/setup.bash

# 5. Chạy Launch file
echo "🚀 Đang khởi động: $LAUNCH_FILE"
ros2 launch $PACKAGE_NAME $LAUNCH_FILE