


#!/bin/bash
##chmod +x camera_setting.sh

DEVICE="/dev/video2"  # Thay nếu cần

echo "Đang đặt lại cấu hình mặc định cho camera tại $DEVICE..."

# ----------- Reset các thông số điều khiển cơ bản ----------- Nếu quá tối → tăng exposure_time_absolute và gain

v4l2-ctl -d $DEVICE -c auto_exposure=1                      # Auto (Aperture Priority Mode)
v4l2-ctl -d $DEVICE -c exposure_time_absolute=130 \        # Giảm thời gian phơi sáng #156
v4l2-ctl -d $DEVICE -c exposure_dynamic_framerate=1

v4l2-ctl -d $DEVICE -c brightness=128
v4l2-ctl -d $DEVICE -c contrast=30
v4l2-ctl -d $DEVICE -c saturation=64
v4l2-ctl -d $DEVICE -c hue=0

v4l2-ctl -d $DEVICE -c white_balance_automatic=1
v4l2-ctl -d $DEVICE -c white_balance_temperature=4000       # Bị vô hiệu nếu auto WB bật

v4l2-ctl -d $DEVICE -c gamma=100 # # Làm dịu vùng sáng 120
v4l2-ctl -d $DEVICE -c gain=0 # 6 # Giảm độ nhạy
v4l2-ctl -d $DEVICE -c power_line_frequency=2               # Auto
v4l2-ctl -d $DEVICE -c sharpness=2
v4l2-ctl -d $DEVICE -c backlight_compensation=0

echo "✅ Camera đã được khôi phục về mặc định."
