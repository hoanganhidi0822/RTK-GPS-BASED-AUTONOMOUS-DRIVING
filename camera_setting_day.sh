DEVICE="/dev/video2"

# Tắt chế độ phơi sáng tự động
# 🔻 Tắt tự động phơi sáng → chuyển về Manual
sudo v4l2-ctl -d $DEVICE -c auto_exposure=1        # Manual Mode
# 🔻 Giảm thời gian phơi sáng → giảm sáng, tránh cháy
sudo v4l2-ctl -d $DEVICE -c exposure_time_absolute=80

sudo v4l2-ctl -d $DEVICE -c exposure_dynamic_framerate=0

# Tăng contrast để chống chói nhẹ
v4l2-ctl -d $DEVICE -c contrast=25

# 🔻 Giảm gamma (tăng độ tương phản tổng thể, giảm cháy)
sudo v4l2-ctl -d $DEVICE -c gamma=120

# 🔻 Tăng sharpness nếu cần làm rõ nét cạnh (không nên quá cao)
sudo v4l2-ctl -d $DEVICE -c sharpness=20

# Tắt cân bằng trắng tự động (nếu chỉnh tay)
v4l2-ctl -d $DEVICE -c white_balance_automatic=0
v4l2-ctl -d $DEVICE -c white_balance_temperature=6000

# Tùy chỉnh thêm nếu cần
v4l2-ctl -d $DEVICE -c brightness=5
v4l2-ctl -d $DEVICE -c saturation=65
