#!/bin/bash

DEVICE="/dev/video0"  # Đường dẫn camera USB, thay nếu cần

echo "Đang cấu hình camera IMX335 tại $DEVICE..."

# ----------- Phơi sáng (exposure) --------------

# Tắt chế độ tự động phơi sáng (auto exposure)
# Giá trị 1 = Manual Mode, cho phép bạn tự điều chỉnh exposure_time_absolute
v4l2-ctl -d $DEVICE -c auto_exposure=1

# Thiết lập thời gian phơi sáng (micro giây * 100)
# Giá trị thấp hơn => ít sáng hơn => chống chói khi trời nắng
# Gợi ý: 60–120 (ngoài trời nắng); 150–250 (trong nhà)
v4l2-ctl -d $DEVICE -c exposure_time_absolute=80

# Tắt dynamic framerate để giữ FPS cố định khi giảm sáng
v4l2-ctl -d $DEVICE -c exposure_dynamic_framerate=0

# ----------- Cân bằng sáng & độ tương phản ------------

# Độ sáng tổng thể của ảnh. Giảm nếu bị chói (gợi ý: 90–130)
v4l2-ctl -d $DEVICE -c brightness=127

# Tăng độ tương phản giúp làm rõ vùng tối – rất cần thiết khi ngoài trời
# Gợi ý: 130–180, tùy độ sáng môi trường
v4l2-ctl -d $DEVICE -c contrast=160

# Gamma giúp điều chỉnh độ sáng trung gian (mid-tone)
# Giảm gamma để ảnh ít bị chói vùng trung tâm
# Gợi ý: 90–110 (ngoài trời); 120 (mặc định trong nhà)
v4l2-ctl -d $DEVICE -c gamma=100

# Bù sáng nền – bật lên nếu vùng tối bị mất chi tiết
# Gợi ý: 1 (bật nhẹ); 2 (bật mạnh); 0 (tắt)
v4l2-ctl -d $DEVICE -c backlight_compensation=1

# ----------- Cân bằng trắng (White Balance) ------------

# Tắt tự động WB để tránh ảnh bị ngả xanh/vàng khi nắng chiếu mạnh
v4l2-ctl -d $DEVICE -c white_balance_automatic=0

# Chỉnh WB thủ công: 5000–5500 phù hợp ánh sáng mặt trời
# Gợi ý: 5000 (trời nắng); 4000 (trong nhà đèn vàng); 6000 (trời râm)
v4l2-ctl -d $DEVICE -c white_balance_temperature=5000

# ----------- Khử nhiễu ------------

# Gain khuếch đại ánh sáng yếu. Cao thì sáng hơn nhưng nhiễu nhiều
# Gợi ý: để giá trị thấp nhất để ảnh sạch (thường là 4)
v4l2-ctl -d $DEVICE -c gain=4

echo "✔️ Cấu hình hoàn tất!"
