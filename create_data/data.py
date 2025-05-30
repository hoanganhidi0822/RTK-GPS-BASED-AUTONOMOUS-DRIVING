import cv2
import os

# Cấu hình
camera_index = 2  # Thường là 0, có thể thay đổi nếu bạn có nhiều camera
output_folder = "captured_images"
save_every_n_frames = 10

# Tạo thư mục lưu ảnh nếu chưa tồn tại
os.makedirs(output_folder, exist_ok=True)

# Mở camera
cap = cv2.VideoCapture(camera_index)

if not cap.isOpened():
    print("Không thể mở camera.")
    exit()

frame_count = 0
image_count = 0

print("Đang đọc từ camera. Nhấn 'q' để thoát.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Không đọc được frame từ camera.")
        break

    frame_count += 1

    # Lưu ảnh mỗi N frame
    if frame_count % save_every_n_frames == 0:
        image_path = os.path.join(output_folder, f"image_{image_count:04d}.jpg")
        cv2.imwrite(image_path, frame)
        print(f"Đã lưu: {image_path}")
        image_count += 1

    # Hiển thị frame để theo dõi
    # cv2.imshow("Camera", frame)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

# Giải phóng tài nguyên
cap.release()
cv2.destroyAllWindows()
