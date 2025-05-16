import cv2
import os

def images_to_video(image_folder, output_video, fps=30):
    # Lấy danh sách file ảnh và sắp xếp theo thứ tự
    images = [img for img in os.listdir(image_folder) if img.endswith(('.png', '.jpg', '.jpeg'))]
    from natsort import natsorted
    images = natsorted(images)
    
    if not images:
        print("Không tìm thấy ảnh trong thư mục.")
        return
    
    # Đọc kích thước ảnh đầu tiên để thiết lập kích thước video
    first_image_path = os.path.join(image_folder, images[0])
    frame = cv2.imread(first_image_path)
    height, width, _ = frame.shape
    
    # Khởi tạo đối tượng VideoWriter
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Định dạng MP4
    video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))
    
    for image in images:
        img_path = os.path.join(image_folder, image)
        frame = cv2.imread(img_path)
        if frame is None:
            print(f"Lỗi khi đọc ảnh: {img_path}")
            continue
        video.write(frame)
    
    video.release()
    print(f"Video đã được tạo: {output_video}")

# Gọi hàm để tạo video từ thư mục ảnh
images_to_video('D:/Documents/Researches/2024_Project/RTK_GPS/Path-Planning/Frenet-Frame/Optimal Trajectory in a Frenet Frame/animation_frames_1', 'output_video9.mp4', fps=10)
