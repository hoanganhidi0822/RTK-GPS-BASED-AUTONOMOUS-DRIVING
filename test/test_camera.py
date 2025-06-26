
import cv2
import matplotlib.pyplot as plt


cap = cv2.VideoCapture(2)

plt.ion()  # Bật chế độ interactive cho matplotlib
fig, ax = plt.subplots(figsize=(10, 6))

while True:
    ret, frame = cap.read()
    if not ret:
        break

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    

    # Hiển thị bằng matplotlib
    ax.clear()
    ax.imshow(seg_map)
    ax.set_title(f"Góc lái: {angle}°")
    ax.axis("off")
    plt.pause(0.1)

    if plt.get_fignums() == []:  # Nếu người dùng đóng cửa sổ
        break

cap.release()
plt.close()
