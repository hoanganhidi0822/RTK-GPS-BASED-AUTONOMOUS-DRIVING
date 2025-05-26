import os

image_dir = '/mnt/NewVolume/Documents/Researches/2024_Project/SegFormer/input/data/data/DatasetUTE/val/images'
label_dir = '/mnt/NewVolume/Documents/Researches/2024_Project/SegFormer/input/data/data/DatasetUTE/val/labels'

# Lấy danh sách tên file (không phần mở rộng) từ folder images
image_names = {os.path.splitext(f)[0] for f in os.listdir(image_dir) if f.endswith(('.jpg', '.png', '.jpeg'))}

# Duyệt các file trong labels/
for label_file in os.listdir(label_dir):
    label_name, ext = os.path.splitext(label_file)
    if label_name not in image_names:
        label_path = os.path.join(label_dir, label_file)
        os.remove(label_path)
        print(f"🗑️ Đã xóa: {label_path}")
