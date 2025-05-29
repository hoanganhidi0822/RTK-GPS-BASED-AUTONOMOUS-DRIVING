import os
import cv2
import random
from tqdm import tqdm
from pathlib import Path
import albumentations as A

# Đường dẫn folder
IMAGE_DIR = "Data/images"
MASK_DIR = "Data/labels"
AUG_IMAGE_DIR = "augmented/images"
AUG_MASK_DIR = "augmented/labels"

# Tạo folder đầu ra nếu chưa có
os.makedirs(AUG_IMAGE_DIR, exist_ok=True)
os.makedirs(AUG_MASK_DIR, exist_ok=True)

# Danh sách các augmentations
augmentations = [
    A.HorizontalFlip(p=1.0),
    # A.RandomBrightness(limit=(-0.4, -0.2), p=1.0),  # tối vừa
    # A.RandomBrightness(limit=(-0.8, -0.6), p=1.0),  # rất tối
    A.RandomBrightnessContrast(brightness_limit=(-0.6, 0.5), contrast_limit=(0.5, 0.7), p=1.0),

    A.Rotate(limit=20, p=1.0,crop_border=1),  # xoay nhẹ ±15 độ
    A.GaussNoise(var_limit=(10.0, 50.0), p=1.0),
    A.MotionBlur(blur_limit=5, p=1.0),
    A.RandomBrightnessContrast(p=1.0),
    A.ElasticTransform(p=1.0, alpha=1, sigma=50, alpha_affine=50),
]

# Lặp qua từng ảnh và mask
image_files = sorted(os.listdir(IMAGE_DIR))

for img_name in tqdm(image_files):
    img_path = os.path.join(IMAGE_DIR, img_name)
    base_name = Path(img_name).stem
    mask_path = os.path.join(MASK_DIR, f"{base_name}.png")  # đúng đuôi mask

    image = cv2.imread(img_path)
    mask = cv2.imread(mask_path)

    if image is None:
        print(f"⚠️ Cannot read image: {img_path}")
        continue
    if mask is None:
        print(f"⚠️ Cannot read mask: {mask_path}")
        continue

    # Lưu ảnh gốc
    cv2.imwrite(os.path.join(AUG_IMAGE_DIR, f"{base_name}_orig.jpg"), image)
    cv2.imwrite(os.path.join(AUG_MASK_DIR, f"{base_name}_orig.png"), mask)

    for i, aug in enumerate(augmentations):
        augmented = aug(image=image, mask=mask)
        aug_img = augmented["image"]
        aug_mask = augmented["mask"]

        cv2.imwrite(os.path.join(AUG_IMAGE_DIR, f"{base_name}_aug{i}.jpg"), aug_img)
        cv2.imwrite(os.path.join(AUG_MASK_DIR, f"{base_name}_aug{i}.png"), aug_mask)