import argparse
import cv2
import numpy as np
import torch
from OBSTACLES.depth_anything_v2.dpt import DepthAnythingV2
import matplotlib
from ultralytics import YOLO
import time
from scipy.spatial.transform import Rotation as R
import config as cf
from collections import deque
from OBSTACLES.Segformer.road_segmentation import get_steering_angle 
camera_index = 0

cf.det_image = np.zeros((480, 640, 3))
cf.depth_image = np.zeros((480, 640, 3))
cf.obstacles = []
cf.persons = []
cf.camera_error = 0
cf.seg_mode = 0
cf.seg_steer = 0
cf.is_intersection = 0

DEPTH_SCALE_FACTOR = 1.25
MAX_DEPTH = 30
MAX_X = 4

# Select device
device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
print(f"Using device: {device}")
# Load YOLO model
model = YOLO('yolo11n.pt').to(device)
# Vehicle parameters
car_x = 0
car_y = 0
car_yaw = np.deg2rad(90.0)  # orientation of the car

parser = argparse.ArgumentParser(description='Depth Anything V2 with Camera Input')
parser.add_argument('--input-size', type=int, default=518)
parser.add_argument('--encoder', type=str, default='vits', choices=['vits', 'vitb', 'vitl', 'vitg'])
parser.add_argument('--grayscale', dest='grayscale', action='store_true', help='Do not apply colorful palette')
args = parser.parse_args()

# Load Depth Estimation Model
model_configs = {
    'vits': {'encoder': 'vits', 'features': 64,  'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
    'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
}

# Cấu hình tối ưu cho GPU
def optimize_for_gpu(model):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = model.to(device)

    if device.type == "cuda":
        # Bật tối ưu hóa CUDA
        torch.backends.cudnn.benchmark = True
        torch.backends.cuda.matmul.allow_tf32 = True  # Dùng TF32 trên Ampere+
        torch.set_float32_matmul_precision('high')

        # Bật Flash Attention và Memory Efficient Attention nếu có
        if hasattr(torch.backends.cuda, "enable_flash_sdp"):
            torch.backends.cuda.enable_flash_sdp(True)
            torch.backends.cuda.enable_mem_efficient_sdp(True)

        # Nếu model hỗ trợ, bật Gradient Checkpointing để giảm bộ nhớ tiêu thụ
        if hasattr(model, "gradient_checkpointing_enable"):
            model.gradient_checkpointing_enable()

        # Sử dụng Torch 2.0+ để biên dịch model tối ưu hơn
        if torch.__version__ >= '2.0':
            model = torch.compile(model, mode="max-autotune")

    return model

depth_anything = DepthAnythingV2(**model_configs[args.encoder])
depth_anything.load_state_dict(torch.load(f'OBSTACLES/checkpoints/depth_anything_v2_{args.encoder}.pth', map_location='cuda'))
depth_anything = optimize_for_gpu(depth_anything).eval()

cmap = matplotlib.colormaps.get_cmap('Spectral_r')
camera_matrix = np.loadtxt('OBSTACLES/camera_param/camera_matrix.txt',dtype=np.float32)
dist_coeffs = np.loadtxt('OBSTACLES/camera_param/distortion_coefficients.txt',dtype=np.float32)
map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, camera_matrix, (640, 480), cv2.CV_16SC2)

# Khởi tạo danh sách vật cản
obstacles = []
persons = []
fps_window = deque(maxlen=30)  # lưu 30 giá trị FPS gần nhất

def process_depth():
    # Open camera
    max_retries = 5
    for attempt in range(max_retries):
        cap = cv2.VideoCapture(camera_index)
        cap.set(cv2.CAP_PROP_FPS, 60)
        if cap.isOpened():
            break
        else:
            print(f"Retrying to connect camera... ({attempt + 1}/{max_retries})")
            time.sleep(1)
    else:
        print("Error: Cannot open camera after retries")
        cf.camera_error = 1
        return


    inv_K = np.linalg.inv(np.array([
        [267,  0, 320],
        [0, 267, 245],
        [0, 0, 1]
    ]))

    global obstacles
    global persons
    fps = 0
    i = 0
    count_frame = 2
    results = []
    time_start = 0
    while 1:
        
        ret, raw_frame = cap.read()
        if not ret:
            print("Error: Lost connection to camera")
            cf.camera_error = 1
            break

        count_frame += 1
        raw_frame = cv2.remap(raw_frame, map1, map2, interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2RGB)

        # Mat tin hieu gps
        if cf.seg_mode == 1:
            angle, overlay, is_intersection = get_steering_angle(rgb, debug=1)
            cf.seg_steer = angle
            cf.is_intersection = is_intersection

            if  count_frame % 4 == 0:
                with torch.no_grad(), torch.amp.autocast('cuda'):  
                    results = model(raw_frame, verbose=False, device=device,classes=[0, 1, 2,3,4,7])
                    depth_map = depth_anything.infer_image(raw_frame, args.input_size)
                    
                depth_map = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min()) * 65535
            # cf.image = overlay 
            # print(f"[INFO] Steering angle: {angle:.2f} degrees")

        # Infer depth
        elif cf.seg_mode == 0 and count_frame % 3 == 0:
            time_start = time.time()
            with torch.no_grad(), torch.amp.autocast('cuda'):  
                results = model(raw_frame, verbose=False, device=device,classes=[0, 1, 2,3,4,7])
                depth_map = depth_anything.infer_image(raw_frame, args.input_size)
        
            depth_map = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min()) * 65535
            # depth_visulize = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min()) * 255.0
            
            # Run YOLO Detector
        found_new_data = False
        for predictions in results:
            for bbox in predictions.boxes:
                class_id = int(bbox.cls.cpu().numpy()[0])
                if class_id not in [0, 1, 2, 3, 4, 7]:  # 0: person, 2: car
                    continue
                xmin, ymin, xmax, ymax = bbox.xyxy[0].cpu().numpy()
                if xmax - xmin < 10 or ymax - ymin < 10:
                    continue  # skip small boxes

                depth_values_bbox = depth_map[int(ymin):int(ymax), int(xmin):int(xmax)]
                if depth_values_bbox.size == 0:
                    continue

                depth_value = np.median(depth_values_bbox)
                # Bỏ qua nếu depth không hợp lệ (0 hoặc quá nhỏ)
                if depth_value <= 1e-3:
                    continue
                DEPTH_SCALE_FACTOR = 1.25
                z_camera = (65535 / depth_value) * DEPTH_SCALE_FACTOR
                center_x = (xmin + xmax) / 2
                center_y = (ymin + ymax) / 2

                # intrinsic_matrix = np.array([
                #     [267,  0  , 293],
                #     [ 0 , 267 , 245],
                #     [ 0 ,  0  ,  1 ]
                # ])
                
                """ 267.97  0.00    293.59
                    0.00    265.42  245.49
                    0.00    0.00     1.00"""

                pixel_coords = np.array([center_x, center_y, 1])
                camera_coords = inv_K @ (pixel_coords * z_camera)

                rotation_matrix = R.from_euler('x', 0, degrees=True).as_matrix()
                real_coords = rotation_matrix @ camera_coords
                x_real, z_real = real_coords[0], real_coords[2]
                
                if z_real < 30 and abs(x_real) <= 8:
                    if class_id == 0:
                        persons.append((x_real, z_real))
                    elif class_id == 2 or class_id == 5 or class_id == 7:
                        obstacles.append((x_real, z_real))
                        # obstacles.append((x_real, z_real + 1.0))
                        
                    
                    cv2.rectangle(raw_frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 0, 255), 1)
                    # Hiển thị thông tin
                    cv2.putText(raw_frame, f"X: {z_real:.2f} m", (int(xmin), int(ymax) + 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cf.obstacles = obstacles
        cf.persons = persons
        obstacles = []
        persons = []

        try:
            if cf.seg_mode == 1:
                cf.image = overlay
            else:
                cf.image = raw_frame
        except NameError:
            cf.image = raw_frame  # fallback an toàn nếu overlay chưa được tạo


        # Tính FPS trung bình
        delta_t = time.time() - time_start
        instantaneous_fps = 1.0 / delta_t if delta_t > 0 else 0.0
        fps_window.append(instantaneous_fps)

        if len(fps_window) == fps_window.maxlen:
            avg_fps = sum(fps_window) / len(fps_window)
            if count_frame %10  == 0:
                print(f" --- [INFO] Perception FPS: {avg_fps:.2f}")
