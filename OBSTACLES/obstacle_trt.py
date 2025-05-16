import cv2
import numpy as np
import pycuda.autoinit
import pycuda.driver as cuda
import tensorrt as trt
import time
import matplotlib
from ultralytics import YOLO
import torch
from scipy.spatial.transform import Rotation as R
import config as cf
device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
print(f"Using device: {device}")
# Load YOLO model
model = YOLO('yolov8n.pt').to(device)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 60)  # Điều chỉnh FPS

if not cap.isOpened():
    print("Error: Cannot open camera")
    exit()

cmap = matplotlib.colormaps.get_cmap('Spectral_r')
camera_matrix = np.loadtxt('D:/Documents/Researches/2024_Project/RTK_GPS/Waypoint-Tracking/Pure-pursuit/frenet-optimal-trajectory/OBSTACLES/camera_param/camera_matrix.txt',dtype=np.float32)
dist_coeffs = np.loadtxt('D:/Documents/Researches/2024_Project/RTK_GPS/Waypoint-Tracking/Pure-pursuit/frenet-optimal-trajectory/OBSTACLES/camera_param/distortion_coefficients.txt',dtype=np.float32)
map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, camera_matrix, (640, 480), cv2.CV_16SC2)
# Khởi tạo danh sách vật cản
obstacles = []

cf.image = np.zeros((480, 1280, 3))
cf.obstacles = []

def preprocess_frame(frame, input_shape):
    frame_resized = cv2.resize(frame, (input_shape[2], input_shape[3]))
    frame_resized = frame_resized.astype(np.float32) / 255.0
    frame_resized = np.transpose(frame_resized, (2, 0, 1))  # HWC -> CHW
    return np.expand_dims(frame_resized, axis=0)
cmap = matplotlib.colormaps.get_cmap('Spectral_r')
def process_depth(engine_path):
    global obstacles
    fps = 0
    
    logger = trt.Logger(trt.Logger.WARNING)
    with open(engine_path, 'rb') as f, trt.Runtime(logger) as runtime:
        engine = runtime.deserialize_cuda_engine(f.read())
    
    with engine.create_execution_context() as context:
        input_shape = context.get_tensor_shape("input")
        output_shape = context.get_tensor_shape("output")
        
        h_input = cuda.pagelocked_empty(trt.volume(input_shape), dtype=np.float32)
        h_output = cuda.pagelocked_empty(trt.volume(output_shape), dtype=np.float32)
        d_input = cuda.mem_alloc(h_input.nbytes)
        d_output = cuda.mem_alloc(h_output.nbytes)
        stream = cuda.Stream()
        
        # cap = cv2.VideoCapture(0)
        
        while cap.isOpened():
            time_start = time.time()
            ret, frame = cap.read()
            if not ret:
                break
            raw_frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
            input_image = preprocess_frame(raw_frame, input_shape)
            np.copyto(h_input, input_image.ravel())
            
            cuda.memcpy_htod_async(d_input, h_input, stream)
            context.set_tensor_address("input", int(d_input))
            context.set_tensor_address("output", int(d_output))
            context.execute_async_v3(stream_handle=stream.handle)
            cuda.memcpy_dtoh_async(h_output, d_output, stream)
            stream.synchronize()
            depth = h_output
            
            with torch.no_grad(), torch.cuda.amp.autocast():  
                results = model(raw_frame, verbose=False, device=device,classes=[0, 1, 2])
            
            depth_map= np.reshape(depth, output_shape[2:])
            depth_map = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min()) * 65535
            depth_visulize = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min()) * 255.0
            # depth = depth.astype(np.uint8)
            # depth = cv2.resize(depth, (frame.shape[1], frame.shape[0]))
            # colored_depth = cv2.applyColorMap(depth, cv2.COLORMAP_PLASMA)
            
            f# Run YOLO Detector
            for predictions in results:
                for bbox in predictions.boxes:
                    class_id = int(bbox.cls.cpu().numpy()[0])
                    if class_id != 2:  # Theo COCO dataset, class ID 2 là "car"
                        continue
                    xmin, ymin, xmax, ymax = bbox.xyxy[0].cpu().numpy()
                    depth_values_bbox = depth_map[int(ymin):int(ymax), int(xmin):int(xmax)]
                    if depth_values_bbox.size == 0:
                        continue

                    depth_value = np.median(depth_values_bbox)
                    scale_factor = 1.00
                    z_camera = (65535 / depth_value) * scale_factor
                    center_x = (xmin + xmax) / 2
                    center_y = (ymin + ymax) / 2

                    intrinsic_matrix = np.array([
                        [267,  0  , 293],
                        [ 0 , 267 , 245],
                        [ 0 ,  0  ,  1 ]
                    ])
                    
                    """ 267.97  0.00    293.59
                        0.00    265.42  245.49
                        0.00    0.00     1.00"""

                    pixel_coords = np.array([center_x, center_y, 1])
                    camera_coords = np.linalg.inv(intrinsic_matrix) @ (pixel_coords * z_camera)

                    rotation_matrix = R.from_euler('x', 0, degrees=True).as_matrix()
                    real_coords = rotation_matrix @ camera_coords

                    x_real, z_real = real_coords[0], real_coords[2]
                
                    if z_real < 17:
                        cv2.rectangle(raw_frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 0, 255), 2)
                        # Hiển thị thông tin
                        offset_text = f"X: {x_real:.3f} m, Z: {z_real:.3f} m"
                        cv2.putText(raw_frame, f"Dist: {z_real:.2f} m", (int(xmin), int(ymax) + 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(raw_frame, offset_text, (int(xmin), int(ymax) + 35),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    # Thêm tọa độ vật thể vào danh sách
                    obstacles.append((x_real, z_real))
                    
                
            depth_display = depth_visulize.astype(np.uint8)
        
            depth_display = cv2.applyColorMap(depth_display, cv2.COLORMAP_PLASMA)

            if depth_display.shape[:2] != raw_frame.shape[:2]:
                depth_display = cv2.resize(depth_display, (raw_frame.shape[1], raw_frame.shape[0]))

            combined_frame = cv2.hconcat([raw_frame, depth_display])
            cf.obstacles = obstacles
            # cf.image = combined_frame
            
            obstacles = []
            # # Visualization
            alpha = 0.5
            delta_t = time.time() - time_start
            if delta_t > 0:
                fps = (1 - alpha) * fps + alpha * (1 / delta_t)
            
            print(f"percept fps: {fps}")
            
            cv2.imshow("img", combined_frame)
            cv2.waitKey(1)

# if __name__ == "__main__":
#     engine_path = "D:/Documents/Researches/2024_Project/Depth Map-Based Obstacle Position Detection/Depth-Anything-V2-main/depth_anything_v2_vits.engine"
#     process_depth(engine_path)