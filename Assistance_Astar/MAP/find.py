import glob
import math
import os

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Bán kính Trái Đất (m)
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c

def find_nearest_waypoint(input_lat, input_lon, folder_path="D:/Documents/Researches/2024_Project/Astar/MAP/"):
    txt_files = glob.glob(f"{folder_path}/*.txt")
    nearest_file = None
    nearest_point = None
    min_distance = float("inf")
    
    for file in txt_files:
        with open(file, "r") as f:
            for line in f:
                line = line.strip()
                if not line:  # Bỏ qua dòng trống
                    continue
                try:
                    lat, lon = map(float, line.split(","))
                    distance = haversine(input_lat, input_lon, lat, lon)
                    if distance < min_distance:
                        min_distance = distance
                        nearest_file = os.path.basename(file)  # Chỉ lấy tên file
                        nearest_point = (lat, lon)
                except ValueError:
                    print(f"Bỏ qua dòng không hợp lệ trong {file}: {line}")
    
    return nearest_file, nearest_point, min_distance


# Nhập tọa độ cần kiểm tra
input_lat, input_lon = 10.851151, 106.773146

# Gọi hàm tìm waypoint gần nhất
file_name, nearest_point, distance = find_nearest_waypoint(input_lat, input_lon)

if file_name and nearest_point:
    print(f"Waypoint gần nhất: {nearest_point} trong file: {file_name}, Khoảng cách: {distance:.2f} m")
else:
    print("Không tìm thấy waypoint gần nhất.")
