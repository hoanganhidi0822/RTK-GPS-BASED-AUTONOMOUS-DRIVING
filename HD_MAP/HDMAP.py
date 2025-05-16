import math

def XY_WAYPOINTS_MAP(input_file):
    """
    Hàm chuyển đổi dữ liệu GPS từ file TXT sang tọa độ x, y.
    
    Parameters:
        input_file (str): Đường dẫn tới file TXT chứa dữ liệu GPS (latitude, longitude).
        
    Returns:
        tuple: Hai danh sách (wx, wy) chứa tọa độ x và y tương ứng.
    """
    
    def latlon_to_xy(lat0, lon0, lat=10.8532570333, lon=106.7715131967):
        """
        Chuyển đổi latitude và longitude sang x, y dùng equirectangular projection.
        """
        R = 6371e3  # Bán kính Trái Đất (mét)
        x = R * math.radians(lon - lon0) * math.cos(math.radians(lat0))
        y = R * math.radians(lat - lat0)
        return x, y

    wx = []  # Danh sách lưu tọa độ x
    wy = []  # Danh sách lưu tọa độ y
    
    # Đọc và xử lý file
    try:
        with open(input_file, 'r') as infile:
            for line in infile:
                line = line.strip()  # Loại bỏ khoảng trắng
                if not line:
                    continue  # Bỏ qua dòng trống
                
                try:
                    # Tách lat, lon
                    lat_str, lon_str = line.split(',')
                    lat = float(lat_str)
                    lon = float(lon_str)
                    
                    # Chuyển đổi sang tọa độ x, y
                    x, y = latlon_to_xy(lat, lon)
                    wx.append(x)
                    wy.append(y)
                except ValueError:
                    print(f"Bỏ qua dòng không hợp lệ: {line}")
                    continue
    except FileNotFoundError:
        print(f"Không tìm thấy file: {input_file}")
    
    return wx, wy

# Gọi hàm với file đầu vào
# input_file = '/mnt/NewVolume/Documents/Researches/2024_Project/RTK_GPS/Waypoint-Tracking/Pure-pursuit/frenet-optimal-trajectory/Assistance_Astar/merged_waypoints.txt'
input_file = "Assistance_Astar/merged_waypoints.txt"
# input_file = "/mnt/NewVolume/Documents/Researches/2024_Project/RTK_GPS/Waypoint-Tracking/Pure-pursuit/frenet-optimal-trajectory/HD_MAP/MAP_NGHIEM_THU.txt"
# input_file = "/mnt/NewVolume/Documents/Researches/2024_Project/ASTAR-HD-MAP/Astar/merged_waypoints.txt"
wx, wy = XY_WAYPOINTS_MAP(input_file)
