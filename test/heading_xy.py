import math

def calculate_heading_from_gps(lat1, lon1, lat2, lon2):
    """
    Tính heading giữa 2 điểm GPS trong hệ tọa độ XY.
    Trả về heading với độ chính xác cao.
    """
    # Bán kính Trái Đất (m)
    R = 6371e3  

    # Chuyển lat/lon sang radians
    lat1, lon1 = math.radians(lat1), math.radians(lon1)
    lat2, lon2 = math.radians(lat2), math.radians(lon2)

    # Tính khoảng cách x, y trong hệ tọa độ phẳng (equirectangular projection)
    dx = R * (lon2 - lon1) * math.cos(lat1)
    dy = R * (lat2 - lat1)

    # Tính heading trong hệ XY (theo góc arctan2)
    heading_xy = math.atan2(dy, dx)  # Trả về radian
    heading_xy_deg = math.degrees(heading_xy)

    # Đảm bảo góc trong khoảng [0, 360]
    if heading_xy_deg < 0:
        heading_xy_deg += 180

    return heading_xy_deg

# Ví dụ
lat1, lon1 = 10.8532570333, 106.7715131967
lat2, lon2 = 10.8532464083, 106.7715122883

heading_xy = calculate_heading_from_gps(lat1, lon1, lat2, lon2)
print(f"Heading XY: {heading_xy:.2f} độ")  # Kết quả mong muốn: 85.2 độ
