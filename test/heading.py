import numpy as np

def convert_yaw(yaw_deg_system1, yaw_offset=90):
    """
    Chuyển đổi góc yaw từ hệ tọa độ 1 sang hệ tọa độ 2.
    
    Args:
        yaw_deg_system1 (float): Góc yaw trong hệ tọa độ 1 (độ).
        yaw_offset (float): Độ lệch góc yaw giữa hệ tọa độ 1 và 2 (độ, mặc định là 100).
        
    Returns:
        float: Góc yaw trong hệ tọa độ 2 (độ).
    """
    # Chuyển góc từ độ sang radian
    yaw_rad_system1 = np.deg2rad(yaw_deg_system1)
    yaw_offset_rad = np.deg2rad(yaw_offset)
    
    # Tính góc yaw trong hệ tọa độ 2
    yaw_rad_system2 = yaw_rad_system1 - yaw_offset_rad
    
    # Đảm bảo góc nằm trong khoảng [0, 360) độ
    yaw_deg_system2 = (90 + (90 - np.rad2deg(yaw_rad_system2))) % 360
    
    return yaw_deg_system2

# Ví dụ sử dụng
yaw_system1 = 100 # Góc yaw trong hệ tọa độ 1
yaw_system2 = convert_yaw(yaw_system1)
print(f"Góc yaw trong hệ tọa độ 1: {yaw_system1}°")
print(f"Góc yaw trong hệ tọa độ 2: {yaw_system2}°")
