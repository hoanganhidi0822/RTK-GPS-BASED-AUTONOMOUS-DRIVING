import numpy as np

def intrinsic_to_camera(x, y, depth, fx, fy, cx, cy):
    """
    Chuyển từ tọa độ pixel 2D trong ảnh sang hệ tọa độ camera 3D.
    
    Args:
        x, y: Tọa độ pixel trong ảnh.
        depth: Giá trị độ sâu đo được.
        fx, fy: Tiêu cự theo trục x và y.
        cx, cy: Tọa độ tâm ảnh.
    
    Returns:
        np.array([Xc, Yc, Zc]): Tọa độ 3D trong hệ camera.
    """
    Xc = (x - cx) * depth / fx
    Yc = (y - cy) * depth / fy
    Zc = depth
    return np.array([Xc, Yc, Zc, 1])  # Thêm 1 để dùng ma trận đồng nhất

def camera_to_vehicle(camera_coords, angle_deg, translation):
    """
    Chuyển từ hệ tọa độ camera sang hệ tọa độ xe.

    Args:
        camera_coords: Tọa độ trong hệ camera [Xc, Yc, Zc, 1].
        angle_deg: Góc nghiêng của camera so với mặt đường (độ).
        translation: Tịnh tiến camera trong hệ tọa độ xe [tx, ty, tz].

    Returns:
        np.array([Xv, Yv, Zv, 1]): Tọa độ trong hệ xe.
    """
    angle_rad = np.radians(angle_deg)
    R = np.array([
        [1, 0, 0],
        [0, np.cos(-angle_rad), -np.sin(-angle_rad)],
        [0, np.sin(-angle_rad),  np.cos(-angle_rad)]
    ])

    T = np.array(translation).reshape(3, 1)

    # Ma trận chuyển đổi đồng nhất
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = T.flatten()

    # Chuyển đổi tọa độ
    vehicle_coords = np.dot(H, camera_coords)
    return vehicle_coords

def project_to_ground(vehicle_coords):
    """
    Chiếu tọa độ 3D trong hệ xe xuống mặt phẳng 2D.

    Args:
        vehicle_coords: Tọa độ trong hệ xe [Xv, Yv, Zv, 1].

    Returns:
        np.array([Xg, Yg]): Tọa độ 2D trên mặt phẳng mặt đường.
    """
    Xv, Yv, Zv, _ = vehicle_coords
    return np.array([Xv, Yv])

# === Ví dụ sử dụng ===
if __name__ == "__main__":
    # Tọa độ pixel từ ảnh và giá trị depth
    x, y = 320, 260  # Tọa độ pixel trong ảnh
    depth = 2.5       # Khoảng cách đo được (m)

    # Thông số nội tại của camera
    fx, fy = 800, 800   # Tiêu cự theo trục x và y
    cx, cy = 320, 240   # Tọa độ tâm ảnh

    # Góc nghiêng camera và tịnh tiến trong hệ xe
    angle_deg = 30       # Góc nghiêng (độ)
    translation = [0, 0, 1]  # Camera cách tâm xe 1m theo trục Z

    # Chuyển từ tọa độ pixel sang hệ tọa độ camera
    camera_coords = intrinsic_to_camera(x, y, depth, fx, fy, cx, cy)

    # Chuyển từ hệ camera sang hệ tọa độ xe
    vehicle_coords = camera_to_vehicle(camera_coords, angle_deg, translation)

    # Chiếu xuống mặt phẳng 2D
    ground_coords = project_to_ground(vehicle_coords)

    print("Tọa độ trong hệ xe:", ground_coords)
