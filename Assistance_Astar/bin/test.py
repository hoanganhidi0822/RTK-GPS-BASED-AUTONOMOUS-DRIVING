import numpy as np
import matplotlib.pyplot as plt
from math import radians, sin, cos, sqrt, atan2

def haversine(coord1, coord2):
    """
    Tính khoảng cách giữa hai tọa độ GPS (vĩ độ, kinh độ) theo mét.
    """
    R = 6371.0  # Bán kính Trái Đất (km)

    lat1, lon1 = radians(coord1[0]), radians(coord1[1])
    lat2, lon2 = radians(coord2[0]), radians(coord2[1])

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance = R * c  # Kết quả khoảng cách tính bằng km
    return distance * 1000  # Chuyển sang mét

def project_point_to_line_gps(A, B, I):
    """
    Chiếu điểm I lên đoạn thẳng AB trong không gian GPS và tính tọa độ điểm H.
    """
    A = np.array(A)
    B = np.array(B)
    I = np.array(I)

    AB = B - A
    AI = I - A

    AB_squared = np.dot(AB, AB)
    if AB_squared == 0:
        return None, float('inf')  # A và B trùng nhau

    t = np.dot(AI, AB) / AB_squared
    t = max(0, min(1, t))  # Giới hạn t trong [0, 1]

    H = A + t * AB
    distance = haversine(I, H)
    return H, distance

def update_segments_with_projection(points, segments, I, threshold):
    """
    Cập nhật segments bằng cách thêm điểm chiếu H nếu khoảng cách từ I đến đoạn thẳng nhỏ hơn ngưỡng.
    """
    added_points = []  # Lưu tọa độ các điểm H được thêm
    updated_segments = []  # Danh sách các đoạn mới

    for seg in segments:
        A, B = points[seg[0]], points[seg[1]]
        H, distance = project_point_to_line_gps(np.array(A), np.array(B), np.array(I))
        if distance < threshold:
            H_name = f"H{len(added_points) + 1}"  # Tạo tên điểm H
            added_points.append((H_name, H))  # Thêm điểm H
            updated_segments.append((seg[0], H_name))  # Đoạn mới từ A đến H
            updated_segments.append((H_name, seg[1]))  # Đoạn mới từ H đến B
        else:
            updated_segments.append(seg)  # Giữ nguyên đoạn ban đầu nếu không có điểm H

    # Cập nhật danh sách segments
    graph = {}
    for start, end in updated_segments:
        if start not in graph:
            graph[start] = []
        graph[start].append(end)

    # Cập nhật các điểm mới vào danh sách points
    for H_name, H_coord in added_points:
        points[H_name] = H_coord

    return graph, points, updated_segments, added_points

def plot_segments(points, segments, I, added_points):
    """
    Vẽ đồ thị các đoạn thẳng và điểm.
    """
    fig, ax = plt.subplots(figsize=(8, 8))

    # Vẽ các đoạn thẳng
    for seg in segments:
        A = points[seg[0]]
        B = points[seg[1]]
        ax.plot([A[1], B[1]], [A[0], B[0]], 'b-')

    # Vẽ điểm I
    ax.plot(I[1], I[0], 'ro', label="I (Chiếu)")

    # Vẽ các điểm H được thêm
    for H_name, H_coord in added_points:
        ax.plot(H_coord[1], H_coord[0], 'go', label=f"{H_name} (Thêm)")

    # Vẽ các điểm góc
    for name, coord in points.items():
        ax.plot(coord[1], coord[0], 'bo')
        ax.text(coord[1] + 0.001, coord[0] + 0.001, name, fontsize=10)

    # Cấu hình đồ thị
    ax.set_aspect('equal')
    ax.set_title("Cập nhật segments với điểm H (GPS)")
    ax.legend()
    plt.grid()
    plt.show()

# Dữ liệu đầu vào
points = {
    "A": (10.8532733433, 106.7715069217),
    "BB": (10.852302, 106.771424),
    "B": (10.8514838933, 106.7713101400),
    "C": (10.851238, 106.772669),
    "CC": (10.851554, 106.772746),
    "D": (10.851198, 106.773302),
    "DD": (10.851641, 106.773369),
    "E": (10.852292, 106.773450),
    "F": (10.852364, 106.772835),
    "G": (10.853240, 106.772932),
    "H": (10.853319, 106.772592),
    "I": (10.853541, 106.772572),
    "K": (10.853686, 106.771636),
}
segments = [
    ("A", "BB"), ("BB", "B"), ("B", "C"), ("C", "D"), ("C", "CC"),
    ("CC", "F"), ("D", "DD"), ("DD", "E"), ("E", "F"), ("F", "G"),
    ("G", "H"), ("H", "I"), ("I", "K"), ("K", "A")
]
I = (10.8512216567, 106.7725038067)  # Điểm cần chiếu
threshold = 10  # Ngưỡng khoảng cách (mét)

# Thực thi
graph, points, updated_segments, added_points = update_segments_with_projection(points, segments, I, threshold)
plot_segments(points, updated_segments, I, added_points)
