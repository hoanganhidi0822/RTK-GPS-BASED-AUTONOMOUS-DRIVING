import numpy as np

def transform_obstacle_to_global(x_vehicle, y_vehicle, heading, x_obstacle, y_obstacle):
    # Ma trận chuyển đổi
    T = np.array([
        [np.cos(heading), -np.sin(heading), x_vehicle],
        [np.sin(heading), np.cos(heading), y_vehicle],
        [0, 0, 1]
    ])
    
    # Tọa độ vật cản trong hệ tọa độ xe
    obstacle_local = np.array([x_obstacle, y_obstacle, 1])
    
    # Chuyển đổi sang hệ toàn cục
    obstacle_global = np.dot(T, obstacle_local)
    
    # Kết quả tọa độ toàn cục
    return obstacle_global[0], obstacle_global[1]

# Ví dụ
x_vehicle = 10   # Tọa độ xe trong hệ toàn cục (x)
y_vehicle = 20   # Tọa độ xe trong hệ toàn cục (y)
heading = np.radians(45)  # Góc hướng của xe (độ -> radian)
x_obstacle = 5    # Tọa độ vật cản so với xe (x)
y_obstacle = 3    # Tọa độ vật cản so với xe (y)

X_global, Y_global = transform_obstacle_to_global(x_vehicle, y_vehicle, heading, x_obstacle, y_obstacle)
print(f"Tọa độ vật cản trong hệ toàn cục: ({X_global:.2f}, {Y_global:.2f})")
