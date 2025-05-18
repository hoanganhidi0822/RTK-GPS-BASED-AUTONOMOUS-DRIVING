import matplotlib.pyplot as plt
import networkx as nx
import math
import numpy as np
from queue import PriorityQueue
import glob
import os
from math import radians, sin, cos, sqrt, atan2
from Assistance_Astar.overlay import *
from Assistance_Astar.location_finder  import *
from RTK_GPS.GPS_module import *
from main import gps_ser

class Graph:
    def __init__(self):
        # Define GPS coordinates for each node (latitude, longitude)
        self.coordinates = {
            "J":  (10.8507759083,106.7715805667), # Toa Trung Tam truoc
            "A":  (10.8532733433, 106.7715069217), # Khu C
            "BB": (10.852302    , 106.771424    ), # Khu D
            "B":  (10.8514838933, 106.7713101400),
            "TT": (10.8512819350, 106.7719588833), # Toa trung Tam
            "C":  (10.851238    ,     106.772669), # Toa Viet Duc
            "CC": (10.851554    ,     106.772746),
            "D":  (10.851198    ,     106.773302),
            "DD": (10.851641    ,     106.773369), # Maker Space
            "E":  (10.852292    ,     106.773450),
            "F":  (10.852364    ,     106.772835),
            "G":  (10.853240    ,     106.772932), # Go
            "H":  (10.853319    ,     106.772592),
            "I":  (10.853541    ,     106.772572),
            "K":  (10.853686    ,     106.771636),
        }

        self.segments = [
            ("A", "BB"),("BB", "A"), ("BB", "B"), ("B", "TT"),("TT", "C"), ("C", "D"), ("C", "CC"),("CC", "C"),("C", "TT"),("TT", "B"),("B", "BB"),
            ("CC", "F"), ("D", "DD"), ("DD", "E"),("E", "F"), ("F", "G"), ("G", "F"),("F", "CC"),
            ("H", "G"), ("H", "I"), ("I", "K"), ("K", "I"),("K", "A"), ("J", "B"), ("B", "J")
        ]
        self.threshold = 10  # Ngưỡng khoảng cách (mét)
        self.graph = {}
        self.weights = {}

    def haversine_distance(self, point1, point2):
        """Calculate Haversine distance between two GPS coordinates."""
        R = 6371.0  # Radius of the Earth in kilometers
        lat1, lon1 = point1
        lat2, lon2 = point2

        # Convert latitude and longitude from degrees to radians
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        # Haversine formula
        a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c * 1000  # Distance in meters

    def project_point_to_line(self, A, B, I):
        """
        Project point I onto line segment AB and calculate the projected point H.
        """
        A = np.array(A)
        B = np.array(B)
        I = np.array(I)

        AB = B - A
        AI = I - A

        AB_squared = np.dot(AB, AB)
        if AB_squared == 0:
            return None, float('inf')  # A and B are the same point

        t = np.dot(AI, AB) / AB_squared
        t = max(0, min(1, t))  # Clamp t to [0, 1]

        H = A + t * AB
        distance = self.haversine_distance(I, H)
        return H, distance

    def update_graph_with_projection(self, current_pos):
        """
        Update graph segments by adding projection points if within the threshold.
        """
        added_points = []  # Store added projection points
        updated_segments = []

        for seg in self.segments:
            A, B = self.coordinates[seg[0]], self.coordinates[seg[1]]
            H, distance = self.project_point_to_line(A, B, current_pos)

            if distance < self.threshold:
                H_name = "H1"
                added_points.append((H_name, H))
                updated_segments.append((seg[0], H_name))
                updated_segments.append((H_name, seg[1]))
            else:
                updated_segments.append(seg)

        for H_name, H_coord in added_points:
            self.coordinates[H_name] = tuple(H_coord)

        self.segments = updated_segments
        self.build_graph()

    def build_graph(self):
        """Build the graph structure and calculate weights."""
        self.graph = {}
        for start, end in self.segments:
            if start not in self.graph:
                self.graph[start] = []
            self.graph[start].append(end)
        self.calculate_weights()

    def calculate_weights(self):
        """Calculate weights for all edges."""
        self.weights = {}
        for from_node, neighbors in self.graph.items():
            for to_node in neighbors:
                self.weights[(from_node, to_node)] = self.haversine_distance(
                    self.coordinates[from_node], self.coordinates[to_node]
                )

    def neighbors(self, node):
        return self.graph.get(node, [])

    def get_cost(self, from_node, to_node):
        return self.weights.get((from_node, to_node), float('inf'))

    def heuristic(self, node, goal):
        return self.haversine_distance(self.coordinates[node], self.coordinates[goal])

def astar(graph, start, goal):
    from queue import PriorityQueue

    order = ['A', 'BB', 'B', 'TT', 'C', 'CC', 'F', 'G', 'H', 'I', 'K']
    queue = PriorityQueue()
    queue.put((0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while not queue.empty():
        _, current = queue.get()

        if current == goal:
            break

        for neighbor in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.get_cost(current, neighbor)
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + graph.heuristic(neighbor, goal)
                queue.put((priority, neighbor))
                came_from[neighbor] = current

    if goal not in came_from:
        return [], None  # No path found

    path = []
    directions = []
    node = goal
    while node != start:
        from_node = came_from[node]
        to_node = node
        path.append((from_node, to_node))

        # Chỉ xác định chiều nếu cả 2 node nằm trong danh sách order
        if from_node in order and to_node in order:
            if order.index(from_node) < order.index(to_node):
                directions.append(True)
            elif order.index(from_node) > order.index(to_node):
                directions.append(False)
            # nếu bằng nhau thì không thêm gì
        # nếu 1 trong 2 node không thuộc order → bỏ qua khi xét chiều

        node = from_node

    path.reverse()
    directions.reverse()

    # Loại bỏ duplicate chiều và xác định kết quả
    unique_dirs = set(directions)
    if len(unique_dirs) == 1:
        dir = unique_dirs.pop()
    elif len(unique_dirs) == 0:
        dir = "không xác định"  # không đủ thông tin
    else:
        dir = "hỗn hợp"

    return path, dir


# Cấu hình file waypoint cho từng đoạn đường
waypoint_files = {
    ('H1' , 'BB' )  : "Assistance_Astar/MAP/THUAN/waypoint_1.txt" ,
    ('H1' , 'DD' )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('H1' , 'B'  )  : "Assistance_Astar/MAP/THUAN/waypoint_2.txt" ,
    ('H1' , 'TT' )  : "Assistance_Astar/MAP/THUAN/waypoint_3.txt" ,
    ('H1' , 'C'  )  : "Assistance_Astar/MAP/THUAN/waypoint_4.txt" ,
    ('H1' , 'CC' )  : "Assistance_Astar/MAP/THUAN/waypoint_5.txt",
    ('H1' , 'F'  )  : "Assistance_Astar/MAP/THUAN/space.txt" ,
    ('BB' , 'B'  )  : "Assistance_Astar/MAP/THUAN/waypoint_2.txt" ,
    ('B'  , 'TT' )  : "Assistance_Astar/MAP/THUAN/waypoint_3.txt" ,
    ('TT' , 'C'  )  : "Assistance_Astar/MAP/THUAN/waypoint_4.txt" ,
    ('C'  , 'CC' )  : "Assistance_Astar/MAP/THUAN/waypoint_5.txt" ,
    ('CC' , 'F'  )  : "Assistance_Astar/MAP/THUAN/waypoints_6.txt",
    ('C'  , 'D'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('D'  , 'DD' )  : "Assistance_Astar/MAP/THUAN/waypoints_8.txt",
    ('DD' , 'E'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('E'  , 'F'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('F'  , 'G'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('G'  , 'H'  )  : "Assistance_Astar/MAP/THUAN/waypoints_7.txt",
    ('H'  , 'I'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('I'  , 'K'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('I'  , 'K'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('K'  , 'A'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    
    ('A'  , 'K'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_1.txt",
    ('K'  , 'I'  )  : "Assistance_Astar/MAP/NGHICH/space.txt"     ,
    ('I'  , 'H'  )  : "Assistance_Astar/MAP/NGHICH/space.txt"     ,
    ('I'  , 'H'  )  : "Assistance_Astar/MAP/NGHICH/space.txt"     ,
    ('H'  , 'G'  )  : "Assistance_Astar/MAP/NGHICH/space.txt"     ,
    ('G'  , 'F'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_2.txt",
    ('F'  , 'CC' )  : "Assistance_Astar/MAP/NGHICH/waypoint_2.txt",
    ('CC' , 'C'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_3.txt",
    ('C'  , 'TT' )  : "Assistance_Astar/MAP/NGHICH/space.txt"     ,
    ('TT' , 'B'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_4.txt",
    ('B'  , 'BB' )  : "Assistance_Astar/MAP/NGHICH/space.txt"     ,
    ('BB' , 'A'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_5.txt",
}


waypoint_files_1 = {
    ('H1' , 'BB' )  : "Assistance_Astar/MAP/THUAN/waypoint_1.txt" ,
    ('H1' , 'DD' )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('H1' , 'B'  )  : "Assistance_Astar/MAP/THUAN/waypoint_2.txt" ,
    ('H1' , 'TT' )  : "Assistance_Astar/MAP/THUAN/waypoint_3.txt" ,
    ('H1' , 'C'  )  : "Assistance_Astar/MAP/THUAN/waypoint_4.txt" ,
    ('H1' , 'CC' )  : "Assistance_Astar/MAP/THUAN/waypoint_5.txt",
    ('H1' , 'F'  )  : "Assistance_Astar/MAP/THUAN/space.txt" ,
    ('J' , 'H1'  )  : "Assistance_Astar/MAP/THUAN/waypoints_10_trungtam.txt" ,
    ('BB' , 'B'  )  : "Assistance_Astar/MAP/THUAN/waypoint_2.txt" ,
    ('B'  , 'TT' )  : "Assistance_Astar/MAP/THUAN/waypoint_3.txt" ,
    ('TT' , 'C'  )  : "Assistance_Astar/MAP/THUAN/waypoint_4.txt" ,
    ('C'  , 'CC' )  : "Assistance_Astar/MAP/THUAN/waypoint_5.txt" ,
    ('CC' , 'F'  )  : "Assistance_Astar/MAP/THUAN/waypoints_6.txt",
    ('C'  , 'D'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('D'  , 'DD' )  : "Assistance_Astar/MAP/THUAN/waypoints_8.txt",
    ('DD' , 'E'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('E'  , 'F'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('F'  , 'G'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('G'  , 'H'  )  : "Assistance_Astar/MAP/THUAN/waypoints_7.txt",
    ('H'  , 'I'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('I'  , 'K'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('I'  , 'K'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('K'  , 'A'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('J'  , 'B'  )  : "Assistance_Astar/MAP/THUAN/waypoints_10_trung_tam.txt"      ,
     ('B' , 'J'  )  : "Assistance_Astar/MAP/NGHICH/MAP_TRUNG_TAM_NGHICH.txt", #
    
}

waypoint_files_1_TT = {
    ('H1' , 'BB' )  : "Assistance_Astar/MAP/THUAN/waypoint_1.txt" ,
    ('H1' , 'DD' )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('H1' , 'B'  )  : "Assistance_Astar/MAP/THUAN/waypoints_10_trungtam.txt",
    ('H1' , 'TT' )  : "Assistance_Astar/MAP/THUAN/waypoint_3.txt" ,
    ('H1' , 'C'  )  : "Assistance_Astar/MAP/THUAN/waypoint_4.txt" ,
    ('H1' , 'CC' )  : "Assistance_Astar/MAP/THUAN/waypoint_5.txt",
    ('H1' , 'F'  )  : "Assistance_Astar/MAP/THUAN/space.txt" ,
    ('J' , 'H1'  )  : "Assistance_Astar/MAP/THUAN/space.txt" ,
    ('BB' , 'B'  )  : "Assistance_Astar/MAP/THUAN/waypoint_2.txt" ,
    ('B'  , 'TT' )  : "Assistance_Astar/MAP/THUAN/waypoint_3_tt.txt" ,
    ('TT' , 'C'  )  : "Assistance_Astar/MAP/THUAN/waypoint_4.txt" ,
    ('C'  , 'CC' )  : "Assistance_Astar/MAP/THUAN/waypoint_5.txt" ,
    ('CC' , 'F'  )  : "Assistance_Astar/MAP/THUAN/waypoints_6.txt",
    ('C'  , 'D'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('D'  , 'DD' )  : "Assistance_Astar/MAP/THUAN/waypoints_8.txt",
    ('DD' , 'E'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('E'  , 'F'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('F'  , 'G'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('G'  , 'H'  )  : "Assistance_Astar/MAP/THUAN/waypoints_7.txt",
    ('H'  , 'I'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('I'  , 'K'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('I'  , 'K'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('K'  , 'A'  )  : "Assistance_Astar/MAP/THUAN/space.txt"      ,
    ('J'  , 'B'  )  : "Assistance_Astar/MAP/THUAN/space.txt" ,  
    ('B' , 'J'  )   : "Assistance_Astar/MAP/NGHICH/MAP_TRUNG_TAM_NGHICH.txt", #
}
waypoint_files_2 = {
    ('H1' , 'BB' )  : "Assistance_Astar/MAP/NGHICH/waypoint_4.txt" ,
    ('H1' , 'B'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_4.txt" ,
    ('H1' , 'TT' )  : "Assistance_Astar/MAP/NGHICH/waypoint_3.txt" ,
    ('H1' , 'C'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_3.txt" ,
    ('H1' , 'CC' )  : "Assistance_Astar/MAP/NGHICH/waypoint_2.txt",
    ('H1' , 'F'  )  : "Assistance_Astar/MAP/NGHICH/space.txt" ,

    ('A'  , 'K'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_1.txt", # 
    ('K'  , 'I'  )  : "Assistance_Astar/MAP/NGHICH/space.txt"     , #
    ('I'  , 'H'  )  : "Assistance_Astar/MAP/NGHICH/space.txt"     , #
    ('I'  , 'H'  )  : "Assistance_Astar/MAP/NGHICH/space.txt"     , #
    ('H'  , 'G'  )  : "Assistance_Astar/MAP/NGHICH/space.txt"     , #
    ('G'  , 'F'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_2.txt", #
    ('F'  , 'CC' )  : "Assistance_Astar/MAP/NGHICH/waypoint_2.txt", #
    ('CC' , 'C'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_3.txt", #
    ('C'  , 'TT' )  : "Assistance_Astar/MAP/NGHICH/space.txt"     , #
    ('TT' , 'B'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_4.txt", #
    ('B'  , 'BB' )  : "Assistance_Astar/MAP/NGHICH/space.txt"     , #
    ('BB' , 'A'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_5.txt", #
    ('B' , 'J'  )   : "Assistance_Astar/MAP/NGHICH/MAP_TRUNG_TAM_NGHICH.txt", #  
}

waypoint_files_2_TT = {
    ('H1' , 'BB' )  : "Assistance_Astar/MAP/NGHICH/waypoint_4.txt" ,
    ('H1' , 'B'  )  : "Assistance_Astar/MAP/NGHICH/waypoints_10_trungtam.txt" ,
    ('J' , 'B'  )  : "Assistance_Astar/MAP/NGHICH/waypoints_10_trungtam.txt" ,
    ('H1' , 'TT' )  : "Assistance_Astar/MAP/NGHICH/waypoint_3.txt" ,
    ('H1' , 'C'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_3.txt" ,
    ('H1' , 'CC' )  : "Assistance_Astar/MAP/NGHICH/waypoint_2.txt",
    ('H1' , 'F'  )  : "Assistance_Astar/MAP/NGHICH/space.txt" ,

    
    ('A'  , 'K'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_1.txt", # 
    ('K'  , 'I'  )  : "Assistance_Astar/MAP/NGHICH/space.txt"     , #
    ('I'  , 'H'  )  : "Assistance_Astar/MAP/NGHICH/space.txt"     , #
    ('I'  , 'H'  )  : "Assistance_Astar/MAP/NGHICH/space.txt"     , #
    ('H'  , 'G'  )  : "Assistance_Astar/MAP/NGHICH/space.txt"     , #
    ('G'  , 'F'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_2.txt", #
    ('F'  , 'CC' )  : "Assistance_Astar/MAP/NGHICH/waypoint_2.txt", #
    ('CC' , 'C'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_3.txt", #
    ('C'  , 'TT' )  : "Assistance_Astar/MAP/NGHICH/space.txt"     , #
    ('TT' , 'B'  )  : "Assistance_Astar/MAP/NGHICH/space.txt", #
    ('B'  , 'BB' )  : "Assistance_Astar/MAP/NGHICH/waypoint_4_tt.txt", #
    ('BB' , 'A'  )  : "Assistance_Astar/MAP/NGHICH/waypoint_5.txt", #
    ('B' , 'J'  )   : "Assistance_Astar/MAP/NGHICH/trash.txt", #
}
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Bán kính Trái Đất (m)
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c

def load_waypoints(file_path):
    """ Đọc nội dung của file waypoint """
    try:
        with open(file_path, 'r') as file:
            waypoints = file.readlines()
        return waypoints
    except FileNotFoundError:
        print(f"⚠️ File {file_path} không tồn tại!")
        return []


def haversine(lat1, lon1, lat2, lon2):
    """Tính khoảng cách Haversine giữa hai tọa độ."""
    R = 6371000  # Bán kính Trái Đất (m)
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c  # Khoảng cách tính bằng mét

def project_point_on_line(p, a, b):
    """ Chiếu điểm P lên đoạn thẳng AB """
    px, py = p
    ax, ay = a
    bx, by = b

    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab_length_squared = abx**2 + aby**2

    # Hệ số t cho vị trí chiếu
    t = (apx * abx + apy * aby) / ab_length_squared
    t = max(0, min(1, t))  # Giữ t trong [0,1] để nằm trên đoạn AB

    # Tọa độ chiếu vuông góc
    projected_x = ax + t * abx
    projected_y = ay + t * aby

    return projected_x, projected_y

def find_nearest_waypoints(input_lat, input_lon, base_folder="Assistance_Astar/MAP/"):
    txt_files = glob.glob(f"{base_folder}/THUAN/*.txt") + glob.glob(f"{base_folder}/NGHICH/*.txt")
    
    nearest_file = None
    nearest_waypoint_1 = None
    nearest_waypoint_2 = None
    min_distance_1 = float("inf")
    min_distance_2 = float("inf")
    
    for file in txt_files:
        with open(file, "r") as f:
            waypoints = []
            for line in f:
                line = line.strip()
                if not line:  # Bỏ qua dòng trống
                    continue
                try:
                    lat, lon = map(float, line.split(","))
                    waypoints.append((lat, lon))
                except ValueError:
                    print(f"⚠️ Bỏ qua dòng không hợp lệ trong {file}: {line}")
        
        # Tìm 2 waypoint gần nhất
        for i in range(len(waypoints) - 1):
            lat1, lon1 = waypoints[i]
            lat2, lon2 = waypoints[i + 1]

            d1 = haversine(input_lat, input_lon, lat1, lon1)
            d2 = haversine(input_lat, input_lon, lat2, lon2)

            if d1 < min_distance_1:
                min_distance_2 = min_distance_1
                nearest_waypoint_2 = nearest_waypoint_1
                
                min_distance_1 = d1
                nearest_waypoint_1 = (lat1, lon1)
                nearest_file = os.path.basename(file)
            
            if d2 < min_distance_1:
                min_distance_2 = min_distance_1
                nearest_waypoint_2 = nearest_waypoint_1
                
                min_distance_1 = d2
                nearest_waypoint_1 = (lat2, lon2)
                nearest_file = os.path.basename(file)

    if nearest_waypoint_1 and nearest_waypoint_2:
        # Chiếu tọa độ hiện tại lên đoạn thẳng nối giữa 2 waypoint gần nhất
        projected_point = project_point_on_line((input_lat, input_lon), nearest_waypoint_1, nearest_waypoint_2)

        # Tính khoảng cách từ điểm chiếu đến tọa độ hiện tại
        projected_distance = haversine(input_lat, input_lon, projected_point[0], projected_point[1])

        # Chỉ trả về nếu khoảng cách < 1.5m
        if projected_distance < 2.0:
            return nearest_file, projected_point, projected_distance
        else:
            return None, None, None

    return None, None, None # Không tìm thấy waypoint phù hợp

def compare_coordinates(coord1, coord2, epsilon=1e-7):
    # print(coord1)
    """So sánh hai tọa độ với sai số nhỏ epsilon. Đảm bảo rằng tọa độ là kiểu float."""
    coord1 = tuple(map(float, coord1))  # Chuyển đổi tọa độ thành kiểu float
    coord2 = tuple(map(float, coord2))  # Chuyển đổi tọa độ thành kiểu float
    return (math.isclose(coord1[0], coord2[0], abs_tol=epsilon) and
            math.isclose(coord1[1], coord2[1], abs_tol=epsilon))

def load_waypoints(file_name):
    waypoints = []
    with open(file_name, 'r') as f:
        for line in f:
            # Tách chuỗi, bỏ khoảng trắng, sau đó chuyển thành tuple (float, float)
            coords = line.strip().split(',')
            if len(coords) == 2:
                waypoints.append(tuple(map(float, coords)))
    return waypoints

def compare_coordinates(coord1, coord2, epsilon=1e-6):
    """So sánh hai tọa độ với sai số nhỏ epsilon. Đảm bảo rằng tọa độ là kiểu float."""
    return (math.isclose(coord1[0], coord2[0], abs_tol= epsilon) and
            math.isclose(coord1[1], coord2[1], abs_tol= epsilon))

def find_optimal_path(current_position, target_node, waypoint_files_1, waypoint_files_1_TT, waypoint_files_2, waypoint_files_2_TT, waypoint_fil = 0):
    # Tìm waypoint gần nhất
    # file_name, waypoint1, waypoint2, projected_point, projected_distance = find_nearest_waypoints(input_lat, input_lon)
    name, nearest_point, distance = find_nearest_waypoints(current_position[0], current_position[1])
    print(f"name: {name}, distance: {distance}, nearest point: {nearest_point}")

    graph = Graph()
    graph.update_graph_with_projection(current_position)

    # Tìm start_node
    projected_nodes = list(set(graph.coordinates.keys()) - set(graph.graph.keys()))
    if projected_nodes:
        start_node = projected_nodes[0]  # Dùng điểm chiếu nếu có
    else:
        closest_node = min(
            graph.coordinates.keys(),
            key=lambda node: graph.haversine_distance(current_position, graph.coordinates[node])
        )
        start_node = closest_node

    # Tìm đường đi tối ưu
    optimal_path, dir = astar(graph, start_node, target_node)
    print(f"Optimal Path: {optimal_path}, Dir: {dir}")
    # Danh sách chứa toàn bộ waypoint từ các file
    all_waypoints = []

    
    if dir:
        if name == "waypoints_10_trungtam.txt":
            waypoint_fil  =  waypoint_files_1_TT
        else:
            waypoint_fil = waypoint_files_1
    else:
        if name == "waypoints_10_trungtam.txt"  or ('B', 'J') in optimal_path:
            waypoint_fil  =  waypoint_files_2_TT
        
        else:
            waypoint_fil = waypoint_files_2

    print("waypoint fil: ",waypoint_fil)
    print("\n### Loading Waypoints ###")
    for (from_node, to_node) in optimal_path:
        print(f"from node: {from_node}, to node: {to_node}")
        file_name = waypoint_fil.get((from_node, to_node))
        print(f"name: {name}")
        print(f"file_name: {file_name}")
        if file_name:
            print(f"📥 Loading waypoints from {file_name} for segment: {from_node} -> {to_node}")

            # Đọc dữ liệu từ file
            waypoints = load_waypoints(file_name)
            print(len(waypoints))
            try:
                file_basename = os.path.basename(file_name) if file_name else "None"
                name_basename = os.path.basename(name) if name else "None"
                print(file_basename, name_basename)
            except Exception as e:
                print(f"⚠️ Error printing basenames: {e}")
                
            if os.path.basename(file_name) == os.path.basename(name):
                # Nếu là file chứa nearest_point, chỉ lấy các waypoint phía sau nó
                try:
                    print("hihi")
                    # Tìm nearest_point trong file waypoint với sai số nhỏ
                    index = next(i for i, wp in enumerate(waypoints) if compare_coordinates(wp, nearest_point))
                    print(f"index: {index}")
                    waypoints = waypoints[index:]  # Lấy từ nearest_point trở đi
                    print(f"processed len: {len(waypoints)}")
                except StopIteration:
                    print(f"⚠️ Nearest point {nearest_point} not found in {file_name}")
            # Lưu vào danh sách tổng hợp
            all_waypoints.extend(waypoints)
        else:
            print(f"⚠️ No waypoint file configured for {from_node} -> {to_node}")

    # Ghi toàn bộ dữ liệu vào file duy nhất
    merged_file = "Assistance_Astar/merged_waypoints.txt"
    with open(merged_file, 'w') as file:
        file.writelines([f"{wp[0]},{wp[1]}\n" for wp in all_waypoints])
    print(f"\n✅ All waypoints merged into {merged_file}")

    return optimal_path

# Loại bỏ các waypoint trùng nhau liên tiếp
def remove_duplicate_waypoints(file_path):
    with open(file_path, 'r') as file:
        waypoints = file.readlines()

    # Xóa các waypoint trùng nhau liên tiếp
    unique_waypoints = [waypoints[0]]  # Giữ waypoint đầu tiên
    for i in range(1, len(waypoints)):
        if waypoints[i] != waypoints[i - 1]:  # Chỉ thêm nếu khác waypoint trước đó
            unique_waypoints.append(waypoints[i])

    # Ghi lại file sau khi loại bỏ trùng lặp
    with open(file_path, 'w') as file:
        file.writelines(unique_waypoints)

    # print(f"\n✅ Duplicate consecutive waypoints removed from {file_path}")

def run_map(name):
    # Example usage 
    simulation = 0
    if simulation != 1:
        lat, lon, _,_,car_heading, sat_count = get_gps_data(gps_ser)
        
        while True:
            lat, lon,_,_, car_heading, sat_count = get_gps_data(gps_ser)
            try:
                lat = float(lat)  
                lon = float(lon)

                if not math.isnan(lat):  
                    break

            except ValueError:
                pass  # Nếu không thể chuyển đổi, tiếp tục lấy dữ liệu GPS

        current_position = (lat, lon)
        # current_position = (10.8532734317,106.7715069650)
    else:
        current_position = (10.8507759433,106.7715801633)

    finder = LocationFinder()
    target_node = finder.get_key(name)
    path = find_optimal_path(current_position, target_node,waypoint_files_1, waypoint_files_1_TT, waypoint_files_2, waypoint_files_2_TT)
    remove_duplicate_waypoints("Assistance_Astar/merged_waypoints.txt")

