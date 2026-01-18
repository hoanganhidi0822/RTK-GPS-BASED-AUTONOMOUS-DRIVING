import os
import math
import heapq
from collections import defaultdict
import csv
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from pyproj import Transformer
import matplotlib.pyplot as plt
import contextily as ctx

PKG_ROOT = Path(__file__).resolve().parents[2]
ROOT_DIR = os.environ.get("MAP_ROOT_DIR", str(PKG_ROOT / "assets" / "MAP"))
path_dir = os.environ.get("WAYPOINT_DIR", str(PKG_ROOT / "way"))

# ====== CHECKPOINTS (lat, lon) ======
CHECKPOINTS = {
    "TTT":  (10.8507759083, 106.7715805667), # Toà Trung Tâm trước
    "C":    (10.8532733433, 106.7715069217), # Khu C
    "D":    (10.852302,     106.771424),     # Khu D
    "B":    (10.8514838933, 106.7713101400),
    "STT":  (10.8512819350, 106.7719588833), # Toà Trung Tâm
    "VD":   (10.851238,     106.772669),     # Toà Việt Đức
    "CC":   (10.851554,     106.772746),
    "MS":   (10.851641,     106.773369),     # Maker Space
    "SG":   (10.852364,     106.772835),
    "G":    (10.853240,     106.772932),     # Go
    "H":    (10.853319,     106.772592),
    "I":    (10.853541,     106.772572),
    "K":    (10.853686,     106.771636),
}


ORDER_THUAN = [
    "C", "D", "TTT", "STT", "VD", "CC", "MS", "E", "SG", "G", "H", "I", "K", "B"
]

transformer = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)

def load_waypoints(path):
    pts = []
    with open(path, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or len(row) < 2:
                continue
            try:
                lat = float(row[0])
                lon = float(row[1])
            except ValueError:
                continue
            pts.append((lat, lon))

    print(f"Đã đọc {len(pts)} waypoint từ CSV: {path}")
    return pts

def get_waypoint_files(folder):
    """
    Lấy các file waypoint dạng:
      NGHICH_*.csv
      THUAN_*.csv
      và kết thúc bằng _densified.csv
    (tên file viết HOA)
    """
    return sorted([
        os.path.join(folder, f)
        for f in os.listdir(folder)
        if (
            f.startswith("NGHICH") or f.startswith("THUAN")
        ) and f.endswith("_densified.csv")
    ])


def plot_group(folder, color, group_label, ax):
    """
    Vẽ toàn bộ waypoint của 1 nhóm (NGHICH/THUAN).
    Đường vẽ ra từ CHÍNH những waypoint (không nội suy).
    """
    files = get_waypoint_files(folder)

    first_label = True
    all_x = []
    all_y = []

    for path in files:
        pts = load_waypoints(path)
        if not pts:
            continue

        lats = [p[0] for p in pts]
        lons = [p[1] for p in pts]

        xs, ys = transformer.transform(lons, lats)

        all_x.extend(xs)
        all_y.extend(ys)

        if first_label:
            ax.plot(
                xs, ys,
                linewidth=2.0,
                color=color, alpha=0.95,
                label=group_label,
                zorder=20,
            )
            first_label = False
        else:
            ax.plot(
                xs, ys,
                linewidth=2.0,
                color=color, alpha=0.95,
                zorder=20,
            )

    return all_x, all_y


# ========= XÂY ĐỒ THỊ TỪ WAYPOINT & A* =========

def build_graph_from_file(csv_path, join_thresh_m=5.0):
    pts = load_waypoints(csv_path)
    if len(pts) < 2:
        print("⚠️ CSV quá ít điểm, không đủ để tạo graph.")
        return [], defaultdict(list)

    nodes = []              # (lat, lon, x, y)
    edges = defaultdict(list)

    lats = [p[0] for p in pts]
    lons = [p[1] for p in pts]
    xs, ys = transformer.transform(lons, lats)

    for lat, lon, x, y in zip(lats, lons, xs, ys):
        nodes.append((lat, lon, x, y))

    N = len(nodes)
    for i in range(N):
        xi, yi = nodes[i][2], nodes[i][3]
        for j in range(i + 1, N):
            xj, yj = nodes[j][2], nodes[j][3]
            d = math.hypot(xj - xi, yj - yi)
            if d <= join_thresh_m:
                edges[i].append((j, d))
                edges[j].append((i, d))

    print(f"Tổng node: {N}, tổng số cạnh (ước lượng): {sum(len(v) for v in edges.values()) // 2}")
    return nodes, edges




def find_nearest_node(nodes, lat_target, lon_target):
    """
    Tìm node (waypoint) gần nhất với toạ độ (lat,lon) bất kỳ.
    """
    x_t, y_t = transformer.transform(lon_target, lat_target)
    best_idx = None
    best_d2 = float("inf")
    for idx, (_, _, x, y) in enumerate(nodes):
        d2 = (x - x_t) ** 2 + (y - y_t) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_idx = idx
    return best_idx


# ========= CHỌN THUẬN / NGHỊCH THEO CHECKPOINT =========

def nearest_checkpoint_name(lat_cur, lon_cur):
    """
    Tìm checkpoint gần nhất với (lat_cur, lon_cur).
    """
    x_cur, y_cur = transformer.transform(lon_cur, lat_cur)
    best_name = None
    best_d2 = float("inf")

    for name, (lat_cp, lon_cp) in CHECKPOINTS.items():
        x_cp, y_cp = transformer.transform(lon_cp, lat_cp)
        d2 = (x_cp - x_cur) ** 2 + (y_cp - y_cur) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_name = name

    return best_name


def choose_group_by_order(start_cp_name, dest_cp_name):
    """
    Rule: nếu index(start) < index(dest) trong ORDER_THUAN -> THUAN, else -> NGHICH.
    """
    if start_cp_name not in ORDER_THUAN or dest_cp_name not in ORDER_THUAN:
        print("⚠️ start/dest không có trong ORDER_THUAN, mặc định THUAN")
        return "THUAN"

    i_start = ORDER_THUAN.index(start_cp_name)
    i_dest = ORDER_THUAN.index(dest_cp_name)
    print(f"Vị trí trong ORDER_THUAN: start={start_cp_name} ({i_start}), dest={dest_cp_name} ({i_dest})")

    if i_start < i_dest:
        return "THUAN"
    else:
        return "NGHICH"


# ========= A* =========

def astar(nodes, edges, start_idx, goal_idx):
    """
    Thuật toán A* trên graph waypoint.
    """
    goal_x, goal_y = nodes[goal_idx][2], nodes[goal_idx][3]

    def h(n):
        x, y = nodes[n][2], nodes[n][3]
        return math.hypot(x - goal_x, y - goal_y)

    open_heap = []  # (f, g, node)
    heapq.heappush(open_heap, (h(start_idx), 0.0, start_idx))

    g_score = {start_idx: 0.0}
    parent = {start_idx: None}
    visited = set()

    while open_heap:
        f_cur, g_cur, u = heapq.heappop(open_heap)

        if u in visited:
            continue
        visited.add(u)

        if u == goal_idx:
            break

        for v, w in edges.get(u, []):
            tentative_g = g_cur + w
            if v not in g_score or tentative_g < g_score[v]:
                g_score[v] = tentative_g
                parent[v] = u
                f_v = tentative_g + h(v)
                heapq.heappush(open_heap, (f_v, tentative_g, v))

    if goal_idx not in parent and start_idx != goal_idx:
        print("⚠️  Không tìm được đường đi giữa 2 điểm trên graph waypoint.")
        return []

    # reconstruct path
    path = [goal_idx]
    cur = goal_idx
    while parent[cur] is not None:
        cur = parent[cur]
        path.append(cur)
    path.reverse()
    return path


def save_waypoints_segment_from_nodes(nodes, indices, out_path):
    """
    Lưu đoạn đường (theo index trong 'nodes') thành file waypoint txt: 'lat,lon' mỗi dòng.
    """
    with open(out_path, "w") as f:
        for i in indices:
            lat, lon = nodes[i][0], nodes[i][1]
            f.write(f"{lat:.10f},{lon:.10f}\n")
    print(f"Đã lưu {len(indices)} waypoint vào: {out_path}")

def plot_route_debug(nodes, route_indices,
                     lat_start, lon_start,
                     destination, dest_cp,
                     chosen_group):
    """
    Vẽ bản đồ debug bằng matplotlib:
    - tất cả waypoint trong group (THUAN / NGHICH)
    - các checkpoint
    - route A*
    - điểm start & dest.
    """
    # Toạ độ xy của tất cả node
    all_x = [n[2] for n in nodes]
    all_y = [n[3] for n in nodes]

    # Checkpoint
    cp_x, cp_y = [], []
    for name, (lat_cp, lon_cp) in CHECKPOINTS.items():
        x_cp, y_cp = transformer.transform(lon_cp, lat_cp)
        cp_x.append(x_cp)
        cp_y.append(y_cp)

    # Route A*
    route_x = [nodes[i][2] for i in route_indices]
    route_y = [nodes[i][3] for i in route_indices]

    # Start + Dest
    x_start, y_start = transformer.transform(lon_start, lat_start)
    lat_dest, lon_dest = destination
    x_dest, y_dest = transformer.transform(lon_dest, lat_dest)

    fig, ax = plt.subplots(figsize=(8, 8))

    # Tất cả waypoint
    ax.scatter(all_x, all_y, s=5, color="lightgray",
               label=f"Waypoints ({chosen_group})", zorder=5)

    # Route A*
    ax.plot(route_x, route_y, color="red", linewidth=2.5,
            label="A* route", zorder=20)

    # Checkpoint
    ax.scatter(cp_x, cp_y, s=40, color="yellow",
               edgecolors="black", zorder=30)
    for (name, (lat_cp, lon_cp)), x_cp, y_cp in zip(CHECKPOINTS.items(), cp_x, cp_y):
        ax.text(
            x_cp, y_cp, name,
            fontsize=8, weight="bold", ha="center", va="center",
            color="black", zorder=31,
            bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="black",
                      alpha=0.8, linewidth=0.4)
        )

    # Start & Dest
    ax.scatter([x_start], [y_start], s=60, color="green",
               label="Start", zorder=40)
    ax.scatter([x_dest], [y_dest], s=60, color="blue",
               label=f"Dest ({dest_cp})", zorder=40)

    # ===== Set khung hình & nền bản đồ =====
    all_x_total = all_x + cp_x + [x_start, x_dest]
    all_y_total = all_y + cp_y + [y_start, y_dest]

    if all_x_total and all_y_total:
        margin_x = (max(all_x_total) - min(all_x_total)) * 0.05
        margin_y = (max(all_y_total) - min(all_y_total)) * 0.05
        ax.set_xlim(min(all_x_total) - margin_x, max(all_x_total) + margin_x)
        ax.set_ylim(min(all_y_total) - margin_y, max(all_y_total) + margin_y)

    ax.set_aspect("equal", adjustable="box")


    try:
        ctx.add_basemap(
            ax,
            source=ctx.providers.OpenStreetMap.Mapnik,
            zorder=0
        )
    except Exception as e:
        print(f"[WARN] Không load được basemap OSM: {e}")

    ax.set_xlabel("X (WebMercator, m)")
    ax.set_ylabel("Y (WebMercator, m)")
    ax.set_title(
        f"MAP_NT + Shortest route from GPS start (A*) → {dest_cp} ({chosen_group})"
    )
    ax.grid(False)
    ax.legend()
    plt.tight_layout()
    plt.show()
