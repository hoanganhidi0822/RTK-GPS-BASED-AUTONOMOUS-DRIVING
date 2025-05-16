import numpy as np
import matplotlib.pyplot as plt

def cubic_polynomial(d0, d0_d, d0_dd, df, sf):
    """
    Tạo phương trình bậc 3 cho chuyển động ngang.
    """
    a0 = d0
    a1 = d0_d
    a2 = (3 * (df - d0) - (2 * d0_d + 0) * sf) / (sf ** 2)
    a3 = (2 * (d0 - df) + (d0_d + 0) * sf) / (sf ** 3)
    return a0, a1, a2, a3

d_values = np.linspace(-2, 2, 10)  # Các giá trị ngang
s_f = 30  # Điểm cuối dọc
s = np.linspace(0, s_f, 100)  # Mẫu dọc

trajectories = []
end_points = []  # Lưu các điểm cuối

# Tạo và lưu trữ các quỹ đạo cùng điểm cuối
for d_f in d_values:
    a0, a1, a2, a3 = cubic_polynomial(0, 0, 0, d_f, s_f)
    d = a0 + a1 * s + a2 * s**2 + a3 * s**3
    trajectories.append(d)
    end_points.append((s_f, d_f))  # Lưu điểm cuối (s_f, d_f)

# Hiển thị quỹ đạo và các điểm cuối
plt.figure(figsize=(10, 6))

# Vẽ các quỹ đạo
for i, d in enumerate(trajectories):
    plt.plot(s, d, label=f"Trajectory {i+1} (d_f={d_values[i]:.2f})")

# Vẽ các điểm cuối
end_s, end_d = zip(*end_points)
plt.scatter(end_s, end_d, color="red", label="End Points", zorder=5)

plt.xlabel("s (Longitudinal)")
plt.ylabel("d (Lateral)")
plt.title("Candidate Trajectories and End Points in Frenet Coordinate")
plt.legend()
plt.grid(True)
plt.show()
