import numpy as np
import matplotlib.pyplot as plt

# Các điểm s
s_f = 30  # Điểm cuối dọc
s = np.linspace(0, s_f, 100)  # Mẫu dọc

# Vẽ các điểm trên trục s
plt.figure(figsize=(8, 4))
plt.plot(s, np.zeros_like(s), 'bo', label="Các điểm trên trục s")
plt.axhline(0, color='gray', linestyle='--', linewidth=0.5)
plt.title("Các điểm dọc trên trục Frenet (trục s)")
plt.xlabel("s (khoảng cách dọc)")
plt.ylabel("Trục ngang (luôn là 0 ở đây)")
plt.legend()
plt.grid()
plt.show()
