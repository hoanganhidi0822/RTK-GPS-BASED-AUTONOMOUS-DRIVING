from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'autonomous_car'

# 1. Khai báo các file cơ bản
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]

# 2. Xử lý ASSETS: Copy đệ quy giữ nguyên cấu trúc thư mục
# Lưu ý: asset_base giờ phải bao gồm cả tên package_name vì đã di chuyển thư mục
asset_base = os.path.join(package_name, 'visualization', 'assets')
install_base = os.path.join('share', package_name, 'assets')

if os.path.exists(asset_base):
    for root, dirs, files in os.walk(asset_base):
        if not files:
            continue
            
        # Tính toán đường dẫn tương đối so với asset_base
        rel_path = os.path.relpath(root, asset_base)
        
        if rel_path == '.':
            dest_path = install_base
        else:
            dest_path = os.path.join(install_base, rel_path)
            
        file_paths = [os.path.join(root, f) for f in files]
        data_files.append((dest_path, file_paths))
else:
    print(f"WARNING: Không tìm thấy thư mục assets tại {asset_base}")

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minh_tan',
    maintainer_email='minh_tan@todo.todo',
    description='Autonomous car package for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Cấu trúc: tên_lệnh = package.folder.file:tên_hàm
            'visualization = autonomous_car.visualization.visualization.main:main',
            'perception_node = autonomous_car.perception_node.perception_node:main',
            'GPS_node = autonomous_car.GPS_node.GPS_node:main',
            'odometry_node = autonomous_car.odometry_node.odometry_node:main',
            'map_planer_node = autonomous_car.map_planer_node.astar_node:main',
            'Fot_node = autonomous_car.Fot_node.Fot_node:main',
            'control_node = autonomous_car.control_node.control_node:main',
            # Ví dụ thêm các node khác nếu cần:
            # 'camera = autonomous_car.camera_node.camera_node:main',
        ],
    },
)
