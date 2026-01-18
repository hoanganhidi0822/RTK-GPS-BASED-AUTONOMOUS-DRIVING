from setuptools import find_packages, setup
import shutil

package_name = 'visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='baotrinh20052006@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "visualization = visualization.main:main"
        ],
    },
)

shutil.copytree(f"/home/dev/ros2_ws/src/vehicle/{package_name}/assets",
                f"/home/dev/ros2_ws/install/{package_name}/share/{package_name}/assets",
                dirs_exist_ok=True)

