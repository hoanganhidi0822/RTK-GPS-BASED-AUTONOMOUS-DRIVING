from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    origin_lat = LaunchConfiguration("origin_lat")
    origin_lon = LaunchConfiguration("origin_lon")

    return LaunchDescription([
        DeclareLaunchArgument("origin_lat", default_value="10.8532570333"),
        DeclareLaunchArgument("origin_lon", default_value="106.7715131967"),
        SetEnvironmentVariable("ORIGIN_LAT", origin_lat),
        SetEnvironmentVariable("ORIGIN_LON", origin_lon),
        Node(
            package='autonomous_car',
            executable='visualization', 
            name='visualization',
            output='screen',
            parameters=[{"origin_lat": origin_lat, "origin_lon": origin_lon}],
        ),
        Node(
            package='autonomous_car',
            executable='perception_node', 
            name='perception_node',
            output='screen',
        ),
        Node(
            package='autonomous_car',
            executable='GPS_node', 
            name='gps_node',
            output='screen',
        ),
        Node(
            package='autonomous_car',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[{"origin_lat": origin_lat, "origin_lon": origin_lon}],
        ),
        Node(
            package='autonomous_car',
            executable='map_planer_node', 
            name='map_planer_node',
            output='screen',
            parameters=[{"origin_lat": origin_lat, "origin_lon": origin_lon}],
        ),
        Node(
            package='autonomous_car',
            executable='Fot_node',
            name='Fot_node',
            output='screen',
        ),
        Node(
            package='autonomous_car',
            executable='control_node',
            name='Fot_node',
            output='screen',
        ),
    ])
