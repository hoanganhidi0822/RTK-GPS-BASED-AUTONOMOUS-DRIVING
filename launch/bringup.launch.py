from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomous_car',
            executable='visualization', 
            name='visualization',
            output='screen',
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
            executable='map_planer_node', 
            name='map_planer_node',
            output='screen',
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
