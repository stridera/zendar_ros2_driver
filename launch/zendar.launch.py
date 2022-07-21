from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='zendar_pointcloud_node',
            package='zendar_pointcloud_node',
            executable='zendar_pointcloud_node',
            output='screen',
            parameters=[{
                'url': '192.168.1.9',
                'max_range': 40.0,
            }]
        )
    ])
