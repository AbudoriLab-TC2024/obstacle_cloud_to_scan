from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directory of the package
    package_dir = get_package_share_directory('obstacle_cloud_to_scan')

    return LaunchDescription([
        Node(
            package='obstacle_cloud_to_scan',
            executable='obstacle_cloud_to_scan',
            name='obstacle_cloud_to_scan_node',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'debug'],
            parameters=[{
                'target_frame': 'base_link',
                'input_topic': '/livox/lidar',
                'output_topic': '/obstacle_cloud/cloud',
                'laser_scan_topic': '/scan',
                'ground_remove_algorithm': 'NORMAL',
                'voxel_leaf_size': 0.1,
                'robot_box_size': [0.9, 0.8, 1.0],
                'robot_box_position': [0.0, 0.0, 0.0],
                'normal_max_slope_angle': 20.0,
                'pmf_max_window_size': 5,
                'pmf_slope': 1.0,
                'pmf_initial_distance': 0.05,
                'pmf_max_distance': 1.0,
                'pmf_cell_size': 0.25,
                # Hole detection parameters (disabled by default)
                'hole_detection_enabled': True,
                'hole_detection_algorithm': 'BASIC',
                'hole_output_topic': '/hole_cloud/cloud',
                'lidar_frame': 'livox_frame',
                'hole_detection_range_x': 3.0,
                'hole_detection_range_y': 5.0,
                'hole_detection_max_height': 0.3,
                'hole_ground_tolerance': 0.05
            }],
        )
    ])

