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
        DeclareLaunchArgument('num_threads', default_value='1'),
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
                'robot_box_size': [0.3, 0.25, 0.3],
                'robot_box_position': [0.0, 0.0, 0.0],
                
                # Obstacle detection range parameters (X, Y, Z PassThrough filter)
                'obstacle_detection_range_x_min': -0.0,
                'obstacle_detection_range_x_max': 3.0,
                'obstacle_detection_range_y_min': -3.0,
                'obstacle_detection_range_y_max': 3.0,
                'obstacle_detection_range_z_min': -0.3,
                'obstacle_detection_range_z_max': 0.5,  
                'normal_max_slope_angle': 20.0,
                'pmf_max_window_size': 5,
                'pmf_slope': 1.0,
                'pmf_initial_distance': 0.05,
                'pmf_max_distance': 1.0,
                'pmf_cell_size': 0.25,
                
                # Phase 2: Hierarchical filtering parameters (disabled by default for optimal performance)
                'enable_hierarchical_filtering': False,  # Set to True for large-scale environments or safety-critical applications
                'collision_distance_threshold': 3.0,    # Distance threshold for near/far classification (meters)
                'far_zone_voxel_multiplier': 2.0,      # Voxel size multiplier for far zone (2.0 = 2x coarser)
                
                # Parallelization parameters (single thread by default for optimal performance)
                'num_threads': LaunchConfiguration('num_threads'),  # 1=single thread, >1=parallel processing (e.g., 2-4 for multi-core)
                'parallel_threshold_points': 100,     # Minimum points to enable parallelization (avoid overhead for small clouds)
                
                # Hole detection parameters (enabled by default in this launch)
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

