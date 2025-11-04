from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directory of the package (optional, if needed)
    # package_dir = get_package_share_directory('obstacle_cloud_to_scan')

    return LaunchDescription([
    
        Node(
            package='obstacle_cloud_to_scan',
            executable='obstacle_cloud_to_scan',
            name='obstacle_cloud_to_scan_node',
            # arguments=['--ros-args', '--log-level', 'debug'],
            output='screen',
            parameters=[{
                'target_frame': 'base_link',
                'input_topic': '/livox/lidar',
                'output_topic': '/cloud_in', # pointcloud_to_laserscanに入力
                'ground_remove_algorithm': 'PMF',
                'voxel_leaf_size': 0.1,
                'robot_box_size': [0.8, 0.5, 0.6],
                'robot_box_position': [0.0, 0.0, 0.0],
                
                # 障害物検知範囲パラメータ（X, Y, Z パススルーフィルタ）
                'obstacle_detection_range_x_min': 0.4,
                'obstacle_detection_range_x_max': 6.0,
                'obstacle_detection_range_y_min': -6.0,
                'obstacle_detection_range_y_max': 6.0,
                'obstacle_detection_range_z_min': -1.0,
                'obstacle_detection_range_z_max': 1.0,  # Default: robot_box_size[2] + 0.3
                'normal_max_slope_angle': 25.0,
                'pmf_max_window_size': 33,
                'pmf_slope': 1.0,
                'pmf_initial_distance': 0.15,
                'pmf_max_distance': 3.0,
                'pmf_cell_size': 0.5,

                # 穴検知パラメータ
                'hole_detection_enabled': True,
                'hole_detection_algorithm': 'BASIC',
                'hole_output_topic': '/hole_cloud/cloud',
                'lidar_frame': 'livox_frame',
                'hole_detection_range_x': 3.0,
                'hole_detection_range_y': 5.0,
                'hole_detection_max_height': 0.3,
                'hole_ground_tolerance': 0.1,

                # LiDAR原点パラメータ（target_frame座標系での位置）
                'lidar_origin_x': 0.32,
                'lidar_origin_y': 0.0,
                'lidar_origin_z': 0.116,

                # 動的地面平面推定パラメータ
                'use_dynamic_ground_plane': True,
                'ground_plane_rolling_window_x': 4.0,
                'ground_plane_rolling_window_y': 6.0,
                'ground_plane_ransac_distance_threshold': 0.05,
                'ground_plane_ransac_max_iterations': 100,
                'hole_detection_height_buffer': 0.1,

                # 地面平面可視化パラメータ
                'visualize_ground_plane': True,
                'ground_plane_visualization_topic': '/ground_plane_marker',
                'ground_plane_visualization_size': 5.0
            }],
        ),
        
        Node(
            package='pointcloud_to_laserscan', 
            executable='pointcloud_to_laserscan_node',
            # arguments=['--ros-args', '--log-level', 'debug'],
            name='pointcloud_to_laserscan_node',
            output='screen',
            remappings=[
                ('cloud_in', '/cloud_in'),
                ('scan', '/scan')  # 通常のトピック名に合わせる
            ],
            parameters=[{
                'target_frame': 'base_link',  # 空文字列から修正
                'transform_tolerance': 0.01,
                'min_height': -1.0,
                'max_height': 2.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,   # M_PI/2
                'angle_increment': 0.0174,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 40.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }]
        )
    ])
