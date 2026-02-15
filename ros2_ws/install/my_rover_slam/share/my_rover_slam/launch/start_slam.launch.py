import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('my_rover_slam')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    return LaunchDescription([
        # 1. Foxglove Bridge
        # Node(
            # package='foxglove_bridge',
            # executable='foxglove_bridge',
            # parameters=[{'port': 8765, 'address': '0.0.0.0'}]
        # ),

        # 2. Static Transforms (Exactly 8 arguments)
        # Change '--yaw', '0' par '--yaw', '3.14159'
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.1', '--y', '0', '--z', '0.05', 
                    '--yaw', '3.14159', '--pitch', '0', '--roll', '0', 
                    '--frame-id', 'base_link', '--child-frame-id', 'laser_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.0', '--y', '0', '--z', '0.1', 
                    '--yaw', '0', '--pitch', '0', '--roll', '0', 
                    '--frame-id', 'base_link', '--child-frame-id', 'imu_link']
        ),

        # 3. PointCloud -> LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'laser_link', 
                'transform_tolerance': 0.1,
                'min_height': -1.0,
                'max_height': 1.0,
                'angle_min': -3.1415,
                'angle_max': 3.1415,
                'angle_increment': 0.01745,
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 12.0,
                'use_inf': True  # CRITICAL: Use True so values > 12m are handled correctly
            }],
            remappings=[('cloud_in', '/point_cloud'), ('scan', '/scan')]
        ),

        # 4. RF2O Laser Odometry (Debug Mode: TF Enabled)
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_odometry',
            parameters=[{
                'laser_scan_topic': '/scan_fixed',
                'odom_topic': '/odom_rf2o',
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'sensor_frame_id': 'laser_link',
                'publish_tf': True,
                'use_best_effort_qos': True,
                # --- ADD THESE THREE LINES ---
                'tf_timeout': 0.5,           # Increase time allowed to find TF
                'freq': 10.0,               # Match this close to your scan rate (6.5Hz)
                'init_pose_from_topic': '',  # Ensure it doesn't wait for a manual trigger
            }]
        ),

        Node(
            package='my_rover_slam', # Or wherever you put the script
            executable='scan_time_fixer',
            name='scan_time_fixer'
        ),

        # 5. Robot Localization (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

        # 6. SLAM Toolbox (Asynchronous Mapping)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame': 'map',
                'scan_topic': '/scan_fixed'
            }]
        )

        
        # 5. EKF and SLAM are commented out for now. 
        # Get /odom_rf2o working first!
    ])