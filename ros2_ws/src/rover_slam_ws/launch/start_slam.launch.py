from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([

        # TFs statiques (laser -> base_link, etc.)
        Node(
            package='my_rover_slam',
            executable='static_tf_publisher',
            name='static_tf_publisher',
            output='screen',
        ),
        
        # NOUVEAU : Odométrie basée sur les commandes moteurs (PWM)
        Node(
            package='my_rover_slam',
            executable='wheel_odometry_node', # <- Mets bien le nom de ton nouveau script compilé ici
            name='wheel_odometry',
            output='screen',
        ),

        Node(
            package='my_rover_slam',
            executable='lap_counter',
            name='lap_counter',
            output='screen'
        ),

        # Attend 2s que les TFs et l'Odométrie soient bien publiés
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[{
                        # Plugin params
                        'solver_plugin': 'solver_plugins::CeresSolver',
                        'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
                        'ceres_preconditioner': 'SCHUR_JACOBI',
                        'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
                        'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
                        'ceres_loss_function': 'None',

                        # ROS Parameters
                        'odom_frame': 'laser_link',
                        'map_frame': 'map',
                        'base_frame': 'laser_link',
                        'scan_topic': '/scan',
                        'use_map_saver': True,
                        'mode': 'mapping',

                        'map_update_interval': 2.0,
                        'transform_publish_period': 0.05,
                        'resolution': 0.05,
                        'max_laser_range': 12.0,
                        'minimum_time_interval': 0.3,
                        'transform_timeout': 0.2,
                        'tf_buffer_duration': 30.0,

                        # General Parameters
                        'use_scan_matching': True,
                        'use_scan_barycenter': True,
                        'minimum_travel_distance': 0.0,
                        'minimum_travel_heading': 0.0,
                        'scan_buffer_size': 30,
                        'scan_buffer_maximum_scan_distance': 10.0,
                        'link_match_minimum_response_fine': 0.5,
                        'do_loop_closing': False,

                        # Correlation Parameters
                        'correlation_search_space_dimension':       0.2,
                        'correlation_search_space_resolution':      0.01,
                        'correlation_search_space_smear_deviation': 0.03,

                        # Loop Closure (désactivé mais paramètres conservés)
                        'loop_search_maximum_distance':         1.5,
                        'loop_match_minimum_response_coarse':   0.7,
                        'loop_match_minimum_response_fine':     0.8,
                    }]
                )
            ]
        ),
    ])