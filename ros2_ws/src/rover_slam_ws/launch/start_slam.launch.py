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
                        'use_sim_time': False,
                        'odom_frame':   'odom',
                        'base_frame':   'base_link',
                        'map_frame':    'map',
                        'scan_topic':   '/scan',
                        
                        # Mise à jour du topic de l'odométrie
                        'odom_topic':   '/odom',
                        'mode':         'mapping',

                        'resolution':              0.05,
                        'map_update_interval': 0.5,       # Met à jour la carte plus souvent
                        'max_laser_range': 8.0,           # Ne pas prendre les points trop loin (souvent bruités)
                        'minimum_time_interval': 0.1,     # Accepte des scans plus rapidement
                        'transform_publish_period': 0.02, # Publie la TF très vite pour éviter le lag

                        'coarse_search_angle_offset':          0.349,
                        'fine_search_angle_offset':            0.00349,
                        'correlation_search_space_dimension':  0.3,
                        'correlation_search_space_resolution': 0.01,

                        # ==================== LOOP CLOSURE ====================
                        # C'est ÇA qui permet de fermer le circuit du couloir
                        'do_loop_closing': True,
                        'loop_search_maximum_distance':        2.0, # Cherche à raccorder si on est à moins de 4m d'un lieu connu
                        'loop_match_minimum_response_fine':    0.8, # Un peu baissé pour forcer l'accroche
                        'loop_match_minimum_response_coarse':  0.8,

                        'transform_timeout':  0.5,
                        'tf_buffer_duration': 10.0,
                    }]
                ),
            ]
        ),
    ])