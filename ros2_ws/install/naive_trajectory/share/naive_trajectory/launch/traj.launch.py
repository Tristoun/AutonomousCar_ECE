import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([       
        Node(
            package='naive_trajectory',
            executable='naive_traj_lidar', # On enlève le .py et on met le nom exact du setup.py
            name='algo_naif'
        ),

        Node(
            package='naive_trajectory',
            executable='planner',          # Doit correspondre à l'entrée setup.py
            name='planner_astar'
        ),

        Node(
            package='naive_trajectory',
            executable='pure_pursuit',     # Idem
            name='pure_pursuit'
        ),
    ])