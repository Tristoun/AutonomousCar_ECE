from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Paths to packages
    robot_description_pkg = get_package_share_directory('my_robot_description')
    trajectory_pkg = get_package_share_directory('naive_trajectory')
    slam_pkg = get_package_share_directory('my_rover_slam')

    # Include launch files
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_pkg, 'launch', 'robot_description.launch.py')
        )
    )

    trajectory_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(trajectory_pkg, 'launch', 'traj.launch.py')
        )
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'start_slam.launch.py')
        )
    )

    return LaunchDescription([
        robot_description_launch,
        slam_launch,
        trajectory_launch,

    ])

