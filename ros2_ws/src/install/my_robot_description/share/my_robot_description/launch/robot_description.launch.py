from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the xacro file
    xacro_file = PathJoinSubstitution([
        get_package_share_directory('my_robot_description'),
        'urdf',
        'meshwaverobot.xacro'
    ])

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    # Foxglove bridge node
    foxglove = Node(
        name='foxglove_bridge',
        package='foxglove_bridge',
        executable='foxglove_bridge',
    )

    # micro-ROS agent process
    micro_ros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'udp4', '--port', '8888', '-v6'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        foxglove,
        micro_ros_agent
    ])
