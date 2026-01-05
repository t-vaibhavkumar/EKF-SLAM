from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('duckie_bot_description')
    
    # 1. Path to the Xacro file
    xacro_file = os.path.join(pkg_path, 'urdf', 'duckie_bot.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # 2. Path to the World file
    # We look inside the 'worlds' folder of your package
    world_file = os.path.join(pkg_path, 'worlds', 'room.world')

    return LaunchDescription([

        # Start Gazebo with the custom world file
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'duckie_bot',
                '-topic', 'robot_description',
                '-x', '0.0', # You can adjust start position
                '-y', '0.0',
                '-z', '0.1'
            ],
            output='screen'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        )
    ])
