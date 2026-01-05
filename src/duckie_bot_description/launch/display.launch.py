from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('duckie_bot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'duckie_bot.urdf.xacro')

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    return LaunchDescription([

        # Publishes joint states (REQUIRED for wheels)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # Publishes TF from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2'
        )
    ])

