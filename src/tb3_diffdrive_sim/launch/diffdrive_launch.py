from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions import FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = FindPackageShare('tb3_diffdrive_sim')

    # Correct way to pass multiple arguments with a space in between
    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'),
            ' ',
            PathJoinSubstitution([pkg_share, 'urdf', 'tb3_with_control.urdf.xacro'])
        ])
    }

    controller_config = os.path.join(
        get_package_share_directory('tb3_diffdrive_sim'),
        'config',
        'tb3_controller.yaml'
    )

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controller_config],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager-timeout', '50'],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
    ])
