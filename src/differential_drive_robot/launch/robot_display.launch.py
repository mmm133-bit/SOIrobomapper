from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    pkg_share = FindPackageShare(package='differential_drive_robot').find('differential_drive_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', '/home/ubu/3ros2_ws/src/differential_drive_robot/urdf/robot.urdf.xacro'])}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    diff_drive_controller_node = Node(
        package='differential_drive_robot',
        executable='diff_drive_controller',
        name='diff_drive_controller',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        diff_drive_controller_node
    ])