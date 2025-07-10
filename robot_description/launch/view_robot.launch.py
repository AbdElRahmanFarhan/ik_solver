import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('robot_description')
    
    urdf_file = os.path.join(package_share_directory, 'urdf', 'combined.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            output='screen',
        ),  
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_robot',
            output='screen',
            # arguments=['-d', os.path.join(package_share_directory, 'config', 'robot.rviz')]
        ),
    ])