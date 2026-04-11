from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = LaunchConfiguration('rviz_config')
    urdf_file = LaunchConfiguration('urdf_file')

    robot_description = Command(['xacro ', urdf_file])

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('rm_debug_visualizer'),
                'rviz',
                'view_model.rviz',
            ]),
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rm_description'),
                'urdf',
                'gimbal_b3.urdf.xacro',
            ]),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
