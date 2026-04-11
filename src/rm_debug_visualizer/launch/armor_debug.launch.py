from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_serial = LaunchConfiguration('use_serial')
    debug_mode = LaunchConfiguration('debug_mode')
    ros_bag_mode = LaunchConfiguration('ros_bag_mode')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_serial',
            default_value='True',
            description='Whether use serial port',
        ),
        DeclareLaunchArgument(
            'debug_mode',
            default_value='False',
        ),
        DeclareLaunchArgument(
            'ros_bag_mode',
            default_value='False',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('rm_debug_visualizer'),
                'rviz',
                'view.rviz',
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('bringup'),
                    'launch',
                    'armor_launch.py',
                ])
            ),
            launch_arguments={
                'use_serial': use_serial,
                'debug_mode': debug_mode,
                'ros_bag_mode': ros_bag_mode,
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
