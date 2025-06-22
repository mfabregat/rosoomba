from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments

    declared_arguments = [
        DeclareLaunchArgument(
            name = 'config',
            default_value = PathJoinSubstitution([
                FindPackageShare('create_bringup'),
                'config',
                'default.yaml'
            ]),
            description='Path to the configuration file'
        ),DeclareLaunchArgument(
            'desc',
            default_value='true',
            description='Whether to include robot description'
        ),
    ]
    
    # Robot description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('create_description'),
                'launch',
                'create_2.launch'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('desc'))
    )
    
    return LaunchDescription(
        declared_arguments + [
        robot_description_launch,
    ])
