from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_yaml_file = LaunchConfiguration('params')

    async_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']),
        ),
        launch_arguments = {'use_sim_time': use_sim_time,
                            'slam_params_file': params_yaml_file}.items()
    )

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
        'params',
        default_value=PathJoinSubstitution(
            [FindPackageShare('hibachi_navigation'),
             'config',
             'mapper_params_online_async.yaml']),
        description='Full path to param yaml file to load')
    ])
    ld.add_action(async_slam_toolbox)
    return ld
