from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.substitutions import FindPackageShare


from launch.actions import LogInfo, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import TextSubstitution, EqualsSubstitution, NotEqualsSubstitution, IfElseSubstitution
  

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    imu_topic_name = IfElseSubstitution(
                        LaunchConfiguration("use_sim_time"),
                        if_value="/imu_filter_madgwick/imu/data",
                        else_value="/mti_630_8A1G6/imu/data")
    
    
    return LaunchDescription([
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='Flse',
                description='Use simulation (Gazebo) clock if true'),

            LogInfo(condition=IfCondition(PythonExpression(['not ', use_sim_time])),
                msg=['use_sim_time set to False. Using ', imu_topic_name]
            ),

            LogInfo(condition=IfCondition(use_sim_time),
                msg=['use_sim_time set to True. Using ', imu_topic_name]
            ),

            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time},
                            PathJoinSubstitution([FindPackageShare('hibachi_localization'), 'config', 'dual_ekf_navsat_params.yaml'])],
                remappings=[("odometry/filtered", "odometry/local"),
                            ("imu/data", imu_topic_name)],
            ),

            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time},
                            PathJoinSubstitution([FindPackageShare('hibachi_localization'), 'config', 'dual_ekf_navsat_params.yaml'])],
                remappings=[("odometry/filtered", "odometry/global"),
                            ("imu/data", imu_topic_name)],
            ),

            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time},
                            PathJoinSubstitution([FindPackageShare('hibachi_localization'), 'config', 'dual_ekf_navsat_params.yaml'])],
                remappings=[
                    ("imu", imu_topic_name),
                    ("gps/fix", "ardusimple/fix"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
                arguments=['--ros-args', '--log-level', 'INFO'],
            ),
    ])