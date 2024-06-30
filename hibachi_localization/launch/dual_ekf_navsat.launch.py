from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
            DeclareLaunchArgument(
                'use_sim_time',
                default_value="false",
                description='Use simulation (Gazebo) clock if true'),
            
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time},
                            PathJoinSubstitution([FindPackageShare('hibachi_localization'), 'config', 'dual_ekf_navsat_params.yaml'])],
                remappings=[("odometry/filtered", "odometry/local")],
            ),

            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time},
                            PathJoinSubstitution([FindPackageShare('hibachi_localization'), 'config', 'dual_ekf_navsat_params.yaml'])],
                remappings=[("odometry/filtered", "odometry/global")],
            ),

            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time},
                            PathJoinSubstitution([FindPackageShare('hibachi_localization'), 'config', 'dual_ekf_navsat_params.yaml'])],
                remappings=[
                    ("imu", "/mti_630_8A1G6/imu/data"),
                    ("gps/fix", "ardusimple/fix"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
            ),
    ])