from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    twist_mux = Node(
            package="twist_stamper",
            executable="twist_stamper",
            parameters=[{'frame_id': 'base_link'}, {'use_sim_time': use_sim_time}],
            # Default publisher for twist_mux is /cmd_vel_out
            # Rename so it matches diff_drive_controller topic
            remappings=[('/cmd_vel_out','/hibachi_base_controller/cmd_vel'),
                        ('/cmd_vel_in', '/hibachi_base_controller/cmd_vel_unstamped')]
        )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),
        twist_mux,
    ])