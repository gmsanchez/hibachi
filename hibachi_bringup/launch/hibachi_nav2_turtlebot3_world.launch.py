# import os

# from ament_index_python.packages import get_package_share_directory
# from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


def generate_launch_description():

   use_sim_time = LaunchConfiguration('use_sim_time', default='True')

   gazebo_turtlebot_world_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("hibachi_gz"),
             "launch",
             "hibachi_gz_turtlebot3_world.launch.py"],
         )
      ),
   )

   ekf_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("hibachi_localization"),
             "launch",
             "ekf.launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time}.items(),
   )
   

   teleop_joy_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("hibachi_teleop"),
             "launch",
             "hibachi_teleop_twist_joy.launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time}.items(),
   )
   
   teleop_twist_stamper_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("hibachi_teleop"),
             "launch",
             "hibachi_twist_stamper.launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time}.items(),
   )

   teleop_twist_mux_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("hibachi_teleop"),
             "launch",
             "hibachi_twist_mux.launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time}.items(),
   )

   nav2_localization_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("nav2_bringup"),
             "launch",
             "amcl_localization.launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time}.items(),
   )

   nav2_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("hibachi_navigation"),
             "launch",
             "nav2.launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time}.items(),
   )

   rviz_nav2_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("hibachi_rviz"),
             "launch",
             "hibachi_rviz_nav2.launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time,
                         'rviz_config_file': PathJoinSubstitution(
                [FindPackageShare("hibachi_rviz"), "rviz", "map_nav2.rviz"])}.items(),
   )
   
   return LaunchDescription([
      gazebo_turtlebot_world_launch,
      ekf_launch,
      # teleop_joy_launch,
      teleop_twist_mux_launch,
      teleop_twist_stamper_launch,
      nav2_localization_launch,
      nav2_launch,
      rviz_nav2_launch,
   ])