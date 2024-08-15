# import os

# from ament_index_python.packages import get_package_share_directory
# from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


def generate_launch_description():

   use_sim_time = LaunchConfiguration('use_sim_time', default='true')
   nav2_params_file = PathJoinSubstitution([FindPackageShare('hibachi_navigation'), 'config', 'nav2_no_map_params.yaml'])

   gazebo_gps_world_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("hibachi_gz"),
             "launch",
             "hibachi_gz_gps_world.launch.py"],
         )
      ),
   )

   dual_ekf_navsat_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("hibachi_localization"),
             "launch",
             "dual_ekf_navsat.launch.py"],
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
   
   twist_stamper_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("twist_stamper_cpp"),
             "launch",
             "twist_stamper_cpp_remapped_launch.py"],
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

#    nav2_localization_launch = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource(
#          PathJoinSubstitution(
#             [FindPackageShare("hibachi_navigation"),
#              "launch",
#              "localization.launch.py"],
#          )
#       ),
#       launch_arguments= {'use_sim_time': use_sim_time}.items(),
#    )
  
   nav2_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution([FindPackageShare('nav2_bringup'),
            'launch',
            'navigation_launch.py']),
      ),
      launch_arguments = {'use_sim_time': use_sim_time,
                           'params_file': nav2_params_file}.items()
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
      gazebo_gps_world_launch,
      dual_ekf_navsat_launch,
      # teleop_joy_launch,
      teleop_twist_mux_launch,
      twist_stamper_launch,
      nav2_launch,
      # rviz_nav2_launch,
   ])