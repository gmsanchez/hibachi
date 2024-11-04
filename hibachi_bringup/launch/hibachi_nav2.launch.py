# import os

# from ament_index_python.packages import get_package_share_directory
# from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


def generate_launch_description():

   use_sim_time = LaunchConfiguration('use_sim_time', default='false')

   ARGUMENTS = [
      DeclareLaunchArgument('slam_param_files', default_value=PathJoinSubstitution([FindPackageShare('hibachi_navigation'), 'config', 'mapper_params_online_async.yaml']),
            description='Full path to param yaml file to load for SLAM Online Async'),
      DeclareLaunchArgument('nav2_params_file', default_value=PathJoinSubstitution([FindPackageShare('hibachi_navigation'), 'config', 'nav2_params.yaml']),
            description='Full path to param yaml file to load for Nav2'),
      DeclareLaunchArgument('amcl_params_file', default_value=PathJoinSubstitution([FindPackageShare('hibachi_navigation'), 'config', 'nav2_params.yaml']),
            description='Full path to param yaml file to load for AMCL'),
      DeclareLaunchArgument('amcl_map_file', default_value=PathJoinSubstitution([FindPackageShare('hibachi_navigation'), 'maps', 'turtlebot3_world.yaml']),
            description='Full path to map yaml file to load for AMCL')
   ]

   slam_params_file = LaunchConfiguration('slam_param_files')
   nav2_params_file = LaunchConfiguration('nav2_params_file')
   amcl_params_file = LaunchConfiguration('amcl_params_file')
   amcl_map_file = LaunchConfiguration('amcl_map_file')

   # Cargar:
   # - hibachi_description (XACRO > robot_state_publisher)
   # - hibachi_control = joint_state_broadcaster + hibachi_base_controller
   hibachi_control_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("hibachi_control"),
            "launch",
            "hibachi_control.launch.py"],
         )
      ),
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

   twist_stamper_cpp_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("twist_stamper_cpp"),
             "launch",
             "twist_stamper_cpp_remapped_launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time}.items(),
   )

   xsens_mti630_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("xsens_mti630_bringup"),
             "launch",
             "mti630.launch.py"],
         )
      ),
      # launch_arguments= {'use_sim_time': use_sim_time}.items(),
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

   rplidar_a1_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("rplidar_a1_bringup"),
             "launch",
             "rplidar.launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time}.items(),
   )

   nav2_amcl_localization_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("nav2_bringup"),
             "launch",
             "localization_launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time,
                        'params_file': amcl_params_file,
                        'map': amcl_map_file}.items(),
   )

   online_async_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("slam_toolbox"),
               "launch",
               "online_async_launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time,
                         'slam_params_file': slam_params_file}.items(),
      # launch_arguments= {'use_sim_time': use_sim_time}.items(),
   )

   nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('hibachi_navigation'),
               'launch',
               'navigation_launch.py']),
        ),
        launch_arguments = {'use_sim_time': use_sim_time,
                            'params_file': nav2_params_file}.items()
    )


   ld = LaunchDescription(ARGUMENTS)
   ld.add_action(hibachi_control_launch)
   ld.add_action(teleop_twist_mux_launch)
   ld.add_action(teleop_joy_launch)
   # ld.add_action(teleop_twist_stamper_launch)
   ld.add_action(twist_stamper_cpp_launch)
   ld.add_action(xsens_mti630_launch)
   ld.add_action(rplidar_a1_launch)
   ld.add_action(ekf_launch)
   ld.add_action(online_async_launch)
   # ld.add_action(nav2_amcl_localization_launch)
   ld.add_action(nav2_launch)

   return ld
