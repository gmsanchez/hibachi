from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ARGUMENTS = [
        DeclareLaunchArgument('nav2_params_file', default_value=PathJoinSubstitution([FindPackageShare('hibachi_navigation'), 'config', 'nav2_no_map_params.yaml']),
            description='Full path to param yaml file to load for Nav2'),
    ]
    nav2_params_file = LaunchConfiguration('nav2_params_file')

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

    twist_stamper_cpp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
            [FindPackageShare("twist_stamper_cpp"),
                "launch",
                "twist_stamper_cpp_launch.py"],
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

    ardusimple_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
            [FindPackageShare("gnss_bringup"),
                "launch",
                "ardusimple.launch.py"],
            )
        ),
        launch_arguments= {'use_sim_time': use_sim_time}.items(),
    )

    m8n_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
            [FindPackageShare("gnss_bringup"),
                "launch",
                "m8n.launch.py"],
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
    ld.add_action(teleop_joy_launch)
    ld.add_action(teleop_twist_mux_launch)
    ld.add_action(twist_stamper_cpp_launch)
    ld.add_action(xsens_mti630_launch)
    ld.add_action(ardusimple_launch)
    ld.add_action(m8n_launch)
    ld.add_action(rplidar_a1_launch)
    ld.add_action(dual_ekf_navsat_launch)
    ld.add_action(nav2_launch)

    return ld
