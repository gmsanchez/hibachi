from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_mag = LaunchConfiguration('use_mag')
    # imu_filter_madgwick_config = PathJoinSubstitution([FindPackageShare('hibachi_localization'), 'config', 'imu_filter_madgwick_with_mag.yaml'])
    
    ARGUMENTS = [
        DeclareLaunchArgument('use_sim_time', default_value='False',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('use_mag', default_value='true',
            description='Madgwick filter: Whether to use the magnetic field data in the data fusion.'),
    ]
    
    imu_filter_madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        namespace='imu_filter_madgwick',
        output='screen',
        parameters=[
                    {'use_mag': use_mag},
                    {'use_magnetic_field_msg': True},
                    {'stateless': False},
                    {'publish_tf': False},
                    {'reverse_tf': False},
                    {'fixed_frame': 'odom'},
                    {'constant_dt': 0.0},
                    {'publish_debug_topics': True},
                    {'world_frame': 'enu'},
                    {'gain': 0.1},
                    {'zeta': 0.0},
                    {'mag_bias_x': 0.0},
                    {'mag_bias_y': 0.0},
                    {'mag_bias_z': 0.0},
                    {'orientation_stddev': 0.0},
                    {'use_sim_time': use_sim_time},
                    # imu_filter_madgwick_config
                    ],
        remappings=[('imu/data_raw','/mti_630_8A1G6/imu/data'),
                    ('imu/mag', '/mti_630_8A1G6/imu/mag'),
                    # ('/imu/data', '/imu_filter_madgwick/imu/data')
                    ]
    )
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(imu_filter_madgwick)
        
    return ld
