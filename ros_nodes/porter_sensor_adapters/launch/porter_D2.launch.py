"""Bridge Porter D2 sensors and publish CAD mounting TF. Gazebo runs separately."""
from pathlib import Path
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    share = Path(get_package_share_directory('porter_sensor_adapters'))
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('bridge_file', default_value=str(share/'config/bridge.yaml')),
        DeclareLaunchArgument('avia_texel_fix', default_value='true',
                              description='Existing Avia renderer correction, validated for ogre2_8_2_3.'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/sensors.launch.py')),
                                 launch_arguments={'bridge_file': LaunchConfiguration('bridge_file'),
                                                   'use_sim_time': LaunchConfiguration('use_sim_time')}.items()),
        Node(package='livox_avia_cloud_adapter', executable='cloud_adapter', name='porter_D2_avia_cloud_adapter',
             parameters=[{'use_sim_time': use_sim_time, 'input_topic': '/porter_D2/avia/raw_points',
                          'output_topic': '/porter_D2/avia/points', 'frame_id': 'porter_D2_avia',
                          'scan_mode': 'nonrepetitive',
                          # Bundled measured pattern extends 0.11 deg beyond
                          # nominal +/-35.2 deg; explicitly sample nearest edge.
                          'pattern_edge_tolerance_rad': math.radians(.12),
                          'texel_fix': ParameterValue(LaunchConfiguration('avia_texel_fix'), value_type=bool)}], output='screen'),
        Node(package='porter_sensor_adapters', executable='avia_imu_frame',
             parameters=[{'use_sim_time': use_sim_time}], output='screen'),
    ])
