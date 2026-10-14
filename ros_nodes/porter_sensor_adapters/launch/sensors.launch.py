from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share = Path(get_package_share_directory('porter_sensor_adapters'))
    config = LaunchConfiguration('config_file')
    bridge = LaunchConfiguration('bridge_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    parameters = [{'config_file': config, 'use_sim_time': use_sim_time}]
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(share / 'config/sensors.json')),
        DeclareLaunchArgument('bridge_file', default_value=str(share / 'config/bridge.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='porter_sensor_bridge',
             parameters=[{'config_file': bridge, 'use_sim_time': use_sim_time}], output='screen'),
        Node(package='porter_sensor_adapters', executable='hflow_adapter', parameters=parameters, output='screen'),
        Node(package='porter_sensor_adapters', executable='range_adapters', parameters=parameters, output='screen'),
        Node(package='porter_sensor_adapters', executable='sensor_frames', parameters=parameters, output='screen'),
    ])
