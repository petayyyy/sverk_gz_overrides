from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def camera_info_adapters(context):
    bridge_file = LaunchConfiguration('bridge_file').perform(context)
    with open(bridge_file) as stream:
        entries = yaml.safe_load(stream)
    nodes = []
    for index, entry in enumerate(entries):
        if entry.get('ros_type_name') != 'sensor_msgs/msg/CameraInfo':
            continue
        source = entry['ros_topic_name']
        if not source.endswith('_gz'):
            raise ValueError('Gazebo CameraInfo must bridge to a separate *_gz topic: ' + source)
        nodes.append(Node(package='gazebo_camera_tools', executable='camera_info',
            name='porter_camera_info_' + str(index),
            parameters=[{'input_topic': source, 'output_topic': source[:-3],
                         'use_sim_time': LaunchConfiguration('use_sim_time')}], output='screen'))
    return nodes

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
        OpaqueFunction(function=camera_info_adapters),
        Node(package='porter_sensor_adapters', executable='hflow_adapter', parameters=parameters, output='screen'),
        Node(package='porter_sensor_adapters', executable='range_adapters', parameters=parameters, output='screen'),
        Node(package='porter_sensor_adapters', executable='sensor_frames', parameters=parameters, output='screen'),
    ])
