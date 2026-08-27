"""Launch the standalone Livox Avia test world, ROS bridge, and RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _configured_nodes(context):
    mode = LaunchConfiguration('scan_mode').perform(context).lower()
    if mode not in ('nonrepetitive', 'repetitive'):
        raise RuntimeError('scan_mode must be nonrepetitive or repetitive')
    gz_topic = f'/livox_avia/{mode}/scan/points'
    return [
        ExecuteProcess(cmd=['gz', 'sim', '-r', 'livox_avia_test.sdf'], output='screen'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[f'{gz_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
            remappings=[(gz_topic, '/livox_avia/raw_points')],
            output='screen',
        ),
        Node(
            package='livox_avia_cloud_adapter',
            executable='cloud_adapter',
            parameters=[{'scan_mode': mode}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(Path(get_package_share_directory(
                'livox_avia_cloud_adapter')) / 'rviz/livox_avia.rviz')],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scan_mode', default_value='nonrepetitive',
            description='Livox Avia scan mode: nonrepetitive or repetitive'),
        OpaqueFunction(function=_configured_nodes),
    ])
