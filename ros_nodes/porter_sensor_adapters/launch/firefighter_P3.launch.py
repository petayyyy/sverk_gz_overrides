"""Bridge Firefighter P3 payloads and measured mounts; Gazebo runs separately."""
from pathlib import Path
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def camera_bridges(context):
    requested=LaunchConfiguration('cameras').perform(context).strip().lower()
    valid=('n6','blackfly','foxeer')
    tokens={s.strip() for s in requested.split(',') if s.strip()}
    if tokens=={'all'}:tokens=set(valid)
    elif tokens in (set(),{'none'}):tokens=set()
    if tokens-set(valid):raise ValueError('P3 cameras must be n6,blackfly,foxeer,all or none')
    world=LaunchConfiguration('gz_world').perform(context)
    instance=LaunchConfiguration('model_instance').perform(context)
    output=[]
    for name in valid:
        if name not in tokens:continue
        base=('/world/'+world+'/model/'+instance+'/model/blackfly_sensor'
              '/link/bfs_pge_50s5c_camera_link/sensor/imager') if name=='blackfly' else '/firefighter_P3/'+name
        for suffix,ros_suffix,ros_type,gz_type in [('image','image_raw','Image','Image'),
                                                 ('camera_info','camera_info','CameraInfo','CameraInfo')]:
            topic=base+'/'+suffix
            output.append(Node(package='ros_gz_bridge',executable='parameter_bridge',
                name='firefighter_P3_'+name+'_'+suffix+'_bridge',
                arguments=[topic+'@sensor_msgs/msg/'+ros_type+'[gz.msgs.'+gz_type],
                remappings=[(topic,'/firefighter_P3/'+name+'/'+ros_suffix+('_gz' if ros_type=='CameraInfo' else ''))],
                parameters=[{'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}],
                output='screen'))
        output.append(Node(package='gazebo_camera_tools', executable='camera_info',
            name='firefighter_P3_'+name+'_camera_info',
            parameters=[{'input_topic':'/firefighter_P3/'+name+'/camera_info_gz',
                         'output_topic':'/firefighter_P3/'+name+'/camera_info',
                         'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}],
            output='screen'))
    return output


def generate_launch_description():
    share=Path(get_package_share_directory('porter_sensor_adapters'))
    sim_time=ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('gz_world',default_value='obrik_aruco'),
        DeclareLaunchArgument('model_instance',default_value='firefighter_P3'),
        DeclareLaunchArgument('cameras',default_value='n6,blackfly',
                              description='P3 image bridges: subset of n6,blackfly,foxeer, or all/none; Foxeer is off by default.'),
        DeclareLaunchArgument('bridge_file',default_value=str(share/'config/firefighter_P3_bridge.yaml')),
        DeclareLaunchArgument('avia_texel_fix',default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/sensors.launch.py')),
            launch_arguments={'config_file':str(share/'config/firefighter_P3_sensors.json'),
                              'bridge_file':LaunchConfiguration('bridge_file'),
                              'use_sim_time':LaunchConfiguration('use_sim_time')}.items()),
        Node(package='livox_avia_cloud_adapter',executable='cloud_adapter',name='firefighter_P3_avia_cloud_adapter',
            parameters=[{'use_sim_time':sim_time,'input_topic':'/firefighter_P3/avia/raw_points',
                         'output_topic':'/firefighter_P3/avia/points','frame_id':'firefighter_P3_avia',
                         'scan_mode':'nonrepetitive','pattern_edge_tolerance_rad':math.radians(.12),
                         'texel_fix':ParameterValue(LaunchConfiguration('avia_texel_fix'),value_type=bool)}],output='screen'),
        Node(package='porter_sensor_adapters',executable='avia_imu_frame',name='firefighter_P3_avia_imu_frame',
            parameters=[{'use_sim_time':sim_time,'input_topic':'/firefighter_P3/avia/raw_imu',
                         'output_topic':'/firefighter_P3/avia/imu','frame_id':'firefighter_P3_avia_imu'}],output='screen'),
        OpaqueFunction(function=camera_bridges),
    ])
