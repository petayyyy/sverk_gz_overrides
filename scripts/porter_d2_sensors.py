#!/usr/bin/env python3
"""Shared Harmonic sensor builders used by generate_porter_d2.py."""
import json
import math
import xml.etree.ElementTree as ET


def sub(parent, tag, text=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if text is not None:
        element.text = str(text)
    return element

def xml_write(root, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space='  ')
    with path.open('wb') as stream:
        ET.ElementTree(root).write(stream, encoding='utf-8', xml_declaration=True)
        stream.write(b'\n')

def pose_mm(point):
    return ' '.join(f'{v / 1000:.9f}' for v in point) + ' 0 0 0'

def frame(model, name, attached, pose='0 0 0 0 0 0'):
    sub(sub(model, 'frame', name=name, attached_to=attached), 'pose', pose, relative_to=attached)

def sensing_frame(model, name, link, point):
    frame(model, name, link, pose_mm(point))

def gaussian(parent, stddev, imu=False, bias=None):
    noise = sub(parent, 'noise', **({'type': 'gaussian'} if imu else {}))
    if not imu:
        sub(noise, 'type', 'gaussian')
    sub(noise, 'mean', 0)
    sub(noise, 'stddev', stddev)
    if bias is not None:
        sub(noise, 'bias_mean', 0)
        sub(noise, 'bias_stddev', bias)
    return noise

def common_sensor(link, name, kind, origin_frame, topic, rate, header_frame):
    sensor = sub(link, 'sensor', name=name, type=kind)
    sub(sensor, 'pose', '0 0 0 0 0 0', relative_to=origin_frame)
    sub(sensor, 'topic', topic)
    sub(sensor, 'gz_frame_id', header_frame)
    sub(sensor, 'always_on', 'false')
    sub(sensor, 'update_rate', rate)
    sub(sensor, 'visualize', 'false')
    return sensor

def add_camera(model, link, name, point_frame, topic, cfg, optical):
    # This metadata frame has +Z forward / +X image-right / +Y image-down.
    # The Gazebo sensor itself keeps zero rotation and looks along local +X.
    frame(model, optical, point_frame, '0 0 0 -1.5707963267948966 0 -1.5707963267948966')
    sensor = common_sensor(link, name, 'camera', point_frame, topic + '/image', cfg['fps'], optical)
    camera = sub(sensor, 'camera')
    sub(camera, 'horizontal_fov', f'{math.radians(cfg["horizontal_fov_deg"]):.12f}')
    image = sub(camera, 'image')
    sub(image, 'width', cfg['width'])
    sub(image, 'height', cfg['height'])
    sub(image, 'format', cfg['format'])
    clip = sub(camera, 'clip')
    sub(clip, 'near', cfg['clip_near_m'])
    sub(clip, 'far', cfg['clip_far_m'])
    sub(camera, 'optical_frame_id', optical)
    sub(camera, 'camera_info_topic', topic + '/camera_info')
    gaussian(camera, cfg['image_noise_stddev'])

def add_range(link, name, point_frame, topic, cfg):
    sensor = common_sensor(link, name, 'gpu_lidar', point_frame, topic + '/scan', cfg['fps'], point_frame)
    lidar = sub(sensor, 'lidar')
    scan = sub(lidar, 'scan')
    for axis in ('horizontal', 'vertical'):
        plane = sub(scan, axis)
        sub(plane, 'samples', cfg[axis + '_samples'])
        sub(plane, 'resolution', 1)
        half_angle = math.radians(cfg[axis + '_fov_deg']) / 2
        sub(plane, 'min_angle', f'{-half_angle:.12f}')
        sub(plane, 'max_angle', f'{half_angle:.12f}')
    bounds = sub(lidar, 'range')
    sub(bounds, 'min', cfg['min_m'])
    sub(bounds, 'max', cfg['max_m'])
    sub(bounds, 'resolution', 0.001)
    gaussian(lidar, cfg['noise_stddev_m'])

def add_imu(link, name, point_frame, topic, cfg):
    sensor = common_sensor(link, name, 'imu', point_frame, topic, cfg['fps'], point_frame)
    imu = sub(sensor, 'imu')
    for group, noise_key, bias_key in (
        ('angular_velocity', 'gyro_noise_stddev_rad_s', 'gyro_bias_stddev_rad_s'),
        ('linear_acceleration', 'accel_noise_stddev_m_s2', 'accel_bias_stddev_m_s2'),
    ):
        group_element = sub(imu, group)
        for axis in 'xyz':
            gaussian(sub(group_element, axis), cfg[noise_key], imu=True, bias=cfg[bias_key])

def build_model(cfg, prefix):
    name = cfg['name']
    root = ET.Element('sdf', version='1.9')
    model = sub(root, 'model', name=name)
    sub(model, 'static', 'false')
    sub(model, 'self_collide', 'false')
    model.append(ET.Comment(' Mesh datum and FLU axes preserved; synthetic noise and provisional settings are recorded in porter_D2/config/sensor_profiles.json. '))
    link_name = name + '_link'
    link = sub(model, 'link', name=link_name)
    inertial = sub(link, 'inertial')
    sub(inertial, 'mass', 0.000001)
    inertia = sub(inertial, 'inertia')
    for axis in ('ixx', 'iyy', 'izz'):
        sub(inertia, axis, '1e-12')
    for axis in ('ixy', 'ixz', 'iyz'):
        sub(inertia, axis, 0)
    visual = sub(link, 'visual', name=name + '_housing')
    sub(visual, 'cast_shadows', 'false')
    mesh = sub(sub(visual, 'geometry'), 'mesh')
    sub(mesh, 'uri', 'model://' + name + '/meshes/sensor.obj')
    sub(mesh, 'scale', '0.001 0.001 0.001')
    frame(model, name + '_origin', link_name)
    point_frame = name + '_sensing'
    sensing_frame(model, point_frame, link_name, cfg['origin_mm'])
    topic = prefix + '/' + cfg['topic']
    if cfg['kind'] in ('camera', 'optical_flow'):
        camera_topic = topic + ('/flow' if cfg['kind'] == 'optical_flow' else '')
        add_camera(model, link, name + '_imager', point_frame, camera_topic, cfg['camera'], name + '_optical')
    if cfg['kind'] == 'range':
        add_range(link, name + '_range', point_frame, topic, cfg['range'])
    if cfg['kind'] == 'optical_flow':
        tof_frame, imu_frame = name + '_tof', name + '_imu'
        sensing_frame(model, tof_frame, link_name, cfg['range']['origin_mm'])
        sensing_frame(model, imu_frame, link_name, cfg['imu']['origin_mm'])
        add_range(link, name + '_tof', tof_frame, topic + '/tof', cfg['range'])
        add_imu(link, name + '_imu', imu_frame, topic + '/imu', cfg['imu'])
    return root

def bridge_entries(config):
    result = [{'ros_topic_name': '/clock', 'gz_topic_name': '/clock', 'ros_type_name': 'rosgraph_msgs/msg/Clock', 'gz_type_name': 'gz.msgs.Clock', 'direction': 'GZ_TO_ROS'}]
    for cfg in config['sensors']:
        topic = config['topic_prefix'] + '/' + cfg['topic']
        pairs = []
        if cfg['kind'] in ('camera', 'optical_flow'):
            image_topic = topic + ('/flow' if cfg['kind'] == 'optical_flow' else '')
            pairs += [(image_topic + '/image', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
                      (image_topic + '/camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo')]
        if cfg['kind'] == 'range':
            pairs += [(topic + '/scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan')]
        if cfg['kind'] == 'optical_flow':
            pairs += [(topic + '/tof/scan/points', 'sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked'),
                      (topic + '/imu', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU')]
        for name, ros_type, gz_type in pairs:
            result.append(dict(ros_topic_name=name, gz_topic_name=name, ros_type_name=ros_type, gz_type_name=gz_type, direction='GZ_TO_ROS', qos_profile='SENSOR_DATA'))
    return result
