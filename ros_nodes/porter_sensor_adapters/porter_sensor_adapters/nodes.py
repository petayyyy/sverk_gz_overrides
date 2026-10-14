"""ROS 2 endpoints; no flight-controller input is enabled implicitly."""
import json
import math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, Range, Imu, PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from cv_bridge import CvBridge
from porter_sensor_msgs.msg import OpticalFlow
from .flow_core import estimate_flow, image_flow_to_flu, scalar_range, fresh_range, FlowNoise
from .range_core import point_cloud_ranges

def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

def config(node):
    from ament_index_python.packages import get_package_share_directory
    default = str(Path(get_package_share_directory('porter_sensor_adapters')) / 'config/sensors.json')
    node.declare_parameter('config_file', default)
    return json.loads(Path(node.get_parameter('config_file').value).read_text(encoding='utf-8'))

class HFlow(Node):
    def __init__(self):
        super().__init__('porter_hflow_adapter')
        cfg = config(self)
        self.sensor = next(s for s in cfg['sensors'] if s['kind'] == 'optical_flow')
        self.settings = self.sensor['flow']
        topic = cfg['topic_prefix'] + '/' + self.sensor['topic']
        self.publisher = self.create_publisher(OpticalFlow, topic + '/optical_flow', qos_profile_sensor_data)
        self.image_sub = self.create_subscription(Image, topic + '/flow/image', self.on_image, qos_profile_sensor_data)
        self.range_sub = self.create_subscription(Range, topic + '/tof/range', self.on_range, qos_profile_sensor_data)
        self.bridge = CvBridge()
        self.previous = None
        self.previous_stamp = None
        self.range_sample = None
        self.noise = FlowNoise(self.settings['white_noise_rate_rad_s'], self.settings['bias_rate_stddev_rad_s'], self.settings['seed'])

    def on_range(self, message):
        lower = max(message.min_range, self.settings['min_distance_m'])
        upper = min(message.max_range, self.settings['max_distance_m'])
        value = float(message.range)
        if not lower <= value <= upper:
            value = math.nan
        self.range_sample = (stamp_ns(message.header), value)

    def on_image(self, message):
        try:
            current = self.bridge.imgmsg_to_cv2(message, desired_encoding='mono8').copy()
        except Exception as exc:
            self.get_logger().warning('H-Flow image conversion failed: ' + str(exc))
            return
        stamp = stamp_ns(message.header)
        previous, previous_stamp = self.previous, self.previous_stamp
        self.previous, self.previous_stamp = current, stamp
        if previous is None:
            return
        dt = (stamp - previous_stamp) / 1e9
        if dt <= 0:
            self.range_sample = None
            return  # Gazebo rewind/reset: do not integrate across the reset.
        if dt > self.settings['max_image_gap_s'] or previous.shape != current.shape:
            return
        try:
            estimate = estimate_flow(previous, current, math.radians(self.sensor['camera']['horizontal_fov_deg']))
        except (ValueError, RuntimeError) as exc:
            self.get_logger().warning('H-Flow tracking failed: ' + str(exc))
            return
        flow = np.array([estimate.right, estimate.down])
        quality = estimate.quality
        if quality:
            flow = self.noise.apply(flow, dt)
        if np.linalg.norm(flow) / dt > self.settings['max_rate_rad_s']:
            quality = 0
        if not quality:
            flow[:] = 0
        distance = fresh_range(self.range_sample, stamp, self.settings['range_max_age_s'],
                               self.settings['min_distance_m'], self.settings['max_distance_m'])
        result = OpticalFlow()
        result.header.stamp = message.header.stamp
        result.header.frame_id = self.sensor['name'] + '_sensing'
        result.integration_time_s = dt
        result.image_flow_rad = flow.tolist()
        angular = image_flow_to_flu(*flow)
        result.angular_displacement.x, result.angular_displacement.y, result.angular_displacement.z = map(float, angular)
        result.quality = quality
        result.tracked_features = estimate.tracked
        result.distance_valid = math.isfinite(distance)
        result.distance_m = distance
        result.max_flow_rate_rad_s = self.settings['max_rate_rad_s']
        self.publisher.publish(result)

class RangeAdapters(Node):
    def __init__(self):
        super().__init__('porter_range_adapters')
        cfg = config(self)
        self.subscriptions_owned = []
        for sensor in cfg['sensors']:
            if sensor['kind'] not in ('range', 'optical_flow'):
                continue
            topic = cfg['topic_prefix'] + '/' + sensor['topic']
            if sensor['kind'] == 'optical_flow':
                topic += '/tof'
            pub = self.create_publisher(Range, topic + '/range', qos_profile_sensor_data)
            is_flow = sensor['kind'] == 'optical_flow'
            def callback(scan, pub=pub, settings=sensor['range'], is_flow=is_flow):
                if is_flow:
                    try:
                        values = point_cloud_ranges(scan)
                        if len(values) != settings['horizontal_samples'] * settings['vertical_samples']:
                            raise ValueError('Unexpected ToF pixel count')
                    except ValueError as error:
                        self.get_logger().warning(str(error))
                        return
                    lower, upper = settings['min_m'], settings['max_m']
                else:
                    values = scan.ranges
                    lower, upper = max(scan.range_min, settings['min_m']), min(scan.range_max, settings['max_m'])
                if lower >= upper:
                    return
                msg = Range()
                msg.header = scan.header
                msg.radiation_type = Range.INFRARED
                msg.field_of_view = math.radians(max(settings['horizontal_fov_deg'], settings['vertical_fov_deg']))
                msg.min_range, msg.max_range = float(lower), float(upper)
                msg.range = scalar_range(values, lower, upper)
                pub.publish(msg)
            kind = PointCloud2 if is_flow else LaserScan
            suffix = '/scan/points' if is_flow else '/scan'
            self.subscriptions_owned.append(self.create_subscription(kind, topic + suffix, callback, qos_profile_sensor_data))

class SensorFrames(Node):
    def __init__(self):
        super().__init__('porter_sensor_frames')
        cfg = config(self)
        self.broadcaster = StaticTransformBroadcaster(self)
        transforms = []
        def add(parent, child, point, quaternion=(0.0, 0.0, 0.0, 1.0)):
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id, tf.child_frame_id = parent, child
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = [float(v) / 1000 for v in point]
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, quaternion)
            transforms.append(tf)
        for mount in cfg.get('mount_transforms', []):
            add(mount['parent'], mount['child'], mount['xyz_mm'], tuple(mount['quaternion_xyzw']))
        for sensor in cfg['sensors']:
            name = sensor['name']
            add(name + '_origin', name + '_sensing', sensor['origin_mm'],
                tuple(sensor.get('sensing_quaternion_xyzw', (0.0, 0.0, 0.0, 1.0))))
            if sensor['kind'] in ('camera', 'optical_flow'):
                add(name + '_sensing', name + '_optical', [0, 0, 0], (-0.5, 0.5, -0.5, 0.5))
            if sensor['kind'] == 'optical_flow':
                add(name + '_origin', name + '_tof', sensor['range']['origin_mm'])
                add(name + '_origin', name + '_imu', sensor['imu']['origin_mm'])
        self.broadcaster.sendTransform(transforms)

class AviaImuFrame(Node):
    """Preserve native Avia data and stamp; identify its measured local IMU frame."""
    def __init__(self):
        super().__init__('porter_D2_avia_imu_frame')
        self.declare_parameter('input_topic', '/porter_D2/avia/raw_imu')
        self.declare_parameter('output_topic', '/porter_D2/avia/imu')
        self.declare_parameter('frame_id', 'porter_D2_avia_imu')
        source = self.get_parameter('input_topic').value
        target = self.get_parameter('output_topic').value
        if self.resolve_topic_name(source) == self.resolve_topic_name(target):
            raise ValueError('Avia IMU input and output must differ')
        self.publisher = self.create_publisher(Imu, target, qos_profile_sensor_data)
        self.subscription = self.create_subscription(Imu, source, self.on_imu, qos_profile_sensor_data)

    def on_imu(self, message):
        message.header.frame_id = self.get_parameter('frame_id').value
        self.publisher.publish(message)

def run(node_type):
    rclpy.init()
    node = node_type()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

def hflow_main():
    run(HFlow)

def range_main():
    run(RangeAdapters)

def frames_main():
    run(SensorFrames)

def avia_imu_main():
    run(AviaImuFrame)
