"""Correct only Gazebo CameraInfo, before any ROS consumers see it."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo
from .calibration import to_opencv

class CameraInfoAdapter(Node):
    def __init__(self):
        super().__init__('gazebo_camera_info')
        self.declare_parameter('input_topic', '/camera/camera_info_gz')
        self.declare_parameter('output_topic', '/camera/camera_info')
        source = self.get_parameter('input_topic').value
        target = self.get_parameter('output_topic').value
        if self.resolve_topic_name(source) == self.resolve_topic_name(target):
            raise ValueError('CameraInfo input and output must differ: avoid repeated half-pixel correction')
        incoming = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        outgoing = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(CameraInfo, target, outgoing)
        self.subscription = self.create_subscription(CameraInfo, source, self.callback, incoming)
    def callback(self, message):
        self.publisher.publish(to_opencv(message))

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
