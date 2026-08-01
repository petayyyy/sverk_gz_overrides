"""Convert a rectangular GPU-lidar cloud into equal-area angular samples."""

from math import cos

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class EqualAreaCloudAdapter(Node):
    """Keep fewer azimuth samples where equal angular rays converge at a pole."""

    def __init__(self):
        super().__init__('unitree_l2_cloud_adapter')
        self.declare_parameter('input_topic', '/unitree_l2/raw_points')
        self.declare_parameter('output_topic', '/unitree_l2/points')
        self.declare_parameter('vertical_min_angle', 0.0)
        self.declare_parameter('vertical_max_angle', 1.5707963267948966)
        self.declare_parameter('min_points_per_channel', 1)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._vertical_min_angle = float(
            self.get_parameter('vertical_min_angle').value)
        self._vertical_max_angle = float(
            self.get_parameter('vertical_max_angle').value)
        self._min_points_per_channel = max(1, int(
            self.get_parameter('min_points_per_channel').value))

        self._publisher = self.create_publisher(
            PointCloud2, output_topic, 5)
        self._subscription = self.create_subscription(
            PointCloud2, input_topic, self._cloud_callback, 5)
        self.get_logger().info(
            f'Equal-area L2 cloud: {input_topic} -> {output_topic}, '
            f'elevation [{self._vertical_min_angle:.6f}, '
            f'{self._vertical_max_angle:.6f}] rad')

    def _cloud_callback(self, message: PointCloud2):
        """Decimate each organized elevation row by cos(elevation).

        A conventional GPU lidar produces the same azimuth count in every
        elevation row.  The horizontal arc length shrinks by cos(elevation),
        so retaining width * cos(elevation) samples distributes the points
        approximately uniformly over solid angle.  The complete point record
        is copied unchanged, preserving Gazebo's fields and frame ID.
        """
        height = int(message.height)
        width = int(message.width)
        point_step = int(message.point_step)
        row_step = int(message.row_step)
        expected_bytes = height * row_step
        if height < 1 or width < 1 or point_step < 1 or len(message.data) < expected_bytes:
            self.get_logger().warning('Ignoring malformed L2 PointCloud2 message')
            return

        output_data = bytearray()
        output_count = 0
        last_row = max(1, height - 1)
        for row in range(height):
            elevation = self._vertical_min_angle + (
                (self._vertical_max_angle - self._vertical_min_angle) * row / last_row)
            keep_count = min(
                width,
                max(self._min_points_per_channel, int(round(width * abs(cos(elevation))))))
            row_offset = row * row_step
            for kept_index in range(keep_count):
                source_column = (kept_index * width) // keep_count
                point_offset = row_offset + source_column * point_step
                output_data.extend(message.data[point_offset:point_offset + point_step])
            output_count += keep_count

        output = PointCloud2()
        output.header = message.header
        output.height = 1
        output.width = output_count
        output.fields = message.fields
        output.is_bigendian = message.is_bigendian
        output.point_step = point_step
        output.row_step = output_count * point_step
        output.data = bytes(output_data)
        output.is_dense = message.is_dense
        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = EqualAreaCloudAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
