"""Shape Gazebo range images into realistic Livox Avia point clouds."""

import gzip
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


POINTS_PER_FRAME = 24000
PATTERN_POINTS = 960000


class LivoxAviaCloudAdapter(Node):
    """Publish 240 kpoint/s clouds in either Avia scan mode."""

    def __init__(self):
        super().__init__('livox_avia_cloud_adapter')
        self.declare_parameter('input_topic', '/livox_avia/raw_points')
        self.declare_parameter('output_topic', '/livox_avia/points')
        self.declare_parameter('scan_mode', 'nonrepetitive')
        self.declare_parameter('frame_id', 'livox_avia')

        self._mode = str(self.get_parameter('scan_mode').value).lower()
        if self._mode not in ('nonrepetitive', 'repetitive'):
            raise ValueError('scan_mode must be nonrepetitive or repetitive')
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._phase = 0
        self._pattern = None
        self._source_indices = None
        self._source_shape = None
        if self._mode == 'nonrepetitive':
            pattern_path = Path(get_package_share_directory(
                'livox_avia_cloud_adapter')) / 'data/avia_pattern_i16.bin.gz'
            with gzip.open(pattern_path, 'rb') as stream:
                self._pattern = np.frombuffer(stream.read(), dtype='<i2').reshape(-1, 2)
            if self._pattern.shape != (PATTERN_POINTS, 2):
                raise RuntimeError(f'invalid Avia pattern shape: {self._pattern.shape}')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._publisher = self.create_publisher(PointCloud2, output_topic, 2)
        self._subscription = self.create_subscription(
            PointCloud2, input_topic, self._cloud_callback, 2)
        self.get_logger().info(
            f'Livox Avia {self._mode}: {input_topic} -> {output_topic}; '
            '10 Hz, 24000 points/cloud, strongest-return approximation')

    def _cloud_callback(self, message: PointCloud2):
        height = int(message.height)
        width = int(message.width)
        point_step = int(message.point_step)
        row_step = int(message.row_step)
        if (height < 1 or width < 1 or point_step < 1 or
                row_step < width * point_step or len(message.data) < height * row_step):
            self.get_logger().warning('Ignoring malformed Avia PointCloud2 message')
            return

        if self._mode == 'repetitive':
            if row_step == width * point_step:
                data = bytes(message.data[:height * row_step])
            else:
                data = b''.join(
                    bytes(message.data[row * row_step:row * row_step + width * point_step])
                    for row in range(height))
            count = width * height
        else:
            data = self._sample_nonrepetitive(message)
            count = POINTS_PER_FRAME

        output = PointCloud2()
        output.header = message.header
        output.header.frame_id = self._frame_id
        output.height = 1
        output.width = count
        output.fields = message.fields
        output.is_bigendian = message.is_bigendian
        output.point_step = point_step
        output.row_step = count * point_step
        output.data = data
        output.is_dense = message.is_dense
        self._publisher.publish(output)

    def _sample_nonrepetitive(self, message):
        """Nearest-sample Livox's published angles from the GPU range image."""
        width = int(message.width)
        height = int(message.height)
        point_step = int(message.point_step)
        row_step = int(message.row_step)
        source_shape = (height, width)
        if self._source_shape != source_shape:
            # Pattern values are centidegrees. Precompute all 40 frames' source
            # indices once; NumPy can then gather a frame without a Python loop.
            columns = np.rint(
                (self._pattern[:, 0].astype(np.float32) + 3520.0) *
                (width - 1) / 7040.0).clip(0, width - 1).astype(np.int32)
            rows = np.rint(
                (self._pattern[:, 1].astype(np.float32) + 3860.0) *
                (height - 1) / 7720.0).clip(0, height - 1).astype(np.int32)
            self._source_indices = rows * width + columns
            self._source_shape = source_shape

        raw = np.frombuffer(message.data, dtype=np.uint8).reshape(height, row_step)
        points = raw[:, :width * point_step].reshape(height * width, point_step)
        frame_indices = self._source_indices[
            self._phase:self._phase + POINTS_PER_FRAME]
        output = points[frame_indices].tobytes()
        self._phase = (self._phase + POINTS_PER_FRAME) % PATTERN_POINTS
        return output


def main(args=None):
    rclpy.init(args=args)
    node = LivoxAviaCloudAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
