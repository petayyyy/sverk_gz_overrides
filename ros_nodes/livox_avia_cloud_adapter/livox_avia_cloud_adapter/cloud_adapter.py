"""Shape Gazebo range images into realistic Livox Avia point clouds."""

import gzip
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


POINTS_PER_FRAME = 24000
PATTERN_POINTS = 960000


def cube_face_texels(width, height):
    """Сторона грани кубической карты Ogre2GpuRays: max(W, H) -> степень двойки, clamp [128, 1024]
    (gz-rendering8 Ogre2GpuRays.cc:743–777)."""
    v = max(int(width), int(height), 1)
    return int(min(max(1 << (v - 1).bit_length(), 128), 1024))


def texel_fix(p32, ntex):
    """Точка на направление центра текселя с той же дальностью: p' = |p|·d_tex.

    gpu_lidar берёт дальность из БЛИЖАЙШЕГО текселя грани (gpu_rays.material: filtering none),
    то есть вдоль направления центра текселя, а gz-sensors ставит точку на ИДЕАЛЬНЫЙ луч. Отсюда
    неподвижный в кадре датчика узор ошибки ~47 мм RMS по высоте на 100 м и дрейф одометрии.
    Float32-арифметика выбора текселя соответствует GPU; дальность сохраняется.
    Только для грани +x (|y/x| <= 1, |z/x| <= 1) — так у Avia (FOV 70.4° x 77.2°)."""
    p = p32.astype(np.float64)
    r = np.linalg.norm(p, axis=1)
    u = np.float32(0.5) - np.float32(0.5) * (p32[:, 1] / p32[:, 0]).astype(np.float32)
    w = np.float32(0.5) - np.float32(0.5) * (p32[:, 2] / p32[:, 0]).astype(np.float32)
    uc = (np.floor(u.astype(np.float64) * ntex) + 0.5) / ntex
    wc = (np.floor(w.astype(np.float64) * ntex) + 0.5) / ntex
    d = np.stack([np.ones_like(uc), 1.0 - 2.0 * uc, 1.0 - 2.0 * wc], 1)
    d /= np.linalg.norm(d, axis=1)[:, None]
    return r[:, None] * d


def texel_fix_points(points, offsets, ntex):
    """points — uint8-массив (N, point_step), изменяется на месте. Возвращает (исправлено, вне грани)."""
    if points.shape[0] == 0:
        return 0, 0
    xyz = np.stack([np.ascontiguousarray(points[:, o:o + 4]).view('<f4')[:, 0] for o in offsets], 1)
    finite = np.isfinite(xyz).all(1) & (xyz[:, 0] > 1e-6)
    with np.errstate(divide='ignore', invalid='ignore'):
        on_face = finite & (np.abs(xyz[:, 1]) <= xyz[:, 0]) & (np.abs(xyz[:, 2]) <= xyz[:, 0])
    if on_face.any():
        xyz[on_face] = texel_fix(xyz[on_face], ntex).astype(np.float32)
        for k, o in enumerate(offsets):
            points[:, o:o + 4] = xyz[:, k].astype('<f4').view(np.uint8).reshape(-1, 4)
    return int(on_face.sum()), int((finite & ~on_face).sum())


class LivoxAviaCloudAdapter(Node):
    """Publish 240 kpoint/s clouds in either Avia scan mode."""

    def __init__(self):
        super().__init__('livox_avia_cloud_adapter')
        self.declare_parameter('input_topic', '/livox_avia/raw_points')
        self.declare_parameter('output_topic', '/livox_avia/points')
        self.declare_parameter('scan_mode', 'nonrepetitive')
        self.declare_parameter('frame_id', 'livox_avia')
        # Исправление узла текселя gpu_lidar (см. texel_fix). Отключать только
        # для воспроизведения поведения старых неисправленных записей.
        self.declare_parameter('texel_fix', True)
        # 0 — сторона грани из сетки лучей сенсора (как Ogre2GpuRays); иначе явное значение.
        self.declare_parameter('cube_face_texels', 0)

        self._mode = str(self.get_parameter('scan_mode').value).lower()
        if self._mode not in ('nonrepetitive', 'repetitive'):
            raise ValueError('scan_mode must be nonrepetitive or repetitive')
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._texel_fix = bool(self.get_parameter('texel_fix').value)
        self._ntex_param = int(self.get_parameter('cube_face_texels').value)
        self._ntex = None
        self._outside_warned = False
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
            '10 Hz, 24000 points/cloud, strongest-return approximation; '
            f'texel_fix={self._texel_fix}')

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

        if self._texel_fix:
            data = self._apply_texel_fix(message, data, count, point_step)

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

    def _apply_texel_fix(self, message, data, count, point_step):
        if self._ntex is None:
            self._ntex = self._ntex_param or cube_face_texels(message.width, message.height)
            self.get_logger().info(
                f'texel_fix: cube face {self._ntex}x{self._ntex} for gpu_lidar grid '
                f'{message.width}x{message.height}')
        names = {f.name: f for f in message.fields}
        if message.is_bigendian or any(
                n not in names or names[n].datatype != PointField.FLOAT32 for n in ('x', 'y', 'z')):
            self.get_logger().error('texel_fix: need little-endian float32 x/y/z; cloud left unchanged')
            return data
        buf = bytearray(data)
        points = np.frombuffer(buf, dtype=np.uint8).reshape(count, point_step)
        _, outside = texel_fix_points(points, [names[n].offset for n in ('x', 'y', 'z')], self._ntex)
        if outside and not self._outside_warned:
            self._outside_warned = True
            self.get_logger().warning(
                f'texel_fix: {outside} points outside the +x cube face left unchanged (FOV > 90 deg?)')
        return bytes(buf)

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
