"""Read every pixel of a PointCloud2 time-of-flight grid without ROS imports."""
import numpy as np


def point_cloud_ranges(message):
    fields = {field.name: field for field in message.fields}
    if any(name not in fields or fields[name].datatype != 7 or fields[name].count != 1
           for name in ('x', 'y', 'z')):
        raise ValueError('ToF cloud needs scalar float32 XYZ fields')
    width, height, step, row = map(int, (message.width, message.height,
                                       message.point_step, message.row_step))
    offsets = [int(fields[name].offset) for name in ('x', 'y', 'z')]
    if (width < 1 or height < 1 or step < 12 or row < width*step
            or len(message.data) != height*row or any(offset < 0 or offset+4 > step for offset in offsets)):
        raise ValueError('Invalid ToF cloud layout')
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=[endian+'f4']*3,
                          offsets=offsets, itemsize=step))
    records = np.ndarray((height, width), dtype=dtype, buffer=message.data,
                         strides=(row, step))
    xyz = np.stack([records[name] for name in ('x', 'y', 'z')], axis=-1).reshape(-1, 3)
    finite = np.isfinite(xyz).all(axis=1)
    ranges = np.full(len(xyz), np.nan)
    ranges[finite] = np.linalg.norm(xyz[finite].astype(float), axis=1)
    return ranges
