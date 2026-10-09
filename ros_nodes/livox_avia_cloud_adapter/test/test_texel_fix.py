"""Исправление текселя в адаптере: совпадает с формулой, проверенной в replay X1, и не трогает лишнего."""
import numpy as np
import pytest
from types import SimpleNamespace
from unittest.mock import Mock

from sensor_msgs.msg import PointCloud2, PointField

from livox_avia_cloud_adapter.cloud_adapter import (
    LivoxAviaCloudAdapter, PATTERN_POINTS, POINTS_PER_FRAME,
    cube_face_texels, texel_fix, texel_fix_points,
)

H, NH = 0.614355897, 352
V, NV = 0.673697091, 386


def grid_cloud(rng, n=5000, point_step=32):
    """Точки gz на лучах сетки 352×386 (как GpuLidarSensor::FillPointCloudMsg), поля x,y,z,intensity."""
    j = rng.integers(0, NH, n); i = rng.integers(0, NV, n)
    h = -H + j * 2 * H / (NH - 1); v = -V + i * 2 * V / (NV - 1)
    r = rng.uniform(5, 190, n)
    d = np.stack([np.cos(v) * np.cos(h), np.cos(v) * np.sin(h), np.sin(v)], 1)
    xyz = (r[:, None] * d).astype(np.float32)
    raw = np.zeros((n, point_step), np.uint8)
    for k, o in enumerate((0, 4, 8)):
        raw[:, o:o + 4] = np.ascontiguousarray(xyz[:, k]).view(np.uint8).reshape(-1, 4)
    raw[:, 16:20] = np.full(n, 7.0, np.float32).view(np.uint8).reshape(-1, 4)   # intensity
    return raw, xyz


def test_face_size():
    assert cube_face_texels(352, 386) == 512
    assert cube_face_texels(600, 40) == 1024
    assert cube_face_texels(10, 10) == 128
    assert cube_face_texels(2000, 10) == 1024


def test_preserves_range_and_other_fields():
    rng = np.random.default_rng(1)
    raw, xyz = grid_cloud(rng)
    before = raw.copy()
    fixed, outside = texel_fix_points(raw, [0, 4, 8], 512)
    assert (fixed, outside) == (len(xyz), 0)
    out = np.stack([np.ascontiguousarray(raw[:, o:o + 4]).view('<f4')[:, 0] for o in (0, 4, 8)], 1)
    np.testing.assert_array_equal(out, texel_fix(xyz, 512).astype(np.float32))
    assert np.array_equal(raw[:, 12:], before[:, 12:])                        # прочие поля не тронуты
    r0 = np.linalg.norm(xyz.astype(np.float64), axis=1); r1 = np.linalg.norm(out.astype(np.float64), axis=1)
    assert np.abs(r1 - r0).max() < 3e-5                                       # дальность сохранена
    ang = np.arccos(np.clip(np.einsum('ij,ij->i', xyz / r0[:, None], out / r1[:, None]), -1, 1))
    assert ang.max() < 3.0e-3 and 0.8e-3 < np.median(ang) < 2.0e-3            # ≤ полтекселя, медиана ~1.4 мрад
    # Исправленные направления проходят через центры, не края текселей.
    uv = (0.5 - 0.5 * out[:, 1:3].astype(np.float64) / out[:, 0:1]) * 512
    np.testing.assert_allclose(uv - np.floor(uv), 0.5, atol=3e-5)


def test_nonfinite_and_outside_face_untouched():
    raw, _ = grid_cloud(np.random.default_rng(2), n=4)
    bad = np.array([[np.inf, 0, 0], [np.nan, 1, 1], [1.0, 2.0, 0.0], [-3.0, 0.1, 0.1]], np.float32)
    for k, o in enumerate((0, 4, 8)):
        raw[:, o:o + 4] = np.ascontiguousarray(bad[:, k]).view(np.uint8).reshape(-1, 4)
    before = raw.copy()
    fixed, outside = texel_fix_points(raw, [0, 4, 8], 512)
    assert fixed == 0 and outside == 1                                        # (1, 2, 0): |y/x| > 1
    assert np.array_equal(raw, before)


@pytest.mark.parametrize('mode', ['repetitive', 'nonrepetitive'])
def test_callback_corrects_both_scan_modes(mode):
    # Используем настоящий callback и настоящую выборку шаблона без DDS/узла.
    adapter = LivoxAviaCloudAdapter.__new__(LivoxAviaCloudAdapter)
    adapter._mode = mode
    adapter._frame_id = 'livox_avia'
    adapter._texel_fix = True
    adapter._ntex_param = 0
    adapter._ntex = None
    adapter._outside_warned = False
    adapter._pattern = np.zeros((PATTERN_POINTS, 2), dtype=np.int16)
    adapter._source_shape = None
    adapter._source_indices = None
    adapter._phase = 0
    adapter.get_logger = lambda: Mock()
    published = []
    adapter._publisher = SimpleNamespace(publish=published.append)

    message = PointCloud2()
    message.width, message.height, message.point_step = NH, NV, 32
    message.row_step = message.width * message.point_step
    message.fields = [PointField(name=name, offset=offset,
                                 datatype=PointField.FLOAT32, count=1)
                      for name, offset in [('x', 0), ('y', 4), ('z', 8), ('intensity', 16)]]
    message.header.stamp.sec = 12
    raw = np.zeros((NH * NV, 32), dtype=np.uint8)
    xyz = np.array([[100, 30, -20]], dtype=np.float32)
    for k, offset in enumerate((0, 4, 8)):
        raw[:, offset:offset + 4] = xyz[:, k].copy().view(np.uint8).reshape(1, 4)
    raw[:, 16:20] = np.array([7], dtype=np.float32).view(np.uint8)
    message.data = raw.tobytes()

    adapter._cloud_callback(message)

    assert len(published) == 1
    output = published[0]
    assert output.header.stamp == message.header.stamp
    assert output.header.frame_id == 'livox_avia'
    assert output.width == (NH * NV if mode == 'repetitive' else POINTS_PER_FRAME)
    assert adapter._ntex == 512
    points = np.frombuffer(output.data, dtype=np.uint8).reshape(output.width, 32)
    out = np.stack([np.ascontiguousarray(points[:, o:o + 4]).view('<f4')[:, 0]
                    for o in (0, 4, 8)], axis=1)
    expected = np.broadcast_to(texel_fix(xyz, 512).astype(np.float32), out.shape)
    np.testing.assert_array_equal(out, expected)
    np.testing.assert_array_equal(points[:, 12:], np.broadcast_to(raw[0, 12:], points[:, 12:].shape))
