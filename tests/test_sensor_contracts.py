"""Physical grids, optics, and noise units must remain compatible after regeneration."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]

def test_avia_grid_and_reference():
    root=ET.parse(ROOT/'models/livox_avia/model.sdf')
    imu=root.find('.//sensor[@name="imu"]')
    assert float(imu.findtext('update_rate')) == 250
    assert imu.find('imu/orientation_reference_frame') is None
    assert 'relative to initial sensor orientation' in (ROOT/'models/livox_avia/model.sdf').read_text()
    for sensor in root.findall('.//sensor[@type="gpu_lidar"]'):
        for axis in ('horizontal','vertical'):
            limits=sensor.find('ray/scan/'+axis)
            assert max(abs(float(limits.findtext(k))) for k in ('min_angle','max_angle')) < math.pi/4

def test_worlds_match_ekf_gravity():
    for path in (ROOT/'worlds').glob('*.sdf'):
        assert ET.parse(path).findtext('world/gravity') == '0 0 -9.80665', path

def test_magnetic_noise_is_tesla_not_gauss():
    for path in (ROOT/'models').glob('*/model.sdf'):
        for sigma in ET.parse(path).findall('.//magnetometer//noise/stddev'):
            assert float(sigma.text) < 1e-6, path

def test_calibrated_camera_is_separate_and_uses_edge_centres():
    root=ET.parse(ROOT/'models/bfs_pge_50s5c_calibrated/model.sdf')
    cam=root.find('.//camera')
    assert float(cam.findtext('lens/intrinsics/cx'))-.5 == 597.3472
    assert float(cam.findtext('lens/intrinsics/cy'))-.5 == 518.5569
    assert abs(float(cam.findtext('horizontal_fov'))-2*math.atan(612/871.6628))<1e-11
    assert ET.parse(ROOT/'models/bfs_pge_50s5c_camera/model.sdf').find('.//lens') is None
