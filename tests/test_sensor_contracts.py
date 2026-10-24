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

# ros_gz_bridge 1.0.24 (Jazzy) knows only these names; any other (e.g. "DEFAULT") makes BridgeConfig log
# "Could not parse entry: Invalid QoS profile" and silently drop the whole entry — the Avia cloud never reached ROS.
ROS_GZ_BRIDGE_QOS={'SYSTEM_DEFAULT','SENSOR_DATA','BEST_AVAILABLE','CLOCK','PARAMETERS','PARAMETER_EVENTS','ROSOUT','SERVICES'}

def test_bridge_qos_profiles_are_valid_and_avia_cloud_reliable():
    import yaml
    paths=sorted((ROOT/'ros_nodes/porter_sensor_adapters/config').glob('*bridge*.yaml'))
    assert paths
    for path in paths:
        for entry in yaml.safe_load(path.read_text()):
            assert entry.get('qos_profile','SYSTEM_DEFAULT') in ROS_GZ_BRIDGE_QOS, (path.name, entry['ros_topic_name'])
            if entry['ros_topic_name'].endswith('/avia/raw_points'):
                # livox_avia_cloud_adapter subscribes with depth 2 (RELIABLE): a BEST_EFFORT publisher never matches
                assert entry['qos_profile']=='SYSTEM_DEFAULT', path.name
    for gen in (ROOT/'scripts').glob('generate_*.py'):
        assert "qos_profile='DEFAULT'" not in gen.read_text(), gen.name
