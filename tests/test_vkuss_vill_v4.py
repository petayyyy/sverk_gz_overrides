"""vkuss_vill_v4 (П-4): generated SDF/ROS/PX4 files reproduce the measured design poses.

Design values (base_link = Cube centre, FLU, metres/radians) are written out literally below so a change of the
inputs or of the generator cannot silently move a sensor: tolerance 0.1 mm / 0.01 deg.
"""
from pathlib import Path
import json
import math
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
NAME = 'vkuss_vill_v4'
MODEL = ROOT/'models'/NAME
ROS = ROOT/'ros_nodes/porter_sensor_adapters/config'
POS_TOL = 1e-4                  # 0.1 mm
ANG_TOL = math.radians(0.01)

LIDAR = ((0.09775, 0.02000, -0.18700), (0.0, 1.3962634, 0.0))
AVIA_IMU = (0.118486, -0.003260, -0.141051)
CAMERA_OPTICAL = ((0.117126, -0.046431, -0.176100), (-2.9554163, -0.0027464, -1.5581843))
CAMERA_GZ_LINK = ((0.117126, -0.046431, -0.176100), (-0.0148362, 1.3845999, -0.0019678))
HFLOW_SENSING = (0.031715, 0.028700, -0.119868)
COM = (0.10275, 0.02000, -0.03450)
ROTORS = {  # index: (FLU thrust point, turning direction)
    0: ((0.233512, -0.145981, -0.000663), 'ccw'), 1: ((-0.028012, 0.185981, -0.000663), 'ccw'),
    2: ((0.233512, 0.185981, -0.000663), 'cw'), 3: ((-0.028012, -0.145981, -0.000663), 'cw'),
    4: ((0.233376, -0.146042, -0.068338), 'cw'), 5: ((-0.027876, 0.186042, -0.068338), 'cw'),
    6: ((0.233377, 0.186041, -0.068338), 'ccw'), 7: ((-0.027878, -0.146041, -0.068338), 'ccw'),
}
# rig_mv133.yaml @ livox-odometry a97d876: p_cam = Rcl p_lidar + Pcl
RCL = np.array([-0.000515, -0.999917, 0.012897, 0.011657, -0.012902, -0.999849, 0.999932, -0.000364, 0.011663]).reshape(3, 3)
PCL = np.array([-0.0667, 0.0202, 0.0071])
# camera_mv133.yaml @ livox-odometry 364a8f9 (OpenCV)
FX, FY, CX, CY = 1528.480242, 1528.218868, 638.7788783, 511.2066348


def rot(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def T(xyz, R):
    m = np.eye(4); m[:3, :3] = R; m[:3, 3] = xyz
    return m


def pose(text):
    v = [float(x) for x in text.split()]
    return T(v[:3], rot(*v[3:]))


def qmat(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def angle(Ra, Rb):
    return math.acos(max(-1.0, min(1.0, (np.trace(Ra.T @ Rb)-1)/2)))


OPT = rot(-math.pi/2, 0, -math.pi/2)  # <name>_sensing -> <name>_optical


@pytest.fixture(scope='module')
def sdf():
    return ET.parse(MODEL/'model.sdf').getroot()


@pytest.fixture(scope='module')
def includes(sdf):
    return {i.findtext('name'): pose(i.findtext('pose')) for i in sdf.iter('include')}


def assert_pose(actual, xyz, R):
    assert np.abs(actual[:3, 3]-np.array(xyz)).max() < POS_TOL, (actual[:3, 3], xyz)
    assert angle(actual[:3, :3], R) < ANG_TOL


def test_flight_controller_is_base_link(sdf):
    base = sdf.find('model/link[@name="base_link"]')
    for name in ('imu_sensor', 'air_pressure_sensor', 'magnetometer_sensor', 'navsat_sensor'):
        sensor = base.find(f'sensor[@name="{name}"]')
        assert sensor is not None and sensor.find('pose') is None, name
    assert sdf.find('model').get('canonical_link') == 'base_link'
    assert sdf.find('model/link[@name="base_link"]/pose') is None


def test_lidar_camera_and_avia_imu_match_design(includes):
    assert_pose(includes['livox_avia_sensor'], LIDAR[0], rot(*LIDAR[1]))
    avia_imu = ET.parse(ROOT/'models/livox_avia/model.sdf').find('.//sensor[@name="imu"]/pose').text
    assert_pose(includes['livox_avia_sensor'] @ pose(avia_imu), AVIA_IMU, rot(*LIDAR[1]))
    assert_pose(includes['mv133_sensor'], *CAMERA_GZ_LINK[:1], rot(*CAMERA_GZ_LINK[1]))
    assert_pose(includes['mv133_sensor'] @ T([0, 0, 0], OPT), CAMERA_OPTICAL[0], rot(*CAMERA_OPTICAL[1]))
    # Level hover: Avia IMU specific force (-0.985, 0, +0.174) g, the real-drone ground check.
    g = includes['livox_avia_sensor'][:3, :3].T @ [0, 0, 1]
    assert np.allclose(g, [-math.cos(math.radians(10)), 0, math.sin(math.radians(10))], atol=1e-6)


def test_lidar_to_camera_reproduces_rig_calibration(includes):
    T_lidar_cam = np.linalg.inv(includes['livox_avia_sensor']) @ includes['mv133_sensor'] @ T([0, 0, 0], OPT)
    Rcl = T_lidar_cam[:3, :3].T
    Pcl = -Rcl @ T_lidar_cam[:3, 3]
    assert np.abs(Rcl-RCL).max() < 1e-6      # rig Rcl is orthonormal to 6e-7; generator uses its SVD projection
    assert np.abs(Pcl-PCL).max() < 1e-6


def test_hflow_mount(includes):
    model = ET.parse(ROOT/'models/vkuss_vill_v4_hflow/model.sdf').getroot()
    sensing = pose(model.find('model/frame[@name="vkuss_vill_v4_hflow_sensing"]/pose').text)
    total = includes['hflow_sensor'] @ sensing
    assert_pose(total, HFLOW_SENSING, rot(0, math.pi/2, 0))  # looking down, image top = nose (SENS_FLOW_ROT 0)
    assert model.find('.//sensor[@type="camera"]/topic').text == '/vkuss_vill_v4/hflow/flow/image'


def test_rotors_and_mass(sdf):
    plugins = {int(p.findtext('actuator_number')): p for p in sdf.iter('plugin') if p.findtext('actuator_number') is not None}
    assert sorted(plugins) == list(range(8))
    total_mass, moment = 0.0, np.zeros(3)
    for link in sdf.find('model').findall('link'):
        origin = pose(link.findtext('pose', '0 0 0 0 0 0'))
        item = link.find('inertial')
        m = float(item.findtext('mass'))
        total_mass += m; moment += m*(origin @ np.r_[pose(item.findtext('pose'))[:3, 3], 1])[:3]
        if link.get('name').startswith('rotor_'):
            i = int(link.get('name')[6:])
            assert np.abs(origin[:3, 3]-np.array(ROTORS[i][0])).max() < POS_TOL
            assert plugins[i].findtext('turningDirection') == ROTORS[i][1]
    for inc in sdf.iter('include'):
        uri = inc.findtext('uri').removeprefix('model://')
        item = ET.parse(ROOT/'models'/uri/'model.sdf').find('model/link/inertial')
        m = float(item.findtext('mass'))
        total_mass += m; moment += m*(pose(inc.findtext('pose')) @ np.r_[pose(item.findtext('pose'))[:3, 3], 1])[:3]
    assert total_mass == pytest.approx(3.0, abs=1e-9)
    assert np.abs(moment/total_mass-np.array(COM)).max() < 1e-6


def test_ros_tf_equals_sdf(includes):
    cfg = json.loads((ROS/f'{NAME}_sensors.json').read_text())
    tf = {(m['parent'], m['child']): T(np.array(m['xyz_mm'])/1000, qmat(m['quaternion_xyzw'])) for m in cfg['mount_transforms']}
    body = NAME+'/base_link'
    assert np.abs(tf[(body, NAME+'_avia')]-includes['livox_avia_sensor']).max() < 1e-9
    assert np.abs(tf[(body, NAME+'_mv133_camera_origin')]-includes['mv133_sensor']).max() < 1e-9
    assert np.abs(tf[(body, NAME+'_hflow_origin')]-includes['hflow_sensor']).max() < 1e-9
    imu = tf[(body, NAME+'_avia')] @ tf[(NAME+'_avia', NAME+'_avia_imu')]
    assert np.abs(imu[:3, 3]-np.array(AVIA_IMU)).max() < POS_TOL
    assert {s['kind'] for s in cfg['sensors']} == {'optical_flow', 'camera'}


def test_camera_intrinsics_rate_and_topics():
    sensor = ET.parse(ROOT/'models/vkuss_vill_v4_mv133_camera/model.sdf').find('.//sensor[@name="imager"]')
    cam = sensor.find('camera')
    intr = {k: float(cam.findtext('lens/intrinsics/'+k)) for k in ('fx', 'fy', 'cx', 'cy')}
    assert intr == pytest.approx(dict(fx=FX, fy=FY, cx=CX+0.5, cy=CY+0.5), abs=1e-7)
    assert float(cam.findtext('horizontal_fov')) == pytest.approx(2*math.atan(1280/(2*FX)), abs=1e-11)
    assert (int(cam.findtext('image/width')), int(cam.findtext('image/height'))) == (1280, 1024)
    assert float(sensor.findtext('update_rate')) == 20
    assert sensor.findtext('topic') == '/vkuss_vill_v4/mv133/image'
    assert cam.findtext('camera_info_topic') == '/vkuss_vill_v4/mv133/camera_info'
    assert sensor.find('pose').get('relative_to') == 'vkuss_vill_v4_mv133_camera_sensing'


def test_px4_parameters_share_one_origin():
    p = json.loads((ROS/f'{NAME}_px4_parameters.json').read_text())
    assert p == json.loads((MODEL/'config/px4_parameters.json').read_text())
    frd = np.diag([1, -1, -1])
    imu = frd @ (np.zeros(3)-COM)
    flow = frd @ (np.array(HFLOW_SENSING)-COM)
    for prefix, expected in (('EKF2_IMU_POS', imu), ('EKF2_EV_POS', imu), ('EKF2_OF_POS', flow), ('EKF2_RNG_POS', flow)):
        assert [p[f'{prefix}_{a}'] for a in 'XYZ'] == pytest.approx(expected, abs=1e-6), prefix
    assert p['CA_ROTOR_COUNT'] == 8 and p['SENS_BOARD_ROT'] == 0
    for i, (xyz, spin) in ROTORS.items():
        assert [p[f'CA_ROTOR{i}_P{a}'] for a in 'XYZ'] == pytest.approx(frd @ (np.array(xyz)-COM), abs=2e-6)
        assert p[f'CA_ROTOR{i}_KM'] == (0.015 if spin == 'ccw' else -0.015)
        assert p[f'SIM_GZ_EC_FUNC{i+1}'] == 101+i


def test_fast_livo_mount_file():
    m = json.loads((ROS/f'{NAME}_fast_livo_mount.json').read_text())
    assert m == json.loads((MODEL/'config/fast_livo_mount.json').read_text())
    assert m['FAST_LIVO_MOUNT_XYZ'] == pytest.approx(AVIA_IMU, abs=1e-6)
    assert m['FAST_LIVO_MOUNT_RPY'] == pytest.approx(LIDAR[1], abs=1e-7)
    assert m['lidar_tf_xyz_rpy'] == pytest.approx([*LIDAR[0], *LIDAR[1]], abs=1e-6)
    assert m['camera_optical_tf_xyz_rpy'] == pytest.approx([*CAMERA_OPTICAL[0], *CAMERA_OPTICAL[1]], abs=1e-6)
    assert m['hflow']['xyz_flu'] == pytest.approx(HFLOW_SENSING, abs=1e-6)
    assert m['hflow']['rpy_flu'] == pytest.approx([0, math.pi/2, 0], abs=1e-9)
    assert m['hflow']['ground_clearance_m'] == pytest.approx(0.355462, abs=1e-6)


def test_bridges():
    for name in (f'{NAME}_bridge.yaml', f'{NAME}_bridge_no_clock.yaml'):
        text = (ROS/name).read_text()
        assert '/vkuss_vill_v4/avia/raw_points' in text and '/vkuss_vill_v4/hflow/flow/image' in text
        assert '/mv133/' not in text           # bridged on demand by launch/vkuss_vill_v4.launch.py
        assert ('"/clock"' in text) == (name.endswith('_bridge.yaml'))


def test_generator_is_reproducible(tmp_path):
    for rel in ('scripts', 'models/vkuss_vill_v4/config', 'models/livox_avia', 'models/porter_D2_hflow'):
        shutil.copytree(ROOT/rel, tmp_path/rel, ignore=shutil.ignore_patterns('meshes', '__pycache__'))
    (tmp_path/'ros_nodes/porter_sensor_adapters/config').mkdir(parents=True)
    shutil.copy(ROS/'sensors.json', tmp_path/'ros_nodes/porter_sensor_adapters/config')
    subprocess.run([sys.executable, '-B', str(tmp_path/'scripts/generate_vkuss_vill_v4.py')], check=True, capture_output=True)
    generated = [p.relative_to(tmp_path) for p in tmp_path.rglob('*') if p.is_file() and NAME in str(p)
                 and 'scripts' not in p.parts and p.name not in ('geometry.json', 'physics.json')]
    assert len(generated) >= 18
    for rel in generated:
        assert (tmp_path/rel).read_bytes() == (ROOT/rel).read_bytes(), rel
