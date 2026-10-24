#!/usr/bin/env python3
"""Generate the vkuss_vill_v4 (П-4) SDF/ROS/PX4 configs: base_link = Cube flight-controller centre (FLU).

Inputs (P4 CAD model axes + one datum number), all other files are generated:
  models/vkuss_vill_v4/config/geometry.json  Cube centre, Avia window, rig_mv133 Rcl/Pcl, H-Flow placement,
                                              8 hub points, collision boxes, camera intrinsics/rate
  models/vkuss_vill_v4/config/physics.json   PROVISIONAL mass/propulsion (P3 values)
Outputs: models/vkuss_vill_v4/{model.sdf,model.config,README.txt,config/*},
         models/vkuss_vill_v4_mv133_camera, models/vkuss_vill_v4_hflow,
         ros_nodes/porter_sensor_adapters/config/vkuss_vill_v4_*.
Requires numpy. Reuses the P3/D2 helpers unchanged; body visual = П-4 CAD collision boxes; motor/propeller visuals reuse model://firefighter_P3 meshes by URI.
"""
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from porter_d2_sensors import sub, xml_write, bridge_entries
from generate_porter_d2 import numbers, rotation, rpy, inertial, box_inertia, visual, collision_box, flight_sensors
from generate_firefighter_p3 import quat, component_mass, motor_plugin, bridge_file, write_json, model_config

ROOT = Path(__file__).resolve().parents[1]
NAME = 'vkuss_vill_v4'
MODEL = ROOT/'models'/NAME
CONFIG = MODEL/'config'
ROS = ROOT/'ros_nodes/porter_sensor_adapters/config'
CAMERA = NAME+'_mv133_camera'
HFLOW = NAME+'_hflow'
C3 = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)  # FLU = C3 @ model  (x=Zm, y=Xm, z=Ym)
FRD = np.diag([1., -1., -1.])
OPT = rotation([-math.pi/2, 0, -math.pi/2])  # optical (Z fwd, X right, Y down) expressed in native camera axes (+X optical)
# Avia internal IMU in the Avia point-cloud frame = -extrinsic_T of rig_mv133.yaml = model://livox_avia IMU sensor pose.
AVIA_IMU_IN_LIDAR_M = np.array([-0.04165, -0.02326, 0.0284])
MAG_NOISE_TESLA = 1e-7  # same as x500_base after 8d8f4ed (flight_sensors() still writes the old gauss value)


def mount(R, t_m, **extra):
    m = np.eye(4); m[:3, :3] = R; m[:3, 3] = np.asarray(t_m)*1000
    return dict(matrix_mm=m.tolist(), xyz_m=list(map(float, t_m)), rpy_rad=rpy(R), quaternion_xyzw=quat(R), **extra)


def hflow_sensor_config():
    source = json.loads((ROS/'sensors.json').read_text(encoding='utf-8'))
    cfg = copy.deepcopy(next(s for s in source['sensors'] if s['topic'] == 'hflow'))
    cfg['name'] = HFLOW
    return cfg


def derive_mounts(g, hflow_cfg):
    """All poses relative to base_link (Cube centre, FLU), metres; one source -> matrix/xyz/rpy/quat agree."""
    cube = C3 @ np.array(g['cube_centre_model_mm'])
    a = g['avia']
    x = C3 @ np.array(a['window_normal_model'], float); x /= np.linalg.norm(x)
    y = C3 @ np.array(a['native_y_hint_model'], float); y -= x*(x @ y); y /= np.linalg.norm(y)
    R_av = np.column_stack([x, y, np.cross(x, y)])
    t_av = (C3 @ np.array(a['window_centroid_model_mm']) - cube)/1000 + R_av @ np.array(a['origin_offset_native_mm'])/1000
    mounts = dict(livox_avia=mount(R_av, t_av, sensing_direction_flu=x.tolist(),
                                   avia_imu_xyz_m=(t_av + R_av @ AVIA_IMU_IN_LIDAR_M).tolist()))
    c = g['camera']
    if c['pose_source'] == 'calibration':
        Rcl = np.array(c['rig']['Rcl']).reshape(3, 3); U, _, Vt = np.linalg.svd(Rcl); Rcl = U @ Vt
        R_opt = R_av @ Rcl.T; t_opt = t_av + R_av @ (-Rcl.T @ np.array(c['rig']['Pcl']))
    else:  # CAD lens disc + optical-centre depth, axis parallel to the Avia, image x = body right
        t_opt = (C3 @ np.array(c['cad_lens_front_disc_model_mm']) - cube)/1000 - x*c['cad_optical_centre_depth_mm']/1000
        R_opt = np.column_stack([-y, np.cross(x, -y), x])
    mounts['mv133'] = mount(R_opt @ OPT.T, t_opt, optical_rpy_rad=rpy(R_opt), optical_xyz_m=list(map(float, t_opt)),
                            sensing_direction_flu=R_opt[:, 2].tolist())
    # H-Flow: STEP part placement composed with the part->mesh registration of the same STEP product (П-3).
    h = g['hflow']
    T_part = np.array(h['part_to_model_mm'], float)
    R_c2m = np.array(h['cad_to_mesh']['rotation'], float); t_c2m = np.array(h['cad_to_mesh']['translation_mm'], float)
    R_h = C3 @ T_part[:3, :3] @ R_c2m.T
    t_h = (C3 @ T_part[:3, 3] - cube - R_h @ t_c2m)/1000
    sensing = t_h + R_h @ np.array(hflow_cfg['origin_mm'])/1000
    mounts['hflow'] = mount(R_h, t_h, sensing_direction_flu=R_h[:, 0].tolist(), sensing_xyz_m=sensing.tolist(),
                            tof_xyz_m=(t_h + R_h @ np.array(hflow_cfg['range']['origin_mm'])/1000).tolist())
    return mounts, cube


def rotors(physics, g, cube):
    out = []
    for index, (label, spin) in enumerate(zip(physics['motor_order'], physics['turning_directions'])):
        p = (C3 @ np.array(g['motors_model_mm'][physics['motor_ids'][label]]) - cube)/1000
        upper = label.startswith('upper')
        out.append(dict(index=index, vmg=label, cad_id=physics['motor_ids'][label], position_m=p.tolist(), axis_flu=[0., 0., 1.],
                        turning_direction=spin, sdf_pose=numbers([*p, 0, 0, 0]), visual_pose=numbers([*p, 0 if upper else math.pi, 0, 0])))
    return out


def camera_model(g, physics):
    spec = g['camera_spec']; cam = g['camera_intrinsics']; rate = g['camera_rate']['rate_hz']
    root = ET.Element('sdf', version='1.9'); model = sub(root, 'model', name=CAMERA)
    sub(model, 'static', 'false'); sub(model, 'self_collide', 'false')
    model.append(ET.Comment(f' {spec["label"]}; model origin = optical centre, +X optical axis, +Y image-left, +Z image-up. '
                            f'Intrinsics: {cam["source"]}; SDF cx/cy = OpenCV + 0.5 px (gz pixel-edge convention), '
                            f'distortion not rendered. Rate {rate:g} Hz: {g["camera_rate"]["source"]}. '))
    link = sub(model, 'link', name=CAMERA+'_link')
    mass = physics['payload_masses_kg']['mv133']
    inertial(link, mass, [-0.03, 0, 0], box_inertia(mass, [0.06, 0.03, 0.03]))
    sensor = sub(link, 'sensor', name='imager', type='camera')
    sub(sensor, 'pose', '0 0 0 0 0 0', relative_to=CAMERA+'_sensing')
    sub(sensor, 'topic', '/'+NAME+'/mv133/image')
    sub(sensor, 'always_on', 'false'); sub(sensor, 'update_rate', f'{rate:g}'); sub(sensor, 'visualize', 'false')
    camera = sub(sensor, 'camera')
    sub(camera, 'horizontal_fov', f'{2*math.atan(cam["width"]/(2*cam["fx"])):.12f}')
    intr = sub(sub(camera, 'lens'), 'intrinsics')
    for k, v in (('fx', cam['fx']), ('fy', cam['fy']), ('cx', cam['cx']+0.5), ('cy', cam['cy']+0.5)):
        sub(intr, k, f'{v:.7f}')
    sub(intr, 's', 0)
    image = sub(camera, 'image'); sub(image, 'width', cam['width']); sub(image, 'height', cam['height']); sub(image, 'format', 'R8G8B8')
    clip = sub(camera, 'clip'); sub(clip, 'near', 0.05); sub(clip, 'far', 300)
    sub(camera, 'optical_frame_id', CAMERA+'_optical')
    sub(camera, 'camera_info_topic', '/'+NAME+'/mv133/camera_info')
    for frame_name, parent, pose in [(CAMERA+'_sensing', CAMERA+'_link', '0 0 0 0 0 0'),
                                     (CAMERA+'_optical', CAMERA+'_sensing', '0 0 0 -1.5707963267948966 0 -1.5707963267948966')]:
        frame = sub(model, 'frame', name=frame_name, attached_to=parent); sub(frame, 'pose', pose, relative_to=parent)
    xml_write(root, ROOT/'models'/CAMERA/'model.sdf')
    model_config(CAMERA, spec['label']+' for vkuss_vill_v4 (П-4); calibrated pinhole intrinsics, distortion not rendered.')


def hflow_model(g):
    """vkuss_vill_v4 instance of model://porter_D2_hflow (same mesh, imaging and noise profile), as P3 does."""
    old = g['hflow']['mesh_model']
    text = (ROOT/'models'/old/'model.sdf').read_text(encoding='utf-8').replace(old, HFLOW).replace('/porter_D2/', '/'+NAME+'/')
    text = text.replace('model://'+HFLOW+'/meshes/', 'model://'+old+'/meshes/')
    root = ET.fromstring(text)
    for sensor in root.findall('model/link/sensor'):
        extension = sensor.find('gz_frame_id')
        if sensor.get('type') == 'camera' and extension is not None:
            sensor.remove(extension)
    xml_write(root, ROOT/'models'/HFLOW/'model.sdf')
    model_config(HFLOW, 'vkuss_vill_v4 (П-4) instance of existing '+old+'; original shared mesh and imaging/noise profile.')


def sensors_config(g, mounts, hflow_cfg):
    spec = g['camera_spec']; cam = g['camera_intrinsics']; rate = g['camera_rate']['rate_hz']
    cfg = dict(target='Gazebo Harmonic / SDFormat 1.9 / ROS 2 Humble or Jazzy', topic_prefix='/'+NAME,
               sensor_axes='+X sensing, +Y left, +Z up', body_frame=NAME+'/base_link', base_link=g['base_link'],
               payload_physics='П-4 provisional payload masses; see models/vkuss_vill_v4/config/physics.json',
               noise_status='H-Flow noise values are synthetic simulation defaults copied from porter_D2_hflow.',
               sensors=[hflow_cfg,
                        dict(name=CAMERA, kind='camera', topic='mv133', origin_mm=[0, 0, 0],
                             camera=dict(width=cam['width'], height=cam['height'], fps=rate,
                                         horizontal_fov_deg=math.degrees(2*math.atan(cam['width']/(2*cam['fx']))),
                                         fx=cam['fx'], fy=cam['fy'], cx=cam['cx'], cy=cam['cy']),
                             intrinsics_source=cam['source'], hardware_assumption=spec['label'])],
               mount_transforms=[])
    for label, name in (('hflow', HFLOW), ('mv133', CAMERA)):
        m = mounts[label]
        cfg['mount_transforms'].append(dict(parent=NAME+'/base_link', child=name+'_origin',
                                            xyz_mm=(np.array(m['xyz_m'])*1000).tolist(), quaternion_xyzw=m['quaternion_xyzw']))
    a = mounts['livox_avia']
    cfg['mount_transforms'] += [dict(parent=NAME+'/base_link', child=NAME+'_avia', xyz_mm=(np.array(a['xyz_m'])*1000).tolist(),
                                     quaternion_xyzw=a['quaternion_xyzw']),
                                dict(parent=NAME+'_avia', child=NAME+'_avia_imu', xyz_mm=(AVIA_IMU_IN_LIDAR_M*1000).tolist(),
                                     quaternion_xyzw=[0., 0., 0., 1.])]
    write_json(ROS/(NAME+'_sensors.json'), cfg); write_json(CONFIG/'sensor_profiles.json', cfg)
    return cfg


def vehicle(g, physics, mounts, cube, rotor_list):
    names = dict(livox_avia=('livox_avia', 'livox_avia_link'), mv133=(CAMERA, CAMERA+'_link'), hflow=(HFLOW, HFLOW+'_link'))
    pieces = []
    for label, (model_name, _) in names.items():
        link = ET.parse(ROOT/'models'/model_name/'model.sdf').find('model/link')
        pieces.append(component_mass(link, mounts[label]['matrix_mm']))
    for r in rotor_list:
        pieces.append((physics['rotor_mass_kg'], np.array(r['position_m']), np.diag(physics['rotor_inertia_kg_m2'])))
    total = physics['total_mass_kg']; base_mass = total - sum(p[0] for p in pieces); assert base_mass > 0
    target = physics.get('target_total_com_model_mm')
    target = (C3 @ np.array(target) - cube)/1000 if target else np.mean([r['position_m'] for r in rotor_list], axis=0)
    base_com = (total*target - sum(m*p for m, p, _ in pieces))/base_mass
    base_tensor = box_inertia(base_mass, physics['base_equivalent_box_m'])
    pieces.append((base_mass, base_com, base_tensor))
    com = sum(m*p for m, p, _ in pieces)/total; tensor = np.zeros((3, 3))
    for m, p, t in pieces:
        d = p-com; tensor += t + m*(d @ d*np.eye(3)-np.outer(d, d))
    lowest = min(np.array(b['center_m'])[2]-cube[2]/1000-b['size_m'][2]/2 for b in g['collision_boxes'] if b['name'].startswith('foot'))
    summary = dict(total_mass_kg=total, base_mass_kg=base_mass, base_com_m=base_com.tolist(), total_com_flu_m=com.tolist(),
                   total_inertia_at_com_flu_kg_m2=tensor.tolist(), base_inertia_kg_m2=base_tensor.tolist(),
                   cube_centre_model_mm=g['cube_centre_model_mm'], lowest_collision_z_m=float(lowest))
    root = ET.Element('sdf', version='1.9'); model = sub(root, 'model', name=NAME, canonical_link='base_link')
    model.append(ET.Comment(' vkuss_vill_v4 (П-4). base_link = Cube flight-controller centre, +X forward/+Y left/+Z up; '
                            'PX4 imu_sensor at its origin. Sensor poses measured (21.10 STEP + rig_mv133 calibration); '
                            'PROVISIONAL physics; body visual = CAD collision boxes. Generated by scripts/generate_vkuss_vill_v4.py. '))
    sub(model, 'static', 'false'); sub(model, 'self_collide', 'false')
    base = sub(model, 'link', name='base_link'); inertial(base, base_mass, base_com, base_tensor)
    # Body visual = the П-4 CAD collision boxes. The P3 body mesh is NOT used: placed at the П-4 model origin it
    # sits inside the H-Flow view (~29 % of the flow image blocked at 8 cm) and is not П-4 geometry anyway.
    for box in g['collision_boxes']:
        item = sub(base, 'visual', name=box['name']+'_visual')
        sub(item, 'pose', numbers([*(np.array(box['center_m'])-cube/1000), *box['rpy_rad']]))
        sub(sub(sub(item, 'geometry'), 'box'), 'size', numbers(box['size_m']))
        material = sub(item, 'material')
        for tag in ('ambient', 'diffuse'): sub(material, tag, '0.15 0.15 0.15 1')
    for r in rotor_list:
        visual(base, r['vmg']+'_motor_housing', 'model://firefighter_P3/meshes/motor.obj', r['visual_pose'], scale='1 1 1')
    for box in g['collision_boxes']:  # P4 CAD boxes, model-origin FLU -> base_link
        collision_box(base, box['name'], np.array(box['center_m'])-cube/1000, box['size_m'], box['rpy_rad'])
    flight_sensors(base)  # imu/baro/mag/navsat at base_link origin = Cube centre, FLU axes, no <pose>
    for sigma in base.findall('sensor[@type="magnetometer"]//noise/stddev'):
        sigma.text = str(MAG_NOISE_TESLA)
    for r in rotor_list:
        i = r['index']; link = sub(model, 'link', name=f'rotor_{i}'); sub(link, 'pose', r['sdf_pose'], relative_to='base_link')
        inertial(link, physics['rotor_mass_kg'], [0, 0, 0], np.diag(physics['rotor_inertia_kg_m2']))
        visual(link, r['vmg']+'_propeller', 'model://firefighter_P3/meshes/propeller.obj',
               numbers([0, 0, 0, 0 if r['vmg'].startswith('upper') else math.pi, 0, 0]), scale='1 1 1')
        joint = sub(model, 'joint', name=f'rotor_{i}_joint', type='revolute')
        sub(joint, 'parent', 'base_link'); sub(joint, 'child', f'rotor_{i}')
        axis = sub(joint, 'axis'); sub(axis, 'xyz', '0 0 1'); lim = sub(axis, 'limit'); sub(lim, 'lower', -1e16); sub(lim, 'upper', 1e16)
        motor_plugin(model, r, physics)
    for label, (model_name, link_name) in names.items():
        inst = label+'_sensor'; inc = sub(model, 'include'); sub(inc, 'uri', 'model://'+model_name); sub(inc, 'name', inst)
        m = mounts[label]; sub(inc, 'pose', numbers([*m['xyz_m'], *m['rpy_rad']]), relative_to='base_link')
        joint = sub(model, 'joint', name=label+'_mount', type='fixed'); sub(joint, 'parent', 'base_link'); sub(joint, 'child', inst+'::'+link_name)
    odom = sub(model, 'plugin', filename='gz-sim-odometry-publisher-system', name='gz::sim::systems::OdometryPublisher')
    for tag, value in dict(odom_frame=NAME+'/odom', robot_base_frame=NAME+'/base_link', dimensions=3, odom_publish_frequency=50,
                           odom_topic='/'+NAME+'/ground_truth/odometry', tf_topic='/'+NAME+'/ground_truth/tf').items():
        sub(odom, tag, value)
    xml_write(root, MODEL/'model.sdf')
    model_config(NAME, 'vkuss_vill_v4 (П-4): base_link at the Cube FC, 8 coaxial rotors, Livox Avia, MindVision MV-SUA133GC + 6 mm, '
                       'H-Flow. Box body visual, P3 rotor visuals; provisional physics.')
    write_json(CONFIG/'derived_physics.json', summary)
    return summary


def bridges(cfg):
    core = copy.deepcopy(cfg); core['sensors'] = [s for s in cfg['sensors'] if s['kind'] != 'camera']
    entries = bridge_entries(core)  # /clock + H-Flow image/camera_info_gz/ToF points/IMU; MV133 is bridged by the launch file
    for ros, gz, rt, gt in [('/avia/raw_points', '/livox_avia/nonrepetitive/scan/points', 'sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked'),
                            ('/avia/raw_imu', '/livox_avia/imu', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU'),
                            ('/ground_truth/odometry', '/'+NAME+'/ground_truth/odometry', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry')]:
        entries.append(dict(ros_topic_name='/'+NAME+ros, gz_topic_name=gz, ros_type_name=rt, gz_type_name=gt,
                            direction='GZ_TO_ROS', qos_profile='SYSTEM_DEFAULT' if 'raw_points' in ros else 'SENSOR_DATA'))
    bridge_file(ROS/(NAME+'_bridge.yaml'), entries)
    bridge_file(ROS/(NAME+'_bridge_no_clock.yaml'), [e for e in entries if e['ros_topic_name'] != '/clock'])


def px4(physics, rotor_list, summary, mounts):
    com = np.array(summary['total_com_flu_m'])
    imu_pos = FRD @ (np.zeros(3) - com)       # PX4 IMU (base_link origin) relative to the CoM, body FRD
    flow_pos = FRD @ (np.array(mounts['hflow']['sensing_xyz_m']) - com)  # H-Flow flow/ToF optical centre (co-located)
    params = dict(SIM_GZ_FLAT_ALT=1, EKF2_GPS_DELAY=20, EKF2_HGT_REF=0, SIM_GZ_EN=1, SENS_EN_GPSSIM=1, SENS_EN_BAROSIM=0, SENS_EN_MAGSIM=1,
                  CA_AIRFRAME=0, CA_ROTOR_COUNT=8, CA_R_REV=0, THR_MDL_FAC=1, SENS_BOARD_ROT=0,
                  MPC_THR_HOVER=physics['total_mass_kg']*9.80665/(8*physics['max_thrust_per_rotor_n']))
    # All EKF2 sensor positions share one origin (the CoM): EKF2 uses pos_sensor - pos_imu as the lever arm.
    # The FAST-LIVO pose sent to EKF2 is the base_link pose, i.e. the IMU point, so EV_POS must equal IMU_POS.
    for prefix, value in (('EKF2_IMU_POS', imu_pos), ('EKF2_EV_POS', imu_pos), ('EKF2_OF_POS', flow_pos), ('EKF2_RNG_POS', flow_pos)):
        for ax, v in zip('XYZ', value): params[f'{prefix}_{ax}'] = float(v)
    for r in rotor_list:
        i = r['index']; pos = FRD @ (np.array(r['position_m'])-com)
        for ax, v in zip('XYZ', pos): params[f'CA_ROTOR{i}_P{ax}'] = float(v)
        for ax, v in zip('XYZ', FRD @ np.array(r['axis_flu'])): params[f'CA_ROTOR{i}_A{ax}'] = float(v)
        params[f'CA_ROTOR{i}_CT'] = 1
        params[f'CA_ROTOR{i}_KM'] = physics['moment_constant_m']*(1 if r['turning_direction'] == 'ccw' else -1)
        for tag, v in [('FUNC', 101+i), ('MIN', physics['idle_omega_rad_s']), ('MAX', physics['max_omega_rad_s']), ('DIS', 0)]:
            params[f'SIM_GZ_EC_{tag}{i+1}'] = v
    write_json(CONFIG/'px4_parameters.json', params); write_json(ROS/(NAME+'_px4_parameters.json'), params)
    rows = ['index,vmg,cad_id,turning_direction,x_flu_m,y_flu_m,z_flu_m']
    rows += [f"{r['index']},{r['vmg']},{r['cad_id']},{r['turning_direction']},{r['position_m'][0]:.6f},{r['position_m'][1]:.6f},{r['position_m'][2]:.6f}" for r in rotor_list]
    (CONFIG/'motor_channels.csv').write_text('\n'.join(rows)+'\n', encoding='utf-8')
    # FAST-LIVO2 / EKF2 helpers for sverk-ros2 full_system_sitl.launch.py (same formulas as frames/vkuss_vill_v4.yaml).
    a = mounts['livox_avia']; h = mounts['hflow']
    livo = dict(FAST_LIVO_MOUNT_XYZ=np.round(a['avia_imu_xyz_m'], 6).tolist(), FAST_LIVO_MOUNT_RPY=np.round(a['rpy_rad'], 7).tolist(),
                lidar_tf_xyz_rpy=[*np.round(a['xyz_m'], 6).tolist(), *np.round(a['rpy_rad'], 7).tolist()],
                camera_optical_tf_xyz_rpy=[*np.round(mounts['mv133']['optical_xyz_m'], 6).tolist(), *np.round(mounts['mv133']['optical_rpy_rad'], 7).tolist()],
                ekf2_imu_pos_frd=np.round(imu_pos, 6).tolist(),
                hflow=dict(xyz_flu=np.round(h['sensing_xyz_m'], 6).tolist(), rpy_flu=np.round(h['rpy_rad'], 9).tolist(),
                           ground_clearance_m=round(-summary['lowest_collision_z_m'], 6),
                           note='H-Flow flow/ToF optical centre in base_link FLU (calibration.json["hflow"] schema of full_system_sitl); '
                                'ground_clearance_m = base_link height over flat ground at rest (lowest foot collision).'))
    write_json(CONFIG/'fast_livo_mount.json', livo); write_json(ROS/(NAME+'_fast_livo_mount.json'), livo)
    return livo


def readme(g, summary, mounts, livo):
    a, m, h = mounts['livox_avia'], mounts['mv133'], mounts['hflow']
    f = lambda v, n=6: ', '.join(f'{x:.{n}f}' for x in v)
    cam = g['camera_intrinsics']
    text = f"""vkuss_vill_v4 (П-4)
===================

Gazebo Harmonic coaxial X8 with Livox Avia, MindVision MV-SUA133GC + 6 mm and Holybro H-Flow.
Model URI: model://{NAME}. Instance spawned by PX4: {NAME}_0 (sverk-ros2 obrik_config:={NAME}).
Generated by scripts/generate_vkuss_vill_v4.py (numpy only); do not edit generated files.

Datum: base_link = geometric centre of the "Cube контроллер" box in the 21.10 STEP,
model ({f(g['cube_centre_model_mm'], 2)}) mm, +X forward, +Y left, +Z up. PX4's imu_sensor/baro/mag/navsat
sit at the base_link origin with body axes (no <pose>; SENS_BOARD_ROT 0 in SITL).
NOTE: sverk-ros2 frames/vkuss_vill_v4.yaml (real drone) still uses the 17.10 Cube centre
(-22.5, 47, -97.75); relative to it every sensor here is FLU (+5.0, -2.5, 0) mm. Not changed here.

Poses in base_link (m, rad; SDF include pose = sensor model origin):
  livox_frame (Avia window centroid)  xyz ({f(a['xyz_m'])}) rpy ({f(a['rpy_rad'], 7)})
  Avia IMU = FAST_LIVO_MOUNT_XYZ      ({f(livo['FAST_LIVO_MOUNT_XYZ'])}), RPY = livox_frame rpy
  MV133 optical frame                 xyz ({f(m['optical_xyz_m'])}) rpy ({f(m['optical_rpy_rad'], 7)})
    (livox_frame o (Rcl^T, -Rcl^T Pcl), rig_mv133.yaml @ livox-odometry a97d876)
  MV133 gz link (+X optical axis)     xyz ({f(m['xyz_m'])}) rpy ({f(m['rpy_rad'], 7)})
  H-Flow model origin                 xyz ({f(h['xyz_m'])}) rpy ({f(h['rpy_rad'], 7)})
  H-Flow flow/ToF optical centre      ({f(h['sensing_xyz_m'])}), looking down, image top = nose
  CoM (provisional, thrust centroid)  ({f(summary['total_com_flu_m'])}); EKF2_IMU_POS = EKF2_EV_POS = FRD(-CoM)
  lowest foot collision z             {summary['lowest_collision_z_m']:.6f} (spawn z 0.372)

Camera: {cam['width']}x{cam['height']}, fx {cam['fx']} fy {cam['fy']} cx {cam['cx']} cy {cam['cy']} (OpenCV),
  {cam['source']}; SDF cx/cy +0.5 px; {g['camera_rate']['rate_hz']:g} Hz ({g['camera_rate']['source']}).
  Topics /{NAME}/mv133/image and /{NAME}/mv133/camera_info (gz CameraInfo -> *_gz -> gazebo_camera_tools).
H-Flow: model://{HFLOW} = porter_D2_hflow profile (128x128 flow image 100 Hz, 8x4 ToF, IMU), placed from the
  STEP part placement and the П-3 part->mesh registration (median 0.1 mm on П-4 CAD vertices).
Not modelled although present in the CAD: 200 m rangefinder, OpenMV N6, Foxeer FPV (visual-only payloads).

Physics is PROVISIONAL (P3 mass/propulsion, 3.0 kg, CoM at the 8-hub centroid; spin/ESC order P3
convention, the CAD has no CW/CCW). Body visual = the П-4 CAD collision boxes (checked: no box in the Avia,
MV133 or H-Flow view); motor/propeller visuals reuse model://firefighter_P3 meshes at the П-4 hub points.
model://livox_avia's housing mesh renders rolled 90 deg about its optical axis; sensor frames are unaffected.
"""
    (MODEL/'README.txt').write_text(text, encoding='utf-8')


def main():
    g = json.loads((CONFIG/'geometry.json').read_text(encoding='utf-8'))
    physics = json.loads((CONFIG/'physics.json').read_text(encoding='utf-8'))
    hflow_cfg = hflow_sensor_config()
    mounts, cube = derive_mounts(g, hflow_cfg)
    write_json(CONFIG/'mounting_poses.json', dict(body_frame=g['base_link'], cube_centre_model_mm=g['cube_centre_model_mm'], mounts=mounts))
    camera_model(g, physics)
    hflow_model(g)
    cfg = sensors_config(g, mounts, hflow_cfg)
    rotor_list = rotors(physics, g, cube)
    summary = vehicle(g, physics, mounts, cube, rotor_list)
    bridges(cfg); livo = px4(physics, rotor_list, summary, mounts)
    readme(g, summary, mounts, livo)
    print('Generated', NAME, 'base_link=Cube', g['cube_centre_model_mm'], 'total CoM FLU', np.round(summary['total_com_flu_m'], 5))


if __name__ == '__main__':
    main()
