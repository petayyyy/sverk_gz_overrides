#!/usr/bin/env python3
"""Regenerate Porter D2 SDF, ROS configuration and PX4 profile from bundled data.

Requires numpy. Assets are already bundled; no CAD application is required.
The physical profile is deliberately provisional, independently of CAD accuracy.
"""
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from porter_d2_sensors import sub, xml_write, build_model, bridge_entries
from collision_profiles import add_airframe_collisions

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'models/porter_D2'
CONFIG = MODEL / 'config'

def numbers(values):
    return ' '.join(f'{float(v):.15g}' for v in values)

def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])

def rpy(matrix):
    return [math.atan2(matrix[2, 1], matrix[2, 2]),
            math.atan2(-matrix[2, 0], math.hypot(matrix[0, 0], matrix[1, 0])),
            math.atan2(matrix[1, 0], matrix[0, 0])]

def inertial(link, mass, position, tensor):
    item = sub(link, 'inertial')
    sub(item, 'pose', numbers([*position, 0, 0, 0]))
    sub(item, 'mass', mass)
    inertia = sub(item, 'inertia')
    for name, i, j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]:
        sub(inertia, name, f'{tensor[i,j]:.15g}')

def box_inertia(mass, dimensions):
    x, y, z = dimensions
    return np.diag([mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12])

def sensor_physics(physics, name):
    """Synthetic payload mass with bbox centre/inertia; avoids tiny fixed links."""
    path = ROOT/'models'/name.replace('porter_', 'porter_D2_', 1)/'meshes/sensor.obj'
    vertices = np.array([[float(v) for v in line.split()[1:4]] for line in path.read_text(encoding='utf-8').splitlines() if line.startswith('v ')])/1000
    lower,upper = vertices.min(axis=0),vertices.max(axis=0)
    mass = physics['payload_masses_kg'][name]
    return mass,(lower+upper)/2,box_inertia(mass,upper-lower)

def visual(link, name, uri, pose, scale='0.001 0.001 0.001'):
    item = sub(link, 'visual', name=name)
    sub(item, 'pose', pose)
    mesh = sub(sub(item, 'geometry'), 'mesh')
    sub(mesh, 'uri', uri)
    sub(mesh, 'scale', scale)

def collision_box(link, name, xyz, dimensions, angles=(0,0,0)):
    item = sub(link, 'collision', name=name)
    sub(item, 'pose', numbers([*xyz, *angles]))
    sub(sub(sub(item, 'geometry'), 'box'), 'size', numbers(dimensions))
    contact = sub(sub(item, 'surface'), 'friction')
    ode = sub(contact, 'ode')
    sub(ode, 'mu', 0.8)
    sub(ode, 'mu2', 0.8)

def beam(link, name, start, end, thickness):
    start, end = np.array(start), np.array(end)
    vector = end-start
    # Box long axis is local X.
    angles = [0, -math.atan2(vector[2], math.hypot(vector[0], vector[1])), math.atan2(vector[1], vector[0])]
    collision_box(link, name, (start+end)/2, [np.linalg.norm(vector), thickness, thickness], angles)

def model_config(name, description):
    item = ET.Element('model')
    sub(item, 'name', name)
    sub(item, 'version', '0.1')
    sub(item, 'sdf', 'model.sdf', version='1.9')
    sub(item, 'description', description)
    xml_write(item, ROOT / 'models' / name / 'model.config')

def flight_sensors(link):
    # Keep the PX4 bridge's expected sensor names under base_link. The flight
    # IMU is at the body datum with FLU axes, not the unmeasured Pixhawk chip.
    for name, kind, rate in [('imu_sensor','imu',250), ('air_pressure_sensor','air_pressure',50),
                             ('magnetometer_sensor','magnetometer',100), ('navsat_sensor','navsat',30)]:
        sensor = sub(link, 'sensor', name=name, type=kind)
        sub(sensor, 'always_on', 'true')
        sub(sensor, 'update_rate', rate)
        if kind == 'imu':
            imu = sub(sensor, 'imu')
            for group, deviation in [('angular_velocity',.0008726646),('linear_acceleration',.00637)]:
                element = sub(imu, group)
                for axis in 'xyz':
                    noise = sub(sub(element, axis), 'noise', type='gaussian')
                    sub(noise, 'mean', 0)
                    sub(noise, 'stddev', deviation)
        elif kind == 'air_pressure':
            noise = sub(sub(sub(sensor, kind), 'pressure'), 'noise', type='gaussian')
            sub(noise, 'mean', 0)
            sub(noise, 'stddev', 3)
        elif kind == 'magnetometer':
            element = sub(sensor, kind)
            for axis in 'xyz':
                noise = sub(sub(element, axis), 'noise', type='gaussian')
                sub(noise, 'stddev', .0001)

def physics_summary(physics, mounts, rotors):
    avia = ET.parse(ROOT / 'models/livox_avia/model.sdf').find('model/link/inertial')
    avia_mass = float(avia.findtext('mass'))
    avia_pose = [float(x) for x in avia.findtext('pose').split()]
    mount = mounts['livox_avia']['proposed_flu_mount']
    matrix = np.array(mount['matrix_mm'])[:3,:3]
    avia_center = np.array(mount['xyz_m']) + matrix @ np.array(avia_pose[:3])
    avia_rotation = matrix @ rotation(avia_pose[3:])
    ii = avia.find('inertia')
    avia_tensor = np.array([[float(ii.findtext('ixx')), float(ii.findtext('ixy')), float(ii.findtext('ixz'))],
                            [float(ii.findtext('ixy')), float(ii.findtext('iyy')), float(ii.findtext('iyz'))],
                            [float(ii.findtext('ixz')), float(ii.findtext('iyz')), float(ii.findtext('izz'))]])
    module_mass = sum(physics['payload_masses_kg'].values())
    base_mass = physics['total_mass_kg'] - avia_mass - 8*physics['rotor_mass_kg'] - module_mass
    assert base_mass > 0
    center = np.array(physics['base_com_m'])
    tensor = box_inertia(base_mass, physics['base_equivalent_box_m'])
    pieces = [(base_mass, center, tensor), (avia_mass, avia_center, avia_rotation @ avia_tensor @ avia_rotation.T)]
    rotor_tensor = np.diag(physics['rotor_inertia_kg_m2'])
    for rotor in rotors:
        matrix = np.array(rotor['rotation'])
        pieces.append((physics['rotor_mass_kg'], np.array(rotor['position_m']), matrix @ rotor_tensor @ matrix.T))
    for name in ('porter_hflow','porter_rangefinder_200m','porter_n6_camera','porter_mv_ca050_camera'):
        mass, local_com, local_tensor = sensor_physics(physics, name)
        mount = mounts[name]['proposed_flu_mount']
        rotation_matrix = np.array(mount['matrix_mm'])[:3,:3]
        pieces.append((mass, np.array(mount['xyz_m'])+rotation_matrix@local_com, rotation_matrix@local_tensor@rotation_matrix.T))
    mass = sum(p[0] for p in pieces)
    com = sum(m*p for m,p,_ in pieces)/mass
    total_tensor = np.zeros((3,3))
    for m,p,t in pieces:
        d = p-com
        total_tensor += t + m*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
    return dict(total_mass_kg=mass, base_mass_kg=base_mass, total_com_flu_m=com.tolist(),
                total_inertia_at_com_flu_kg_m2=total_tensor.tolist(),
                base_inertia_kg_m2=tensor.tolist(), avia_mass_kg=avia_mass)

def make_rotors(physics, mounts):
    result = []
    for index, (vmg, direction) in enumerate(zip(physics['motor_order'], physics['turning_directions'])):
        mount = mounts[vmg]['proposed_flu_mount']
        matrix = np.array(mount['matrix_mm'])[:3,:3]
        up = matrix[:,1] * np.sign(matrix[2,1])
        x = matrix[:,0]
        physical = np.column_stack([x, np.cross(up,x), up])
        result.append(dict(index=index, vmg=vmg, turning_direction=direction,
                           position_m=mount['xyz_m'], axis_flu=up.tolist(), rotation=physical.tolist(),
                           sdf_pose=numbers([*mount['xyz_m'], *rpy(physical)])))
    return result

def generate_vehicle(physics, mounts, rotors, summary):
    root = ET.Element('sdf', version='1.9')
    model = sub(root, 'model', name='porter_D2', canonical_link='base_link')
    sub(model, 'static', 'false')
    sub(model, 'self_collide', 'false')
    model.append(ET.Comment(' FLU / Z-up. CAD placements are measured; mass, inertia and propulsion are PROVISIONAL. '))
    base = sub(model, 'link', name='base_link')
    inertial(base, summary['base_mass_kg'], physics['base_com_m'], np.array(summary['base_inertia_kg_m2']))
    visual(base, 'decimated_body', 'model://porter_D2/meshes/body.obj', '0 0 0 1.5707963267948966 0 -1.5707963267948966')
    for rotor in rotors:
        visual(base, rotor['vmg']+'_motor_housing', 'model://porter_D2/meshes/motor.obj',
               mounts[rotor['vmg']]['proposed_flu_mount']['sdf_pose_m_rad'])
    # Mesh-fitted simple proxies are independent of the provisional inertia.
    add_airframe_collisions(base, CONFIG / 'collision_boxes.json', collision_box)
    flight_sensors(base)
    for rotor in rotors:
        index = rotor['index']
        link = sub(model, 'link', name=f'rotor_{index}')
        sub(link, 'pose', rotor['sdf_pose'], relative_to='base_link')
        inertial(link, physics['rotor_mass_kg'], [0,0,0], np.diag(physics['rotor_inertia_kg_m2']))
        mesh_rotation = np.array(mounts[rotor['vmg']]['proposed_flu_mount']['matrix_mm'])[:3,:3]
        local_rotation = np.array(rotor['rotation']).T @ mesh_rotation
        visual(link, rotor['vmg']+'_propeller', 'model://porter_D2/meshes/propeller.obj',
               numbers([0,0,0,*rpy(local_rotation)]))
        joint = sub(model, 'joint', name=f'rotor_{index}_joint', type='revolute')
        sub(joint, 'parent', 'base_link')
        sub(joint, 'child', f'rotor_{index}')
        sub(joint, 'pose', '0 0 0 0 0 0', relative_to=f'rotor_{index}')
        axis = sub(joint, 'axis')
        sub(axis, 'xyz', '0 0 1')
        limits = sub(axis, 'limit')
        sub(limits, 'lower', '-inf')
        sub(limits, 'upper', 'inf')
        plugin = sub(model, 'plugin', name='gz::sim::systems::MulticopterMotorModel', filename='gz-sim-multicopter-motor-model-system')
        values = dict(jointName=f'rotor_{index}_joint',linkName=f'rotor_{index}',turningDirection=rotor['turning_direction'],
                      timeConstantUp=physics['time_constant_up_s'],timeConstantDown=physics['time_constant_down_s'],
                      maxRotVelocity=physics['max_omega_rad_s'],motorConstant=physics['max_thrust_per_rotor_n']/physics['max_omega_rad_s']**2,
                      momentConstant=physics['moment_constant_m'],commandSubTopic='command/motor_speed',
                      actuator_number=index,rotorDragCoefficient=physics['rotor_drag_coefficient'],
                      rollingMomentCoefficient=physics['rolling_moment_coefficient'],rotorVelocitySlowdownSim=physics['rotor_velocity_slowdown_sim'],motorType='velocity')
        for tag, value in values.items():
            sub(plugin, tag, value)
    for name in ('porter_hflow','porter_rangefinder_200m','porter_n6_camera','porter_mv_ca050_camera','livox_avia'):
        model_name = name.replace('porter_', 'porter_D2_', 1) if name.startswith('porter_') else name
        link_name = model_name+'_link' if name != 'livox_avia' else 'livox_avia_link'
        include = sub(model, 'include', merge='true')
        sub(include, 'uri', 'model://'+model_name)
        sub(include, 'pose', mounts[name]['proposed_flu_mount']['sdf_pose_m_rad'], relative_to='base_link')
        joint = sub(model, 'joint', name=model_name+'_mount_joint', type='fixed')
        sub(joint, 'parent', 'base_link')
        sub(joint, 'child', link_name)
    plugin = sub(model, 'plugin', name='gz::sim::systems::OdometryPublisher', filename='gz-sim-odometry-publisher-system')
    for tag,value in dict(odom_frame='porter_D2/odom', robot_base_frame='porter_D2/base_link', dimensions=3,
                          odom_publish_frequency=50,odom_topic='/porter_D2/ground_truth/odometry',tf_topic='/porter_D2/ground_truth/tf').items():
        sub(plugin, tag, value)
    xml_write(root, MODEL / 'model.sdf')
    model_config('porter_D2', 'Porter D2: decimated CAD assembly, X8 VMGs, H-Flow, 200 m rangefinder, N6, MV-CA050 and reused Livox Avia. Provisional physics.')

def generate_ros(sensors, mounts, physics):
    sensors['topic_prefix'] = '/porter_D2'
    sensors['body_frame'] = 'porter_D2/base_link'
    sensors['payload_physics'] = 'D2 payload masses and bounding-box inertias are provisional and set from models/porter_D2/config/physics.json; Avia retains its own inertia.'
    sensors['mount_transforms'] = []
    for cfg in sensors['sensors']:
        old_name = cfg['name']
        cfg['name'] = old_name.replace('porter_', 'porter_D2_', 1)
        mount = mounts[old_name]['proposed_flu_mount']
        sensors['mount_transforms'].append(dict(parent=sensors['body_frame'], child=cfg['name']+'_origin',
                                               xyz_mm=mount['xyz_mm'], quaternion_xyzw=mount['quaternion_xyzw']))
        root = build_model(cfg, sensors['topic_prefix'])
        link = root.find('model/link')
        link.remove(link.find('inertial'))
        mass, local_com, local_tensor = sensor_physics(physics, old_name)
        inertial(link, mass, local_com, local_tensor)
        xml_write(root, ROOT / 'models' / cfg['name'] / 'model.sdf')
        model_config(cfg['name'], cfg['hardware_assumption'])
    mount = mounts['livox_avia']['proposed_flu_mount']
    sensors['mount_transforms'] += [dict(parent=sensors['body_frame'],child='porter_D2_avia',xyz_mm=mount['xyz_mm'],quaternion_xyzw=mount['quaternion_xyzw']),
                                    dict(parent='porter_D2_avia',child='porter_D2_avia_imu',xyz_mm=[-41.65,-23.26,28.4],quaternion_xyzw=[0,0,0,1])]
    folder = ROOT / 'ros_nodes/porter_sensor_adapters/config'
    folder.mkdir(parents=True, exist_ok=True)
    (folder/'sensors.json').write_text(json.dumps(sensors, indent=2, ensure_ascii=False)+'\n',encoding='utf-8')
    entries = bridge_entries(sensors)
    for ros,gz,rt,gt in [('/porter_D2/avia/raw_points','/livox_avia/nonrepetitive/scan/points','sensor_msgs/msg/PointCloud2','gz.msgs.PointCloudPacked'),
                         ('/porter_D2/avia/raw_imu','/livox_avia/imu','sensor_msgs/msg/Imu','gz.msgs.IMU'),
                         ('/porter_D2/ground_truth/odometry','/porter_D2/ground_truth/odometry','nav_msgs/msg/Odometry','gz.msgs.Odometry')]:
        # The existing Avia adapter subscribes with reliable QoS (depth 2).
        entries.append(dict(ros_topic_name=ros,gz_topic_name=gz,ros_type_name=rt,gz_type_name=gt,direction='GZ_TO_ROS',qos_profile='DEFAULT' if 'raw_points' in ros else 'SENSOR_DATA'))
    lines = []
    for entry in entries:
        for index,(key,val) in enumerate(entry.items()):
            lines.append((' - ' if index == 0 else '   ')+key+': '+json.dumps(val))
    (folder/'bridge.yaml').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    # The full-system launch already owns /clock; avoid a duplicate publisher.
    without_clock = []
    for entry in entries:
        if entry['ros_topic_name']=='/clock':
            continue
        for index,(key,val) in enumerate(entry.items()):
            without_clock.append((' - ' if index == 0 else '   ')+key+': '+json.dumps(val))
    (folder/'bridge_no_clock.yaml').write_text('\n'.join(without_clock)+'\n',encoding='utf-8')

def generate_px4(physics, rotors, summary):
    # PX4 body FRD is FLU with Y and Z negated. Lever arms are relative to
    # aggregate CoM, not the arbitrary CAD root. KM matches Gazebo's -spin*kQ.
    frd = np.diag([1,-1,-1])
    com = np.array(summary['total_com_flu_m'])
    params = dict(SIM_GZ_FLAT_ALT=1,EKF2_GPS_DELAY=20,EKF2_HGT_REF=0,SIM_GZ_EN=1,SENS_EN_GPSSIM=1,SENS_EN_BAROSIM=0,SENS_EN_MAGSIM=1,
                  CA_AIRFRAME=0,CA_ROTOR_COUNT=8,CA_R_REV=0,THR_MDL_FAC=1,
                  MPC_THR_HOVER=physics['total_mass_kg']*9.80665/(8*physics['max_thrust_per_rotor_n']*rotors[0]['axis_flu'][2]),
                  EKF2_IMU_POS_X=-com[0],EKF2_IMU_POS_Y=com[1],EKF2_IMU_POS_Z=com[2])
    rows = []
    for rotor in rotors:
        i = rotor['index']
        pos = frd@(np.array(rotor['position_m'])-com)
        axis = frd@np.array(rotor['axis_flu'])
        km = physics['moment_constant_m'] * (1 if rotor['turning_direction']=='ccw' else -1)
        for letter,value in zip('XYZ',pos):
            params[f'CA_ROTOR{i}_P{letter}'] = value
        for letter,value in zip('XYZ',axis):
            params[f'CA_ROTOR{i}_A{letter}'] = value
        params[f'CA_ROTOR{i}_CT'] = 1
        params[f'CA_ROTOR{i}_KM'] = km
        params[f'SIM_GZ_EC_FUNC{i+1}'] = 101+i
        params[f'SIM_GZ_EC_MIN{i+1}'] = physics['idle_omega_rad_s']
        params[f'SIM_GZ_EC_MAX{i+1}'] = physics['max_omega_rad_s']
        params[f'SIM_GZ_EC_DIS{i+1}'] = 0
        rows.append(dict(channel=i+1,vmg=rotor['vmg'],spin=rotor['turning_direction'],
                         x_flu_m=rotor['position_m'][0],y_flu_m=rotor['position_m'][1],z_flu_m=rotor['position_m'][2],
                         axis_x_flu=rotor['axis_flu'][0],axis_y_flu=rotor['axis_flu'][1],axis_z_flu=rotor['axis_flu'][2]))
    folder = ROOT / 'px4/porter_D2'
    folder.mkdir(parents=True, exist_ok=True)
    header = '#!/bin/sh\n# @name Gazebo Porter D2 (PROVISIONAL physics)\n# @type Octocopter\n\n. ${R}etc/init.d/rc.mc_defaults\n\nPX4_SIMULATOR=${PX4_SIMULATOR:=gz}\nPX4_GZ_WORLD=${PX4_GZ_WORLD:=default}\nPX4_SIM_MODEL=${PX4_SIM_MODEL:=gz_porter_D2}\n\n'
    body = '\n'.join(f'param set-default {name} {float(value):.12g}' for name,value in params.items())+'\n'
    (folder/'4902_gz_porter_D2').write_text(header+body,encoding='utf-8')
    with (CONFIG/'motor_channels.csv').open('w', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    (CONFIG/'px4_parameters.json').write_text(json.dumps(params, indent=2)+'\n',encoding='utf-8')
    (ROOT/'ros_nodes/porter_sensor_adapters/config/px4_parameters.json').write_text(json.dumps(params,indent=2)+'\n',encoding='utf-8')

def main():
    physics = json.loads((CONFIG/'physics.json').read_text(encoding='utf-8'))
    placements = json.loads((CONFIG/'mounting_poses.json').read_text(encoding='utf-8'))
    mounts = {mount['id']:mount for mount in placements['mounts']}
    sensors = json.loads((CONFIG/'sensor_profiles.json').read_text(encoding='utf-8'))
    rotors = make_rotors(physics, mounts)
    summary = physics_summary(physics, mounts, rotors)
    generate_vehicle(physics, mounts, rotors, summary)
    generate_ros(sensors, mounts, physics)
    generate_px4(physics, rotors, summary)
    (CONFIG/'derived_physics.json').write_text(json.dumps(summary, indent=2)+'\n',encoding='utf-8')
    print('Generated porter_D2, four sensor modules, ROS config and PX4 profile.')

if __name__ == '__main__':
    main()
