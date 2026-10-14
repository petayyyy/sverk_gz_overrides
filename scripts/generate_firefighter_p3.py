#!/usr/bin/env python3
"""Regenerate Firefighter P3 SDF/ROS/PX4 configs from measured mounts and assets.

Requires numpy; no original CAD application or source exports are required.
Geometry is measured; the complete physics profile remains provisional.
"""
import copy
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from porter_d2_sensors import sub, xml_write, bridge_entries
from collision_profiles import add_airframe_collisions
from generate_porter_d2 import (numbers, rotation, rpy, inertial, box_inertia,
                               visual, collision_box, beam, flight_sensors)

ROOT=Path(__file__).resolve().parents[1]
NAME='firefighter_P3'
MODEL=ROOT/'models'/NAME
CONFIG=MODEL/'config'
ROS=ROOT/'ros_nodes/porter_sensor_adapters/config'


def write_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')


def model_config(name,description):
    root=ET.Element('model')
    sub(root,'name',name);sub(root,'version','0.1')
    sub(root,'sdf','model.sdf',version='1.9')
    sub(root,'description',description)
    xml_write(root,ROOT/'models'/name/'model.config')


def modules(mounts,physics):
    source=json.loads((ROOT/'ros_nodes/porter_sensor_adapters/config/sensors.json').read_text(encoding='utf-8'))
    source['sensors']=[s for s in source['sensors'] if s['topic'] in ('hflow','rangefinder_200m','n6')]
    source.update(topic_prefix='/'+NAME,body_frame=NAME+'/base_link',
                  payload_physics='P3 provisional payload masses; see models/firefighter_P3/config/physics.json',
                  mount_transforms=[])
    names={}
    for cfg in source['sensors']:
        label=cfg['topic'];old=cfg['name'];new=NAME+'_'+old.removeprefix('porter_D2_')
        text=(ROOT/'models'/old/'model.sdf').read_text(encoding='utf-8').replace(old,new).replace('/porter_D2/','/'+NAME+'/')
        text=text.replace('model://'+new+'/meshes/','model://'+old+'/meshes/')
        root=ET.fromstring(text)
        # Camera optical_frame_id supplies the proper frame in native camera
        # headers. Keep Gazebo's extension only for non-camera sensors.
        for sensor in root.findall('model/link/sensor'):
            extension=sensor.find('gz_frame_id')
            if sensor.get('type')=='camera' and extension is not None:sensor.remove(extension)
        xml_write(root,ROOT/'models'/new/'model.sdf')
        model_config(new,'P3 instance of existing '+old+'; original shared mesh and imaging/noise profile.')
        cfg['name']=new;names[label]=(new,new+'_link')
        add_mount(source,cfg,mounts[label])
    # Foxeer reuses the original mesh, native camera profile and FLU correction.
    old='foxeer_t_rex_mini_camera';new=NAME+'_foxeer_camera'
    text=(ROOT/'models'/old/'model.sdf').read_text(encoding='utf-8').replace(old,new)
    text=text.replace('foxeer_t_rex_mini_link',new+'_link').replace('model://'+new+'/meshes/','model://'+old+'/meshes/')
    root=ET.fromstring(text);model=root.find('model');link=model.find('link');sensor=link.find('sensor');camera=sensor.find('camera')
    sensor.find('pose').text='0 0 0 0 0 0'
    sensor.find('pose').set('relative_to',new+'_sensing')
    link.find('inertial/mass').text=str(physics['payload_masses_kg']['foxeer'])
    for tag,value in zip(['ixx','iyy','izz'],np.diag(box_inertia(physics['payload_masses_kg']['foxeer'],[.025,.025,.025]))):
        link.find('inertial/inertia/'+tag).text=f'{value:.15g}'
    sub(sensor,'topic','/'+NAME+'/foxeer/image')
    sub(camera,'camera_info_topic','/'+NAME+'/foxeer/camera_info')
    sub(camera,'optical_frame_id',new+'_optical')
    noise=sub(camera,'noise');sub(noise,'type','gaussian');sub(noise,'mean',0);sub(noise,'stddev',.004)
    # The reused mesh normalization aligns the lens axis but carries a 90 deg
    # roll relative to the image's up direction in this CAD installation.
    # Correct imaging axes only; do not rotate the accurately placed housing.
    for frame_name,parent,pose in [(new+'_sensing',new+'_link','0.0215 0 0 1.5707963267948966 0 0'),
                                   (new+'_optical',new+'_sensing','0 0 0 -1.5707963267948966 0 -1.5707963267948966')]:
        frame=sub(model,'frame',name=frame_name,attached_to=parent);sub(frame,'pose',pose,relative_to=parent)
    xml_write(root,ROOT/'models'/new/'model.sdf');model_config(new,'P3 instance of the existing Foxeer FPV camera; bridge is optional.')
    cfg=dict(name=new,kind='camera',topic='foxeer',origin_mm=[21.5,0,0],
             sensing_quaternion_xyzw=[math.sqrt(.5),0.,0.,math.sqrt(.5)],
             camera=dict(width=640,height=480,fps=30,horizontal_fov_deg=120),
             hardware_assumption='Existing Foxeer T-Rex Mini 1500TVL ideal FPV camera profile; synthetic noise added')
    source['sensors'].append(cfg);add_mount(source,cfg,mounts['foxeer']);names['foxeer']=(new,new+'_link')
    # The standalone Blackfly keeps the supplied conventional part datum.
    cfg=dict(name='bfs_pge_50s5c_camera',kind='camera',topic='blackfly',origin_mm=[86.882,0,0],
             camera=dict(width=2448,height=2048,fps=24,horizontal_fov_deg=60),
             hardware_assumption='Existing Blackfly camera; provisional lens/noise/physics documented in its model')
    source['sensors'].append(cfg);add_mount(source,cfg,mounts['blackfly']);names['blackfly']=('bfs_pge_50s5c_camera','bfs_pge_50s5c_camera_link')
    avia=mounts['livox_avia'];source['mount_transforms'].extend([
        dict(parent=NAME+'/base_link',child=NAME+'_avia',xyz_mm=(np.array(avia['xyz_m'])*1000).tolist(),quaternion_xyzw=quat(np.array(avia['matrix_mm'])[:3,:3])),
        dict(parent=NAME+'_avia',child=NAME+'_avia_imu',xyz_mm=[-41.65,-23.26,28.4],quaternion_xyzw=[0.,0.,0.,1.])])
    names['livox_avia']=('livox_avia','livox_avia_link')
    write_json(ROS/(NAME+'_sensors.json'),source)
    write_json(CONFIG/'sensor_profiles.json',source)
    return source,names


def quat(matrix):
    # Use the same stable transform conversion as the measured placement parser.
    r=matrix
    k=np.array([[r[0,0]-r[1,1]-r[2,2],r[0,1]+r[1,0],r[0,2]+r[2,0],r[2,1]-r[1,2]],
                [r[0,1]+r[1,0],r[1,1]-r[0,0]-r[2,2],r[1,2]+r[2,1],r[0,2]-r[2,0]],
                [r[0,2]+r[2,0],r[1,2]+r[2,1],r[2,2]-r[0,0]-r[1,1],r[1,0]-r[0,1]],
                [r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1],np.trace(r)]])/3
    _,v=np.linalg.eigh(k);q=v[:,-1]
    return (q if q[3]>=0 else -q).tolist()


def add_mount(config,cfg,mount):
    config['mount_transforms'].append(dict(parent=NAME+'/base_link',child=cfg['name']+'_origin',
        xyz_mm=(np.array(mount['xyz_m'])*1000).tolist(),quaternion_xyzw=quat(np.array(mount['matrix_mm'])[:3,:3])))


def rotors(physics):
    output=[]
    for index,(label,spin) in enumerate(zip(physics['motor_order'],physics['turning_directions'])):
        upper=label.startswith('upper');position=np.array(physics['upper_motor_origin_m' if upper else 'lower_motor_origin_m'])
        position[0]=abs(position[0])*(1 if 'front' in label else -1)
        position[1]=abs(position[1])*(-1 if 'right' in label else 1)
        output.append(dict(index=index,vmg=label,position_m=position.tolist(),axis_flu=[0.,0.,1.],
                           turning_direction=spin,sdf_pose=numbers([*position,0,0,0]),
                           visual_pose=numbers([*position,0 if upper else math.pi,0,0])))
    return output


def motor_plugin(model,rotor,physics):
    index=rotor['index']
    plugin=sub(model,'plugin',name='gz::sim::systems::MulticopterMotorModel',filename='gz-sim-multicopter-motor-model-system')
    values=dict(jointName=f'rotor_{index}_joint',linkName=f'rotor_{index}',turningDirection=rotor['turning_direction'],
                timeConstantUp=physics['time_constant_up_s'],timeConstantDown=physics['time_constant_down_s'],
                maxRotVelocity=physics['max_omega_rad_s'],motorConstant=physics['max_thrust_per_rotor_n']/physics['max_omega_rad_s']**2,
                momentConstant=physics['moment_constant_m'],commandSubTopic='command/motor_speed',actuator_number=index,
                rotorDragCoefficient=physics['rotor_drag_coefficient'],rollingMomentCoefficient=physics['rolling_moment_coefficient'],
                rotorVelocitySlowdownSim=physics['rotor_velocity_slowdown_sim'],motorType='velocity')
    for tag,value in values.items():sub(plugin,tag,value)


def component_mass(link,transform):
    item=link.find('inertial');pose=[float(x) for x in item.findtext('pose','0 0 0 0 0 0').split()]
    mass=float(item.findtext('mass'));tensor=item.find('inertia')
    local=np.array([[float(tensor.findtext('ixx')),float(tensor.findtext('ixy','0')),float(tensor.findtext('ixz','0'))],
                    [float(tensor.findtext('ixy','0')),float(tensor.findtext('iyy')),float(tensor.findtext('iyz','0'))],
                    [float(tensor.findtext('ixz','0')),float(tensor.findtext('iyz','0')),float(tensor.findtext('izz'))]])
    matrix=np.array(transform);rotation_matrix=matrix[:3,:3]@rotation(pose[3:])
    centre=matrix[:3,3]/1000+matrix[:3,:3]@pose[:3]
    return mass,centre,rotation_matrix@local@rotation_matrix.T


def vehicle(physics,mounts,names,rotor_list):
    pieces=[]
    for label,(model_name,_) in names.items():
        link=ET.parse(ROOT/'models'/model_name/'model.sdf').find('model/link')
        pieces.append(component_mass(link,mounts[label]['matrix_mm']))
    for rotor in rotor_list:
        pieces.append((physics['rotor_mass_kg'],np.array(rotor['position_m']),np.diag(physics['rotor_inertia_kg_m2'])))
    base_mass=physics['total_mass_kg']-sum(x[0] for x in pieces)
    assert base_mass>0
    base_tensor=box_inertia(base_mass,physics['base_equivalent_box_m']);base_com=np.array(physics['base_com_m'])
    pieces.append((base_mass,base_com,base_tensor));mass=sum(m for m,_,_ in pieces)
    com=sum(m*p for m,p,_ in pieces)/mass;tensor=np.zeros((3,3))
    for m,p,t in pieces:
        delta=p-com;tensor+=t+m*(delta@delta*np.eye(3)-np.outer(delta,delta))
    summary=dict(total_mass_kg=mass,base_mass_kg=base_mass,total_com_flu_m=com.tolist(),
                 total_inertia_at_com_flu_kg_m2=tensor.tolist(),base_inertia_kg_m2=base_tensor.tolist())
    root=ET.Element('sdf',version='1.9');model=sub(root,'model',name=NAME,canonical_link='base_link')
    model.append(ET.Comment(' Original CAD root, +X forward/+Y left/+Z up; PROVISIONAL physics. '))
    sub(model,'static','false');sub(model,'self_collide','false')
    base=sub(model,'link',name='base_link');inertial(base,base_mass,base_com,base_tensor)
    visual(base,'decimated_body','model://'+NAME+'/meshes/body.obj','0 0 0 0 0 0')
    for rotor in rotor_list:
        visual(base,rotor['vmg']+'_motor_housing',
               'model://'+NAME+'/meshes/motor.obj',rotor['visual_pose'],scale='1 1 1')
    add_airframe_collisions(base, CONFIG / 'collision_boxes.json', collision_box)
    flight_sensors(base)
    for rotor in rotor_list:
        i=rotor['index'];link=sub(model,'link',name=f'rotor_{i}');sub(link,'pose',rotor['sdf_pose'],relative_to='base_link')
        inertial(link,physics['rotor_mass_kg'],[0,0,0],np.diag(physics['rotor_inertia_kg_m2']))
        visual(link,rotor['vmg']+'_propeller','model://'+NAME+'/meshes/propeller.obj',
               numbers([0,0,0,0 if rotor['vmg'].startswith('upper') else math.pi,0,0]),scale='1 1 1')
        joint=sub(model,'joint',name=f'rotor_{i}_joint',type='revolute')
        sub(joint,'parent','base_link');sub(joint,'child',f'rotor_{i}')
        axis=sub(joint,'axis');sub(axis,'xyz','0 0 1');limit=sub(axis,'limit');sub(limit,'lower',-1e16);sub(limit,'upper',1e16)
        motor_plugin(model,rotor,physics)
    for label,(model_name,link_name) in names.items():
        instance=label+'_sensor';include=sub(model,'include');sub(include,'uri','model://'+model_name);sub(include,'name',instance)
        mount=mounts[label];sub(include,'pose',numbers([*mount['xyz_m'],*mount['rpy_rad']]),relative_to='base_link')
        joint=sub(model,'joint',name=label+'_mount',type='fixed');sub(joint,'parent','base_link');sub(joint,'child',instance+'::'+link_name)
    odom=sub(model,'plugin',filename='gz-sim-odometry-publisher-system',name='gz::sim::systems::OdometryPublisher')
    for tag,value in dict(odom_frame=NAME+'/odom',robot_base_frame=NAME+'/base_link',dimensions=3,
                          odom_publish_frequency=50,odom_topic='/'+NAME+'/ground_truth/odometry',tf_topic='/'+NAME+'/ground_truth/tf').items():sub(odom,tag,value)
    xml_write(root,MODEL/'model.sdf');model_config(NAME,'Firefighter P3: decimated FLU CAD body, 8 reused VMGs, Livox Avia, H-Flow, rangefinder, N6, Foxeer and Blackfly. Provisional physics.')
    write_json(CONFIG/'derived_physics.json',summary)
    return summary


def bridge_file(path,entries):
    lines=[]
    for entry in entries:
        for index,(key,value) in enumerate(entry.items()):lines.append((' - ' if index==0 else '   ')+key+': '+json.dumps(value))
    path.write_text('\n'.join(lines)+'\n',encoding='utf-8')


def bridges(sensors):
    core=copy.deepcopy(sensors);core['sensors']=[s for s in sensors['sensors'] if s['kind']!='camera']
    entries=bridge_entries(core)
    for ros,gz,rt,gt in [('/avia/raw_points','/livox_avia/nonrepetitive/scan/points','sensor_msgs/msg/PointCloud2','gz.msgs.PointCloudPacked'),
                         ('/avia/raw_imu','/livox_avia/imu','sensor_msgs/msg/Imu','gz.msgs.IMU'),
                         ('/ground_truth/odometry','/'+NAME+'/ground_truth/odometry','nav_msgs/msg/Odometry','gz.msgs.Odometry')]:
        entries.append(dict(ros_topic_name='/'+NAME+ros,gz_topic_name=gz,ros_type_name=rt,gz_type_name=gt,
                            direction='GZ_TO_ROS',qos_profile='DEFAULT' if 'raw_points' in ros else 'SENSOR_DATA'))
    bridge_file(ROS/(NAME+'_bridge.yaml'),entries)
    bridge_file(ROS/(NAME+'_bridge_no_clock.yaml'),[e for e in entries if e['ros_topic_name']!='/clock'])


def px4(physics,rotor_list,summary):
    com=np.array(summary['total_com_flu_m']);frd=np.diag([1,-1,-1])
    params=dict(SIM_GZ_EN=1,SENS_EN_GPSSIM=1,SENS_EN_BAROSIM=0,SENS_EN_MAGSIM=1,
                CA_AIRFRAME=0,CA_ROTOR_COUNT=8,CA_R_REV=0,THR_MDL_FAC=1,
                MPC_THR_HOVER=physics['total_mass_kg']*9.80665/(8*physics['max_thrust_per_rotor_n']),
                EKF2_IMU_POS_X=-com[0],EKF2_IMU_POS_Y=com[1],EKF2_IMU_POS_Z=com[2])
    for rotor in rotor_list:
        i=rotor['index'];pos=frd@(np.array(rotor['position_m'])-com)
        for axis,value in zip('XYZ',pos):params[f'CA_ROTOR{i}_P{axis}']=float(value)
        for axis,value in zip('XYZ',frd@np.array(rotor['axis_flu'])):params[f'CA_ROTOR{i}_A{axis}']=float(value)
        params[f'CA_ROTOR{i}_CT']=1
        params[f'CA_ROTOR{i}_KM']=physics['moment_constant_m']*(1 if rotor['turning_direction']=='ccw' else -1)
        for tag,value in [('FUNC',101+i),('MIN',physics['idle_omega_rad_s']),('MAX',physics['max_omega_rad_s']),('DIS',0)]:params[f'SIM_GZ_EC_{tag}{i+1}']=value
    write_json(CONFIG/'px4_parameters.json',params);write_json(ROS/(NAME+'_px4_parameters.json'),params)
    with (CONFIG/'motor_channels.csv').open('w',newline='',encoding='utf-8') as stream:
        fields=['index','vmg','turning_direction','x_flu_m','y_flu_m','z_flu_m']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for rotor in rotor_list:writer.writerow(dict(index=rotor['index'],vmg=rotor['vmg'],turning_direction=rotor['turning_direction'],**dict(zip(fields[3:],rotor['position_m']))))


def main():
    physics=json.loads((CONFIG/'physics.json').read_text());mounts=json.loads((CONFIG/'mounting_poses.json').read_text(encoding='utf-8'))['mounts']
    sensors,names=modules(mounts,physics);rotor_list=rotors(physics)
    summary=vehicle(physics,mounts,names,rotor_list);bridges(sensors);px4(physics,rotor_list,summary)
    print('Generated firefighter_P3 with 8 VMGs and 6 sensors; total mass',summary['total_mass_kg'])


if __name__=='__main__':main()
