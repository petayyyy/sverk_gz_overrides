#!/usr/bin/env python3
"""Regenerate Firefighter P4 SDF/ROS/PX4 configs from measured mounts and assets.

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
NAME='firefighter_P4'
MODEL=ROOT/'models'/NAME
CONFIG=MODEL/'config'
ROS=ROOT/'ros_nodes/porter_sensor_adapters/config'


def write_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n',encoding='utf-8',newline='\n')


def model_config(name,description):
    root=ET.Element('model')
    sub(root,'name',name);sub(root,'version','0.1')
    sub(root,'sdf','model.sdf',version='1.9')
    sub(root,'description',description)
    xml_write(root,ROOT/'models'/name/'model.config')


def modules(mounts,physics):
    """Reuse P3 sensor behaviour with the corrected, normalized P4 CAD meshes."""
    source=json.loads((ROS/'firefighter_P3_sensors.json').read_text(encoding='utf-8'))
    source=json.loads(json.dumps(source).replace('firefighter_P3','firefighter_P4').replace('bfs_pge_50s5c_camera','firefighter_P4_blackfly_camera'))
    source.update(mesh_units='metres',topic_prefix='/'+NAME,body_frame=NAME+'/base_link',
                  payload_physics='Provisional P4 payload masses; see models/firefighter_P4/config/physics.json',mount_transforms=[])
    names={}
    originals={'hflow':'firefighter_P3_hflow','rangefinder_200m':'firefighter_P3_rangefinder_200m',
               'n6':'firefighter_P3_n6_camera','foxeer':'firefighter_P3_foxeer_camera','blackfly':'bfs_pge_50s5c_camera'}
    configs={s['topic']:s for s in source['sensors']}
    for label in [*originals,'livox_avia']:
        old=originals.get(label,'livox_avia')
        new=configs[label]['name'] if label in configs else NAME+'_livox_avia'
        root=ET.parse(ROOT/'models'/old/'model.sdf').getroot()
        text=ET.tostring(root,encoding='unicode').replace(old,new).replace('/firefighter_P3/','/'+NAME+'/')
        root=ET.fromstring(text);model=root.find('model');model.set('name',new);link=model.find('link');link.set('name',new+'_link')
        if label=='livox_avia':
            # Keep the existing Avia native topics and IMU offset. Its ROS adapter
            # selects/reshapes the scan and publishes P4-specific frames/topics.
            for e in root.iter():
                if e.text:e.text=e.text.replace('/'+new+'/', '/livox_avia/')
        meshpath=ROOT/'models'/new/'meshes/sensor.obj'
        points=np.array([[float(v) for v in line.split()[1:4]] for line in meshpath.read_text(encoding='utf-8').splitlines() if line.startswith('v ')])
        lower,upper=points.min(0),points.max(0)
        for e in list(link.findall('visual'))+list(link.findall('collision'))+list(link.findall('inertial')):link.remove(e)
        mass=physics['payload_masses_kg'][label]
        inertial(link,mass,(lower+upper)/2,box_inertia(mass,upper-lower))
        visual(link,'housing_visual','model://'+new+'/meshes/sensor.obj','0 0 0 0 0 0',scale='1 1 1')
        # A CAD-axis box tightly encloses the housing without inflating it to a
        # sensor-axis AABB. All offsets and rotations remain in the link frame.
        reg=np.array(mounts[label]['cad_to_sensor_matrix_mm']);reg[:3,3]/=1000
        native=(points-reg[:3,3])@reg[:3,:3]
        lo,hi=native.min(0),native.max(0);center=reg[:3,:3]@((lo+hi)/2)+reg[:3,3]
        collision_box(link,'housing_collision',center,hi-lo+.001,rpy(reg[:3,:3]))
        for sensor in link.findall('sensor'):
            ext=sensor.find('gz_frame_id')
            if sensor.get('type')=='camera':
                if ext is not None:sensor.remove(ext)
                topic=sensor.find('topic')
                if topic is None:topic=sub(sensor,'topic')
                camera_topic='/'+NAME+'/'+label+('/flow' if label=='hflow' else '')
                topic.text=camera_topic+'/image'
                camera=sensor.find('camera');info=camera.find('camera_info_topic')
                if info is None:info=sub(camera,'camera_info_topic')
                info.text=camera_topic+'/camera_info'
                camera.find('optical_frame_id').text=new+'_optical'
        xml_write(root,ROOT/'models'/new/'model.sdf')
        model_config(new,'Firefighter P4 '+label+'; existing sensor profile, measured CAD placement, provisional noise and mass.')
        names[label]=(new,new+'_link')
        if label in configs:
            cfg=configs[label];cfg['mesh_source']='Updated P4 STEP; normalized bundled sensor.obj (metres)'
            add_mount(source,cfg,mounts[label])
    avia=mounts['livox_avia'];source['mount_transforms'].extend([
        dict(parent=NAME+'/base_link',child=NAME+'_avia',xyz_mm=(np.array(avia['xyz_m'])*1000).tolist(),quaternion_xyzw=quat(np.array(avia['matrix_mm'])[:3,:3])),
        dict(parent=NAME+'_avia',child=NAME+'_avia_imu',xyz_mm=[-41.65,-23.26,28.4],quaternion_xyzw=[0.,0.,0.,1.])])
    write_json(ROS/(NAME+'_sensors.json'),source);write_json(CONFIG/'sensor_profiles.json',source)
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
    visual(base,'decimated_body','model://'+NAME+'/meshes/body.obj','0 0 0 0 0 0',scale='1 1 1')
    for rotor in rotor_list:
        visual(base,rotor['vmg']+'_motor_housing',
               'model://'+NAME+'/meshes/motor.obj',rotor['visual_pose'],scale='1 1 1')
    add_airframe_collisions(base, CONFIG / 'collision_boxes.json', collision_box)
    flight_sensors(base)
    # Gazebo magnetic fields and their noise use tesla; 0.0001 gauss = 1e-8 T.
    for sigma in base.findall('.//magnetometer//noise/stddev'):
        sigma.text='1e-08'
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
    xml_write(root,MODEL/'model.sdf');model_config(NAME,'Firefighter P4: decimated FLU CAD body, 8 reused VMGs, Livox Avia, H-Flow, rangefinder, N6, Foxeer and corrected Blackfly. Provisional physics.')
    write_json(CONFIG/'derived_physics.json',summary)
    return summary


def bridge_file(path,entries):
    lines=[]
    for entry in entries:
        for index,(key,value) in enumerate(entry.items()):lines.append((' - ' if index==0 else '   ')+key+': '+json.dumps(value))
    path.write_text('\n'.join(lines)+'\n',encoding='utf-8',newline='\n')


def bridges(sensors):
    core=copy.deepcopy(sensors);core['sensors']=[s for s in sensors['sensors'] if s['kind']!='camera']
    entries=bridge_entries(core)
    for ros,gz,rt,gt in [('/avia/raw_points','/livox_avia/nonrepetitive/scan/points','sensor_msgs/msg/PointCloud2','gz.msgs.PointCloudPacked'),
                         ('/avia/raw_imu','/livox_avia/imu','sensor_msgs/msg/Imu','gz.msgs.IMU'),
                         ('/ground_truth/odometry','/'+NAME+'/ground_truth/odometry','nav_msgs/msg/Odometry','gz.msgs.Odometry')]:
        entries.append(dict(ros_topic_name='/'+NAME+ros,gz_topic_name=gz,ros_type_name=rt,gz_type_name=gt,
                            direction='GZ_TO_ROS',qos_profile='SYSTEM_DEFAULT' if 'raw_points' in ros else 'SENSOR_DATA'))
    bridge_file(ROS/(NAME+'_bridge.yaml'),entries)
    bridge_file(ROS/(NAME+'_bridge_no_clock.yaml'),[e for e in entries if e['ros_topic_name']!='/clock'])


def px4(physics,rotor_list,summary):
    com=np.array(summary['total_com_flu_m']);frd=np.diag([1,-1,-1])
    params=dict(SIM_GZ_FLAT_ALT=1,EKF2_GPS_DELAY=20,EKF2_HGT_REF=0,SIM_GZ_EN=1,SENS_EN_GPSSIM=1,SENS_EN_BAROSIM=0,SENS_EN_MAGSIM=1,
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
        writer=csv.DictWriter(stream,fieldnames=fields,lineterminator='\n');writer.writeheader()
        for rotor in rotor_list:writer.writerow(dict(index=rotor['index'],vmg=rotor['vmg'],turning_direction=rotor['turning_direction'],**dict(zip(fields[3:],rotor['position_m']))))


def main():
    physics=json.loads((CONFIG/'physics.json').read_text());mounts=json.loads((CONFIG/'mounting_poses.json').read_text(encoding='utf-8'))['mounts']
    sensors,names=modules(mounts,physics);rotor_list=rotors(physics)
    summary=vehicle(physics,mounts,names,rotor_list);bridges(sensors);px4(physics,rotor_list,summary)
    print('Generated firefighter_P4 with 8 VMGs and 6 sensors; total mass',summary['total_mass_kg'])


if __name__=='__main__':main()
