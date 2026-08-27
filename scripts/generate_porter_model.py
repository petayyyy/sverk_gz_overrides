#!/usr/bin/env python3
"""Generate the native Gazebo Porter model from p22_description's URDF.

The P-2.2 repository remains the mechanical source of truth. This conversion
removes its embedded uniform-raster Livox and generic IMU before sdformat
lumps fixed links, then adds the production PX4 sensors and reusable Avia
module used by the SVErk simulation.
"""

import argparse
import copy
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET


PX4_SENSORS = """
<sensors>
  <sensor name="air_pressure_sensor" type="air_pressure">
    <always_on>1</always_on><update_rate>50</update_rate>
    <air_pressure><pressure><noise type="gaussian"><mean>0</mean><stddev>3</stddev></noise></pressure></air_pressure>
  </sensor>
  <sensor name="magnetometer_sensor" type="magnetometer">
    <always_on>1</always_on><update_rate>100</update_rate>
    <magnetometer>
      <x><noise type="gaussian"><stddev>0.0001</stddev></noise></x>
      <y><noise type="gaussian"><stddev>0.0001</stddev></noise></y>
      <z><noise type="gaussian"><stddev>0.0001</stddev></noise></z>
    </magnetometer>
  </sensor>
  <sensor name="imu_sensor" type="imu">
    <always_on>1</always_on><update_rate>250</update_rate>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><mean>0</mean><stddev>0.0008726646</stddev></noise></x>
        <y><noise type="gaussian"><mean>0</mean><stddev>0.0008726646</stddev></noise></y>
        <z><noise type="gaussian"><mean>0</mean><stddev>0.0008726646</stddev></noise></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><mean>0</mean><stddev>0.00637</stddev></noise></x>
        <y><noise type="gaussian"><mean>0</mean><stddev>0.00637</stddev></noise></y>
        <z><noise type="gaussian"><mean>0</mean><stddev>0.00686</stddev></noise></z>
      </linear_acceleration>
    </imu>
  </sensor>
  <sensor name="navsat_sensor" type="navsat">
    <always_on>1</always_on><update_rate>30</update_rate>
  </sensor>
</sensors>
"""

MESHES = (
    'base_frame.stl',
    'base_shell.stl',
    'velox2808.stl',
    't9048_cw.stl',
    't9048_ccw.stl',
    'tattu_6s_8000.stl',
)

# `gz sdf -p` retains the visual meshes from the URDF but drops its named
# material definitions. Keep the source P-2.2 palette explicitly so binary
# STL files (which carry no colour information) render as intended.
MATERIALS = {
    'base_frame.stl': '0.13 0.13 0.14 1',
    'base_shell.stl': '0.82 0.25 0.09 1',
    'velox2808.stl': '0.35 0.36 0.38 1',
    't9048_cw.stl': '0.10 0.10 0.11 1',
    't9048_ccw.stl': '0.10 0.10 0.11 1',
    'tattu_6s_8000.stl': '0.15 0.17 0.20 1',
}
CAMERA_MATERIAL = '0.05 0.05 0.05 1'


def remove_embedded_modules(robot):
    for element in list(robot):
        tag = element.tag.rsplit('}', 1)[-1]
        name = element.get('name', '')
        reference = element.get('reference', '')
        if tag == 'link' and name in ('livox_base', 'livox'):
            robot.remove(element)
        elif tag == 'joint' and name in ('livox_base_joint', 'livox_joint'):
            robot.remove(element)
        elif tag == 'gazebo' and reference in ('livox', 'imu_link'):
            robot.remove(element)


def restore_materials(model):
    for visual in model.iter('visual'):
        colour = None
        uri = visual.findtext('geometry/mesh/uri', default='')
        for mesh, candidate in MATERIALS.items():
            if uri.endswith(mesh):
                colour = candidate
                break
        if colour is None and 'camera_' in visual.get('name', ''):
            colour = CAMERA_MATERIAL
        if colour is None:
            continue
        existing = visual.find('material')
        if existing is not None:
            visual.remove(existing)
        visual.append(ET.fromstring(
            f'<material><diffuse>{colour}</diffuse><ambient>{colour}</ambient></material>'
        ))


def fix_lower_motor_visual_orientations(model):
    """Correct two CAD-exported lower motors whose shafts lie sideways."""
    for visual in model.iter('visual'):
        if not visual.findtext('geometry/mesh/uri', default='').endswith('velox2808.stl'):
            continue
        pose = visual.find('pose')
        values = [float(value) for value in pose.text.split()]
        x, y, z = values[:3]
        if z >= -0.02:
            continue
        # The FR and RL lower mounts were exported with a -90° X rotation.
        # Match the vertical (180° X) mounting convention used by their peers.
        if x > 0 and y < 0:
            yaw = -2.530727
        elif x < 0 and y > 0:
            yaw = 0.610865
        else:
            continue
        pose.text = f'{x:.12g} {y:.12g} {z:.12g} 3.14159265359 0 {yaw}'


def generate(p22_dir, output_dir):
    source_urdf = p22_dir / 'urdf' / 'p22.urdf'
    robot_tree = ET.parse(source_urdf)
    remove_embedded_modules(robot_tree.getroot())

    with tempfile.TemporaryDirectory(prefix='porter-sdf-') as temporary:
        prepared_urdf = Path(temporary) / 'porter.urdf'
        robot_tree.write(prepared_urdf, encoding='utf-8', xml_declaration=True)
        result = subprocess.run(
            ['gz', 'sdf', '-p', str(prepared_urdf)],
            check=True, text=True, capture_output=True)

    sdf = ET.fromstring(result.stdout)
    sdf.set('version', '1.9')
    model = sdf.find('model')
    model.set('name', 'porter')
    # Keep the model origin neutral. Its physical ground height is supplied
    # through PX4_GZ_MODEL_POSE by the launch file, before Gazebo creates it.
    model.insert(0, ET.fromstring('<pose>0 0 0 0 0 0</pose>'))
    model.insert(1, ET.fromstring('<self_collide>false</self_collide>'))
    model.insert(2, ET.fromstring('<static>false</static>'))

    for uri in model.iter('uri'):
        uri.text = uri.text.replace('model://p22_description/', 'model://porter/')
    for sensor in model.iter('sensor'):
        frame_id = sensor.find('gz_frame_id')
        if frame_id is not None:
            sensor.remove(frame_id)
        if sensor.get('type') == 'camera':
            sensor.insert(0, ET.fromstring('<always_on>false</always_on>'))

    for plugin in model.findall('plugin'):
        namespace = plugin.find('robotNamespace')
        if namespace is not None:
            plugin.remove(namespace)
        actuator = plugin.find('actuatorNumber')
        if actuator is not None:
            actuator.tag = 'motorNumber'

    restore_materials(model)
    fix_lower_motor_visual_orientations(model)

    base_link = model.find("link[@name='base_link']")
    sensor_group = ET.fromstring(PX4_SENSORS)
    for sensor in list(sensor_group):
        base_link.append(copy.deepcopy(sensor))

    model.append(ET.fromstring("""
      <include merge="true">
        <uri>model://livox_avia</uri>
        <pose relative_to="base_link">-0.021893 -0.003378 -0.112559 0 1.395838 0.012887</pose>
      </include>
    """))
    model.append(ET.fromstring("""
      <joint name="livox_avia_mount_joint" type="fixed">
        <parent>base_link</parent><child>livox_avia_link</child>
      </joint>
    """))

    output_dir.mkdir(parents=True, exist_ok=True)
    meshes_dir = output_dir / 'meshes'
    meshes_dir.mkdir(exist_ok=True)
    for mesh in MESHES:
        shutil.copy2(p22_dir / 'meshes' / mesh, meshes_dir / mesh)

    ET.indent(sdf, space='  ')
    ET.ElementTree(sdf).write(
        output_dir / 'model.sdf', encoding='utf-8', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--p22-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    generate(args.p22_dir.resolve(), args.output_dir.resolve())


if __name__ == '__main__':
    main()
