"""Payload/vehicle contracts for the generated P4 configuration."""
import hashlib
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'models/firefighter_P4'


class FirefighterP4Tests(unittest.TestCase):
    def test_camera_topics_and_frames_match_adapter_contract(self):
        config = json.loads((MODEL / 'config/sensor_profiles.json').read_text())
        for profile in config['sensors']:
            if profile['kind'] not in ('camera', 'optical_flow'):
                continue
            with self.subTest(sensor=profile['topic']):
                root = ET.parse(ROOT / 'models' / profile['name'] / 'model.sdf')
                sensor = root.find("model/link/sensor[@type='camera']")
                base = config['topic_prefix'] + '/' + profile['topic']
                if profile['kind'] == 'optical_flow':
                    base += '/flow'
                self.assertEqual(sensor.findtext('topic'), base + '/image')
                self.assertEqual(sensor.findtext('camera/camera_info_topic'), base + '/camera_info')
                self.assertEqual(sensor.findtext('camera/optical_frame_id'), profile['name'] + '_optical')
                self.assertEqual(int(sensor.findtext('camera/image/width')), profile['camera']['width'])
                self.assertEqual(int(sensor.findtext('camera/image/height')), profile['camera']['height'])
                self.assertAlmostEqual(float(sensor.findtext('camera/horizontal_fov')),
                                       math.radians(profile['camera']['horizontal_fov_deg']))

    def test_eight_physical_rotors_and_complete_mass_accounting(self):
        model = ET.parse(MODEL / 'model.sdf').find('model')
        physics = json.loads((MODEL / 'config/physics.json').read_text())
        plugins = model.findall("plugin[@name='gz::sim::systems::MulticopterMotorModel']")
        self.assertEqual(sorted(int(p.findtext('actuator_number')) for p in plugins), list(range(8)))
        self.assertEqual({p.findtext('jointName') for p in plugins}, {f'rotor_{i}_joint' for i in range(8)})
        mass = sum(float(link.findtext('inertial/mass')) for link in model.findall('link'))
        for include in model.findall('include'):
            name = include.findtext('uri').removeprefix('model://')
            nested = ET.parse(ROOT / 'models' / name / 'model.sdf')
            mass += sum(float(link.findtext('inertial/mass')) for link in nested.findall('model/link'))
        self.assertAlmostEqual(mass, physics['total_mass_kg'])
        params = json.loads((MODEL / 'config/px4_parameters.json').read_text())
        self.assertEqual(params['CA_ROTOR_COUNT'], 8)
        self.assertEqual([params[f'SIM_GZ_EC_FUNC{i}'] for i in range(1, 9)], list(range(101, 109)))

    def test_collision_profile_matches_mesh_and_sdf(self):
        profile = json.loads((MODEL / 'config/collision_boxes.json').read_text())
        self.assertEqual(hashlib.sha256((ROOT / profile['body_mesh']).read_bytes()).hexdigest(),
                         profile['body_mesh_sha256'])
        boxes = profile['boxes']
        self.assertEqual(len(boxes), profile['expected_box_count'])
        sdf = ET.parse(MODEL / 'model.sdf')
        collisions = sdf.findall("model/link[@name='base_link']/collision")
        self.assertEqual({b['name'] for b in boxes}, {c.get('name') for c in collisions})
        self.assertEqual(len([b for b in boxes if b['category'] == 'foot']), 4)
        self.assertEqual(len([b for b in boxes if b['category'] == 'leg']), 4)
        self.assertEqual(len([b for b in boxes if b['category'] == 'arm']), 4)
        soles = [b['center_m'][2] - b['size_m'][2] / 2 for b in boxes if b['category'] == 'foot']
        self.assertLess(max(soles) - min(soles), 1e-9)
        for box in boxes:
            self.assertTrue(all(math.isfinite(x) for x in box['center_m'] + box['rpy_rad'] + box['size_m']))
            self.assertGreater(min(box['size_m']), 0)


if __name__ == '__main__':
    unittest.main()
