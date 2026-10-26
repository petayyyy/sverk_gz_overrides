"""Install measured box primitives without coupling collision shape to inertia."""
import hashlib
import json
from pathlib import Path
import numpy as np


def add_airframe_collisions(link, path, collision_box):
    profile = json.loads(Path(path).read_text(encoding='utf-8'))
    root = Path(path).resolve().parents[3]
    mesh = root / profile['body_mesh']
    if hashlib.sha256(mesh.read_bytes()).hexdigest() != profile['body_mesh_sha256']:
        raise ValueError('Collision fit must be reviewed after changing ' + str(mesh))
    boxes = profile['boxes']
    expected_count = profile.get('expected_box_count', 13)
    if (not isinstance(expected_count, int) or expected_count <= 0 or
            len(boxes) != expected_count or len({box['name'] for box in boxes}) != expected_count):
        raise ValueError(f'Expected {expected_count} uniquely named airframe boxes')
    for box in boxes:
        values = box['center_m'] + box['size_m'] + box['rpy_rad']
        if len(values) != 9 or not np.isfinite(values).all() or min(box['size_m']) <= 0:
            raise ValueError('Invalid collision box: ' + box['name'])
        collision_box(link, box['name'], box['center_m'], box['size_m'], box['rpy_rad'])
