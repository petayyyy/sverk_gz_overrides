"""Проверка того же копирования ROS-пакета, которое выполняет сборка SITL."""

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = 'livox_avia_cloud_adapter'


class OverrideInstallationTests(unittest.TestCase):
    def test_installed_adapter_replaces_stale_copy_and_passes_regressions(self):
        with tempfile.TemporaryDirectory(prefix='gazebo overrides ') as directory:
            root = Path(directory)
            source = root / 'source'
            (source / 'models').mkdir(parents=True)
            (source / 'worlds').mkdir()
            (source / 'worlds/test.sdf').write_text('<sdf version="1.9"><world name="test"/></sdf>')
            shutil.copytree(ROOT / 'ros_nodes' / PACKAGE, source / 'ros_nodes' / PACKAGE)

            workspace = root / 'workspace/simulation'
            installed = workspace / PACKAGE
            (installed / PACKAGE).mkdir(parents=True)
            (installed / PACKAGE / 'cloud_adapter.py').write_text('# Old adapter without texel_fix\n')
            (installed / 'obsolete.py').write_text('# Must not survive replacement\n')

            subprocess.run([
                'bash', str(ROOT / 'scripts/update_overrides.sh'),
                '--src', str(source), '--px4-dir', str(root / 'px4'),
                '--ros-nodes-dir', str(workspace),
            ], check=True, capture_output=True, text=True)

            self.assertFalse((installed / 'obsolete.py').exists())
            self.assertEqual(
                (installed / PACKAGE / 'cloud_adapter.py').read_bytes(),
                (source / 'ros_nodes' / PACKAGE / PACKAGE / 'cloud_adapter.py').read_bytes(),
            )
            environment = dict(os.environ)
            environment['PYTHONPATH'] = str(installed) + os.pathsep + environment.get('PYTHONPATH', '')
            environment['PYTHONDONTWRITEBYTECODE'] = '1'
            environment['PYTEST_DISABLE_PLUGIN_AUTOLOAD'] = '1'
            result = subprocess.run([
                sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
                str(installed / 'test/test_texel_fix.py'),
            ], env=environment, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
