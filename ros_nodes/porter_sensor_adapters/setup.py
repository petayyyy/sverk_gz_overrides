from pathlib import Path
from glob import glob
from setuptools import setup, find_packages

HERE = Path(__file__).resolve().parent
setup(
    name='porter_sensor_adapters', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/porter_sensor_adapters']),
        ('share/porter_sensor_adapters', ['package.xml']),
        ('share/porter_sensor_adapters/launch', sorted(glob('launch/*.launch.py'))),
        ('share/porter_sensor_adapters/config', sorted(glob('config/*.json') + glob('config/*.yaml'))),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='sverk', maintainer_email='petayyyy@gmail.com',
    description='Image-based H-Flow, range and mounting TF for Porter D2 and Firefighter P3.', license='Apache-2.0',
    entry_points={'console_scripts': [
        'hflow_adapter = porter_sensor_adapters.nodes:hflow_main',
        'range_adapters = porter_sensor_adapters.nodes:range_main',
        'sensor_frames = porter_sensor_adapters.nodes:frames_main',
        'avia_imu_frame = porter_sensor_adapters.nodes:avia_imu_main',
    ]},
)
