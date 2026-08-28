from setuptools import setup


package_name = 'livox_avia_cloud_adapter'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/data', ['data/avia_pattern_i16.bin.gz', 'data/SOURCE.md']),
        (f'share/{package_name}/rviz', ['rviz/livox_avia.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='sverk',
    maintainer_email='petayyyy@gmail.com',
    description='Livox Avia scan-pattern adapter for Gazebo GPU lidar clouds.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cloud_adapter = livox_avia_cloud_adapter.cloud_adapter:main',
        ],
    },
)
