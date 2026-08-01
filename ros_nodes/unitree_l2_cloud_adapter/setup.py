from setuptools import setup


package_name = 'unitree_l2_cloud_adapter'


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sverk',
    maintainer_email='petayyyy@gmail.com',
    description='Equal-area point-cloud decimator for the Unitree L2 Gazebo GPU lidar.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'equal_area_cloud_adapter = '
            'unitree_l2_cloud_adapter.equal_area_cloud_adapter:main',
        ],
    },
)
