from setuptools import setup
setup(name='gazebo_camera_tools',version='0.1.0',packages=['gazebo_camera_tools'],
 data_files=[('share/ament_index/resource_index/packages',['resource/gazebo_camera_tools']),('share/gazebo_camera_tools',['package.xml'])],
 entry_points={'console_scripts':['camera_info=gazebo_camera_tools.camera_info:main']})
