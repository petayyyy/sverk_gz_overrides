Porter D2
=========

Gazebo Harmonic X8 configuration with five payload sensors. Model URI:
model://porter_D2. Body axes are +X forward, +Y left, +Z up; the body datum
is the CAD origin. The decimated body retains its source millimetre units
and is scaled and rotated in SDF. Motor and propeller meshes retain the original VMG datum.

H-Flow, the rangefinder, N6 and MV-CA050 look downward. Livox Avia points
approximately 10 degrees forward from downward. Sensor mounting transforms
are in config/mounting_poses.json; each module applies its sensing offset
once. Camera optical frames use +Z forward, +X right, +Y down.

Run
---
Install models and ROS packages with scripts/update_overrides.sh. In the ROS
workspace, build the dependencies and source the result:

  colcon build --packages-up-to porter_sensor_adapters livox_avia_cloud_adapter
  source install/setup.bash

Tested runtime: Gazebo Harmonic 8.15, ROS 2 Humble and PX4 v1.15.

In sverk-ros2, select the configuration with an existing world. PX4 spawns
the vehicle into that world:

  bash scripts/sitl/sim.sh launch --display headless --config porter_D2 --world obrik_aruco

This requires the D2 selector support in sverk-ros2. It applies
config/px4_parameters.json to the existing eight-motor PX4 airframe.
For a separate PX4 installation, scripts/install_porter_d2_airframe.py
installs the optional 4902 airframe; rebuild PX4 after installing it.

To start only the ROS payload bridges for an already spawned vehicle:

  ros2 launch porter_sensor_adapters porter_D2.launch.py

ROS outputs
-----------
Topics are under /porter_D2:

  hflow/optical_flow         porter_sensor_msgs/OpticalFlow
  hflow/tof/scan/points      sensor_msgs/PointCloud2 (8x4 ToF grid)
  hflow/tof/range            sensor_msgs/Range
  hflow/imu                  sensor_msgs/Imu
  rangefinder_200m/range     sensor_msgs/Range
  n6/image                  sensor_msgs/Image
  n6/camera_info            sensor_msgs/CameraInfo
  mv_ca050/image            sensor_msgs/Image
  mv_ca050/camera_info      sensor_msgs/CameraInfo
  avia/points               sensor_msgs/PointCloud2
  avia/imu                  sensor_msgs/Imu
  ground_truth/odometry     nav_msgs/Odometry

Static TF mounts root at porter_D2/base_link. Localization must provide its
world transform. Payload flow/range data are not injected into PX4's EKF.
The ToF Range is the median of valid rays from the full grid and supplies
distance to the optical-flow adapter.
The Avia module retains global Gazebo topics /livox_avia/...; this setup
supports one Avia per world. The Avia GPU-ray texel correction is enabled by default; use
avia_texel_fix:=false to disable it.

Parameters
----------
config/physics.json sets a provisional 18 kg total mass, box inertias and
motor response: 45 N per motor at 600 rad/s. Replace these with measured
mass, inertia and propulsion data before comparing real flight dynamics.
Motor housings stay fixed; propellers rotate on the eight thrust-producing
rotor joints. rotor_velocity_slowdown_sim=10 slows visible rotation, as in
the existing configs; thrust uses the configured motor speed. Coaxial wake
interference is not simulated.

config/collision_boxes.json defines 13 fitted boxes for the body, arms,
legs and feet; the inherited Avia housing adds one. Small protrusions and
propellers have no dedicated collision geometry.

config/sensor_profiles.json defines rates, fields of view and synthetic
noise. N6 uses a 1280x800/30 Hz PAG7936 reference profile; the installed module
is unconfirmed. MV-CA050 uses 1224x1024/20 Hz with a provisional 60-degree
horizontal FoV. The 200 m rangefinder is generic. Lens distortion and
hardware processing are not simulated.

Regenerate
----------
  python3 -B scripts/generate_porter_d2.py

The generator uses bundled assets/configuration and requires numpy. No CAD
application is required. It regenerates the vehicle, sensor modules, ROS
configuration and PX4 parameters.
