Firefighter P3
==============

Gazebo Harmonic X8 configuration with six payload sensors. Model URI:
model://firefighter_P3. Body axes are +X forward, +Y left, +Z up; the body
datum is the CAD origin. Decimated body and sensor meshes use millimetres,
scaled by 0.001 in SDF. Eight VMGs reuse geometry separated from
x500_obrik_firefighter_base/meshes/vmg.obj; these VMG meshes use metres.

H-Flow, the rangefinder, N6 and Blackfly look downward. Livox Avia points
approximately 10 degrees forward from downward. Foxeer faces +X with an
upright image. Sensor poses are in config/mounting_poses.json. Camera optical
frames use +Z forward, +X right, +Y down.

Run
---
Install models and ROS packages with scripts/update_overrides.sh. In the ROS
workspace, build the dependencies and source the result:

  colcon build --packages-up-to porter_sensor_adapters livox_avia_cloud_adapter
  source install/setup.bash

Tested runtime: Gazebo Harmonic 8.15, ROS 2 Humble and PX4 v1.15.

In sverk-ros2, select the configuration with an existing world. PX4 spawns
the vehicle into that world:

  bash scripts/sitl/sim.sh launch --display headless --config firefighter_P3 --world obrik_aruco

This requires the P3 selector support in sverk-ros2. It applies
config/px4_parameters.json to the existing eight-motor PX4 airframe.

To start only the ROS payload bridges for an already spawned vehicle:

  ros2 launch porter_sensor_adapters firefighter_P3.launch.py gz_world:=obrik_aruco model_instance:=firefighter_P3

N6 and Blackfly images bridge by default; Foxeer is optional. In the shared
launch, add -- firefighter_p3_cameras:=all, :=none, or a comma-separated
subset of n6,blackfly,foxeer. The payload launch uses cameras:=all instead.
Cameras render only when subscribed. Full-resolution Blackfly rendering can
be slow with a software renderer. Set gz_world and model_instance to the
actual world and spawned vehicle names for the scoped Blackfly bridge.

ROS outputs
-----------
Topics are under /firefighter_P3:

  hflow/optical_flow                  porter_sensor_msgs/OpticalFlow
  hflow/tof/scan/points               sensor_msgs/PointCloud2 (8x4 ToF grid)
  hflow/tof/range                     sensor_msgs/Range
  hflow/imu                           sensor_msgs/Imu
  rangefinder_200m/range              sensor_msgs/Range
  {n6,blackfly,foxeer}/image_raw       sensor_msgs/Image
  {n6,blackfly,foxeer}/camera_info     sensor_msgs/CameraInfo
  avia/points                         sensor_msgs/PointCloud2
  avia/imu                            sensor_msgs/Imu
  ground_truth/odometry              nav_msgs/Odometry

Static TF mounts root at firefighter_P3/base_link. The shared launch aliases
this to base_link; localization must provide its world transform. Payload
flow/range data are not injected into PX4's EKF. The ToF Range is the median
of valid rays from the full grid and supplies the optical-flow adapter.
The Avia module retains global Gazebo topics /livox_avia/...; this setup supports one Avia per world.
The Avia GPU-ray texel correction is enabled by default; use
avia_texel_fix:=false to disable it.

Parameters
----------
config/physics.json sets a provisional 3 kg total mass, box inertias and
motor response: 12 N per motor at 1000 rad/s. Replace these with measured
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
is unconfirmed. Blackfly renders 2448x2048/24 Hz ideal RGB with a provisional
60-degree horizontal FoV. Foxeer uses 640x480/30 Hz and 120-degree horizontal
FoV. The 200 m rangefinder is generic. Lens distortion and hardware processing
are not simulated.

Regenerate
----------
  python3 -B scripts/generate_firefighter_p3.py

The generator uses bundled assets/configuration and requires numpy. No CAD
application is required. It regenerates the vehicle, sensor modules, ROS
configuration and PX4 parameters.
