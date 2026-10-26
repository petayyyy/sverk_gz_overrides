Firefighter P4
==============

Gazebo Harmonic X8 with eight physical motor channels and six payload sensors.
Model URI: model://firefighter_P4. Axes: +X forward, +Y left, +Z up.
Meshes use metres. The original P4 assembly datum is retained.

Geometry comes from the updated P4 STEP, including the corrected Blackfly.
The loose Orange Pi reference is excluded. Motor/propeller meshes reuse P3.
Upper motor origins: (+/-130.762, +/-165.981, +42.400) mm.
Lower motor origins: (+/-130.626, +/-166.042, -17.400) mm.

Sensors
-------
H-Flow, the generic 200 m rangefinder and N6 look down. Foxeer looks forward.
Blackfly and Livox point approximately 10 degrees forward from down.
config/mounting_poses.json records CAD placements and sensor-frame registration.
The Avia registration reuses the approximately millimetre-accurate P3 fit.
Camera optical frames use +Z forward, +X image-right, +Y image-down.

Sensor rates, FoVs and synthetic noise reuse the existing P3 profiles.
N6 uses 1280x800 at 30 Hz; Blackfly uses 2448x2048 at 24 Hz and a provisional
60-degree horizontal FoV; Foxeer uses 640x480 at 30 Hz and 120 degrees.
Unidentified lenses, sensor noise and H-Flow optical/ToF co-location remain
provisional. H-Flow computes image-based flow and retains all 32 ToF pixels.
At N6's provisional 68.8-degree FoV, the body and Livox obstruct part of the
image. The measured CAD mount is retained; review placement before using N6
for vision tasks that need its full field of view.

Run
---
Install using scripts/update_overrides.sh and build porter_sensor_adapters.
The companion sverk-ros2 launch must include the firefighter_P4 selector:

  bash scripts/sitl/sim.sh launch --display headless --config firefighter_P4 --world obrik_aruco

N6 and Blackfly bridge by default. To include FPV, append:

  -- firefighter_p4_cameras:=all

For an already spawned vehicle, start only its payload bridges:

  ros2 launch porter_sensor_adapters firefighter_P4.launch.py gz_world:=obrik_aruco model_instance:=firefighter_P4_0 cameras:=all

ROS outputs under /firefighter_P4:
  {n6,blackfly,foxeer}/{image_raw,camera_info}
  hflow/{optical_flow,imu,tof/range,tof/scan/points}
  rangefinder_200m/range
  avia/{points,imu}
  ground_truth/odometry

Payload flow and range are not automatically injected into PX4's EKF.
Foxeer bridging is optional. Avia retains /livox_avia native Gazebo topics;
one Avia instance per world is supported. Its texel correction stays enabled.

Physics and collisions
----------------------
config/physics.json uses a provisional 3 kg total mass, estimated CoM and
box inertias, and 12 N per motor at 1000 rad/s. Replace these with measured
data before comparing real flight performance. Coaxial interference is not
simulated. Motor housings stay fixed and all eight propellers rotate.

config/collision_boxes.json contains 28 oriented airframe boxes fitted to
the P4 structural CAD, with approximately 1 mm padding. Separate ring segments
preserve the open landing frame. Four feet share a horizontal contact plane.
Each sensor adds a fitted housing box. Flexible antennas, cables, tiny
fasteners and propellers have no dedicated collision geometry.
Collision geometry is independent of the provisional inertia estimates.

Regenerate
----------
  python3 -B scripts/generate_firefighter_p4.py

The generator uses bundled meshes, mounts and existing P3 sensor definitions.
It requires numpy, but no CAD application or original STEP file.
