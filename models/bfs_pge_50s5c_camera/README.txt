Blackfly S BFS-PGE-50S5C-C
=========================

Reusable Gazebo Harmonic camera: model://bfs_pge_50s5c_camera.
Link: bfs_pge_50s5c_camera_link. Sensor: imager.

Frame and mounting
------------------
The decimated 4,016-triangle mesh retains the original CAD origin and axes:
+X sensing direction, +Y left, +Z up. Vertex units are millimetres; the SDF
applies 0.001 scale. The sensing origin is (0.086882, 0, 0) metres from the
mesh origin, using the front lens surface as the approximate pinhole centre.
The actual lens principal plane is unknown.

Mount the module using the mesh-origin pose; the sensing offset is already
applied inside the module. A fixed joint attaches
<instance>::bfs_pge_50s5c_camera_link to the vehicle link. Firefighter P3
includes this camera with its +X axis directed toward drone -Z.

The optical frame has +Z forward, +X image-right, +Y image-down. To publish
ROS TF, use the mesh-origin mount followed by sensing translation
(0.086882, 0, 0) and optical RPY (-pi/2, 0, -pi/2). SDF frames alone do not
publish ROS TF.

Profile
-------
  Image: 2448x2048 RGB at 24 Hz
  Horizontal FoV: provisional 60 degrees
  Near/far clip: 0.01 / 300 metres
  Gaussian intensity noise: mean 0, stddev 0.004 (normalized channels)
  Mass including lens: provisional 0.15 kg

The IMX264 hardware reaches 24 fps in BayerRG8; the simulation renders ideal
processed RGB. The installed lens is unidentified. Distortion, demosaicing,
exposure, motion blur and GigE/driver behaviour are not simulated. Noise and
bounding-box inertia are estimates. This payload has no collision geometry.
Include its mass in the vehicle total once.

Rendering is subscriber-driven. Gazebo creates scoped image and camera_info
topics; the model does not start a ROS bridge. For multiple instances, assign
unique optical_frame_id values and publish matching ROS mounting transforms.

Use in a vehicle
----------------
Include model://bfs_pge_50s5c_camera and publish its mounting transforms.
In Firefighter P3, the payload launch provides
/firefighter_P3/blackfly/image_raw and /firefighter_P3/blackfly/camera_info.

profile.json records source hashes, mesh bounds and imaging assumptions.
Manufacturer specifications:
https://softwareservices.flir.com/BFS-PGE-50S5/latest/Model/spec.html
