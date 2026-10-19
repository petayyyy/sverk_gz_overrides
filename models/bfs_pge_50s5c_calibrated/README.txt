Author's calibrated FLIR attestation rig
=====================================
This model is opt-in. It represents the report author's 1224x1024 binned
BFS-PGE-50S5C camera, not an inferred lens calibration for Firefighter P3.
Porter D2 remains camera-free by default; P3 keeps its native camera profile.

calibration.json contains OpenCV pixel-centre intrinsics. SDF cx/cy are +0.5
pixel because Gazebo's renderer uses pixel-edge coordinates. Bridge the raw
Gazebo CameraInfo to a *_gz ROS topic, then run gazebo_camera_tools camera_info
with distinct input_topic and output_topic. That node subtracts 0.5 from K/P
principal points, preserving image stamps and all other calibration fields.
Do not apply this correction a second time or to real-camera CameraInfo.

The associated ROS repository's prepare_navigation_experiment.py supports
--calibrated-flir to generate the full opt-in Porter author rig and calibration.
