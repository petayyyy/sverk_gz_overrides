# Orange Pi camera smoke test

The temporary `worlds/orange_pi_camera_aruco_test.sdf` world places the
`orange_pi_camera` 2.5 m above `obrik_aruco_map_4x4`, pointing straight down.

The default simulated stream uses the OV13850 family's real EIS-720p mode:
1408x792 RGB at 30 Hz, with a 77.6 degree horizontal field of view and a
0.1 m near plane. The model also exposes an inactive `imager_fullres` sensor
with the native 4224x3136 mode; it starts rendering only when its scoped
Gazebo topic is explicitly subscribed.

## Start Gazebo

From this repository in the Gazebo / ROS 2 environment:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r worlds/orange_pi_camera_aruco_test.sdf
```

## Bridge the camera to ROS 2

In another terminal, with the same Gazebo partition:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/orange_pi_camera_aruco_test/model/orange_pi_camera_test/link/orange_pi_camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /world/orange_pi_camera_aruco_test/model/orange_pi_camera_test/link/orange_pi_camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  --ros-args \
  -r /world/orange_pi_camera_aruco_test/model/orange_pi_camera_test/link/orange_pi_camera_link/sensor/imager/image:=/orange_pi_camera/image_raw \
  -r /world/orange_pi_camera_aruco_test/model/orange_pi_camera_test/link/orange_pi_camera_link/sensor/imager/camera_info:=/orange_pi_camera/camera_info
```

View the image with either:

```bash
rqt_image_view /orange_pi_camera/image_raw
```

or add an `Image` display in RViz 2 and select
`/orange_pi_camera/image_raw`.

The operational mode is also a native OV13850 output size; it avoids reducing
a software-rendered two-camera flight simulation to about 0.01x real time.
Use `imager_fullres` only for explicit full-resolution sensor validation.
