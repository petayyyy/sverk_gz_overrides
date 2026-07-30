# MTF-01 smoke test

The temporary `worlds/mtf01_aruco_test.sdf` world places the reusable `mtf01`
model 1 m above `obrik_aruco_map_4x4`, pointing down with the same 180-degree
X-axis rotation used on Gigaobrik.

The model uses the MicoAir MTF-01 specifications: a 42 degree optical-flow
field of view, 100 Hz output rate, 8 cm minimum optical-flow distance, and a
2 cm to 8 m ToF range. Its sensing direction is model-local `+Z`; the shared
flow/range origin is `(0, -0.0035, 0.010)` m in `mtf01_link`.

The map model itself is visual-only, so this test world also supplies a white
rendered floor 1 mm below it. That surface is what the render-based ToF sensor
measures.

## Install/build the bridge

Run the override installer once in the SITL checkout, then rebuild the new ROS
package and PX4 (the DDS input is generated into the PX4 binary):

```bash
bash scripts/update_overrides.sh
cd ~/sverk_ws
colcon build --packages-select mtf01_px4_bridge
source install/setup.bash
cd ~/PX4-Autopilot
make px4_sitl
```

## Start Gazebo

From this repository in the Gazebo / ROS 2 environment:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r worlds/mtf01_aruco_test.sdf
```

Inspect the sensor topics:

```bash
gz topic -l | grep mtf01_test
```

## Bridge and view the image

In another sourced ROS terminal:

```bash
image_topic=/world/mtf01_aruco_test/model/mtf01_test/link/flow_link/sensor/flow_camera/image
scan_topic=/world/mtf01_aruco_test/model/mtf01_test/link/lidar_sensor_link/sensor/lidar/scan

ros2 run ros_gz_bridge parameter_bridge \
  "$image_topic@sensor_msgs/msg/Image@gz.msgs.Image" \
  "$scan_topic@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan" \
  --ros-args \
  -r "$image_topic:=/obrik/mtf01/flow/image_raw" \
  -r "$scan_topic:=/obrik/mtf01/rangefinder/scan"
```

Select `/obrik/mtf01/flow/image_raw` in `rqt_image_view`, or add an Image
display for it in RViz. Check the ToF value with:

```bash
ros2 topic echo /obrik/mtf01/rangefinder/scan --once
```

## Verify the PX4 optical-flow message

With the bridge above still running:

```bash
ros2 run mtf01_px4_bridge mtf01_px4_bridge
ros2 topic echo /fmu/in/sensor_optical_flow --once \
  --qos-reliability best_effort
```

The model is static in this test world, so steady-state flow should be close to
zero and quality should remain nonzero over the textured map. The flow camera
is an internal simulation source, not a public video output of the physical
MTF-01.

The automated validation used the same path and measured:

- a non-uniform 100×100 RGB image;
- ToF distance `0.990 m`;
- nonzero integrated flow after a commanded 3 cm translation;
- flow quality `201/255`;
- no Gazebo or ROS bridge errors.
