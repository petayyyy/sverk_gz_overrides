#!/usr/bin/env python3

from __future__ import annotations

import math
import statistics

import numpy as np
import rclpy
from cv_bridge import CvBridge
from px4_msgs.msg import SensorOpticalFlow
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from .flow import estimate_flow


class Mtf01Px4Bridge(Node):
    """Convert the simulated MTF-01 camera and ToF scan to PX4 optical flow."""

    def __init__(self) -> None:
        super().__init__("mtf01_px4_bridge")
        self.declare_parameter("image_topic", "/obrik/mtf01/flow/image_raw")
        self.declare_parameter("range_topic", "/obrik/mtf01/rangefinder/scan")
        self.declare_parameter("output_topic", "/fmu/in/sensor_optical_flow")
        self.declare_parameter("device_id", 2)
        self.declare_parameter("horizontal_fov", 0.733038285838)
        self.declare_parameter("min_ground_distance", 0.08)
        self.declare_parameter("max_ground_distance", 8.0)
        self.declare_parameter("max_flow_rate", 7.0)

        self._device_id = int(self.get_parameter("device_id").value)
        self._horizontal_fov = float(self.get_parameter("horizontal_fov").value)
        self._min_ground_distance = float(
            self.get_parameter("min_ground_distance").value
        )
        self._max_ground_distance = float(
            self.get_parameter("max_ground_distance").value
        )
        self._max_flow_rate = float(self.get_parameter("max_flow_rate").value)
        if not 0.0 < self._horizontal_fov < math.pi:
            raise ValueError("horizontal_fov must be between 0 and pi")
        if not 0.0 < self._min_ground_distance < self._max_ground_distance:
            raise ValueError("ground-distance limits must satisfy 0 < min < max")

        image_topic = str(self.get_parameter("image_topic").value)
        range_topic = str(self.get_parameter("range_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._cv_bridge = CvBridge()
        self._previous_image: np.ndarray | None = None
        self._previous_stamp_us: int | None = None
        self._distance_m = math.nan
        self._publisher = self.create_publisher(
            SensorOpticalFlow, output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, range_topic, self._on_scan, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"MTF-01 bridge: {image_topic} + {range_topic} -> {output_topic}"
        )

    def _on_scan(self, scan: LaserScan) -> None:
        lower = max(self._min_ground_distance, float(scan.range_min))
        upper = min(self._max_ground_distance, float(scan.range_max))
        samples = [
            float(value)
            for value in scan.ranges
            if math.isfinite(value) and lower <= value <= upper
        ]
        self._distance_m = statistics.median(samples) if samples else math.nan

    def _on_image(self, image: Image) -> None:
        current = self._cv_bridge.imgmsg_to_cv2(
            image, desired_encoding="mono8"
        )
        current = np.ascontiguousarray(current)
        stamp_us = (
            int(image.header.stamp.sec) * 1_000_000
            + int(image.header.stamp.nanosec) // 1_000
        )
        if stamp_us <= 0:
            stamp_us = self.get_clock().now().nanoseconds // 1_000

        if self._previous_image is None or self._previous_stamp_us is None:
            self._previous_image = current
            self._previous_stamp_us = stamp_us
            return

        integration_us = stamp_us - self._previous_stamp_us
        previous = self._previous_image
        self._previous_image = current
        self._previous_stamp_us = stamp_us
        if integration_us <= 0 or integration_us > 1_000_000:
            return

        estimate = estimate_flow(previous, current, self._horizontal_fov)
        msg = SensorOpticalFlow()
        now_us = self.get_clock().now().nanoseconds // 1_000
        msg.timestamp = now_us
        # PX4 expects both timestamps in the XRCE-DDS time domain. The image
        # stamp is Gazebo simulation time, so only use it for integration time
        # unless /clock is explicitly driving this ROS node.
        msg.timestamp_sample = now_us
        msg.device_id = self._device_id
        msg.pixel_flow = [estimate.integrated_x, estimate.integrated_y]
        msg.delta_angle = [math.nan, math.nan, math.nan]
        msg.delta_angle_available = False
        msg.distance_m = float(self._distance_m)
        msg.distance_available = math.isfinite(self._distance_m)
        msg.integration_timespan_us = int(integration_us)
        msg.quality = estimate.quality
        msg.error_count = 0
        msg.max_flow_rate = self._max_flow_rate
        msg.min_ground_distance = self._min_ground_distance
        msg.max_ground_distance = self._max_ground_distance
        msg.mode = SensorOpticalFlow.MODE_BRIGHT
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Mtf01Px4Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
