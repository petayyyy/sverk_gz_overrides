#!/usr/bin/env python3

from __future__ import annotations

import math
import statistics

import rclpy
from px4_msgs.msg import DistanceSensor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class RangefinderPx4Bridge(Node):
    """Convert a downward Gazebo scan to PX4's distance-sensor input."""

    def __init__(self) -> None:
        super().__init__("rangefinder_px4_bridge")
        self.declare_parameter("input_topic", "/obrik/rangefinder_down/scan")
        self.declare_parameter("output_topic", "/fmu/in/distance_sensor")
        self.declare_parameter("device_id", 1)
        self.declare_parameter("min_distance", 0.03)
        self.declare_parameter("max_distance", 4.0)
        self.declare_parameter("variance", 0.0004)

        self._device_id = int(self.get_parameter("device_id").value)
        self._min_distance = float(self.get_parameter("min_distance").value)
        self._max_distance = float(self.get_parameter("max_distance").value)
        self._variance = float(self.get_parameter("variance").value)
        if self._min_distance <= 0.0 or self._max_distance <= self._min_distance:
            raise ValueError("distance limits must satisfy 0 < min < max")
        if self._variance < 0.0:
            raise ValueError("variance must not be negative")

        output_topic = str(self.get_parameter("output_topic").value)
        input_topic = str(self.get_parameter("input_topic").value)
        self._publisher = self.create_publisher(
            DistanceSensor, output_topic, qos_profile_sensor_data
        )
        self._subscription = self.create_subscription(
            LaserScan, input_topic, self._on_scan, qos_profile_sensor_data
        )
        self._warned_invalid_scan = False
        self.get_logger().info(f"Rangefinder bridge: {input_topic} -> {output_topic}")

    def _on_scan(self, scan: LaserScan) -> None:
        lower = max(self._min_distance, float(scan.range_min))
        upper = min(self._max_distance, float(scan.range_max))
        valid = [
            float(value)
            for value in scan.ranges
            if math.isfinite(value) and lower <= value <= upper
        ]
        if not valid:
            if not self._warned_invalid_scan:
                self.get_logger().warning("Ignoring range scan without a valid sample")
                self._warned_invalid_scan = True
            return
        self._warned_invalid_scan = False

        msg = DistanceSensor()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.device_id = self._device_id
        msg.min_distance = self._min_distance
        msg.max_distance = self._max_distance
        msg.current_distance = statistics.median(valid)
        msg.variance = self._variance
        msg.signal_quality = 100
        msg.type = DistanceSensor.MAV_DISTANCE_SENSOR_LASER
        msg.h_fov = max(0.0, float(scan.angle_max - scan.angle_min))
        msg.v_fov = 0.0
        msg.q = [1.0, 0.0, 0.0, 0.0]
        msg.orientation = DistanceSensor.ROTATION_DOWNWARD_FACING
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RangefinderPx4Bridge()
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
