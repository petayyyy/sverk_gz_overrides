#!/usr/bin/env python3

from __future__ import annotations

from typing import Any
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class GraffitiServoSimNode(Node):
    """Servo-compatible ROS interface for the Gazebo graffiti mechanism."""

    def __init__(self) -> None:
        super().__init__("servo_node")

        self.declare_parameter("min_angle_deg", 30.0)
        self.declare_parameter("max_angle_deg", 180.0)
        self.declare_parameter("center_angle_deg", 90.0)
        self.declare_parameter("pressed_angle_deg", 180.0)
        self.declare_parameter("startup_angle_deg", 90.0)
        self.declare_parameter("start_enabled", True)
        self.declare_parameter("servo_speed_deg_per_sec", 360.0)
        self.declare_parameter("command_publish_rate_hz", 30.0)
        self.declare_parameter("status_publish_rate_hz", 2.0)
        self.declare_parameter("reverse_travel_fraction", 0.6666666667)

        self.declare_parameter("travel_topic", "/spray/servo/travel")
        self.declare_parameter("publish_spray_command", True)
        self.declare_parameter("spray_command_topic", "/spray")
        self.declare_parameter("spray_active_angle_deg", 170.0)

        self._min_angle_deg = float(self.get_parameter("min_angle_deg").value)
        self._max_angle_deg = float(self.get_parameter("max_angle_deg").value)
        self._center_angle_deg = float(self.get_parameter("center_angle_deg").value)
        self._pressed_angle_deg = float(self.get_parameter("pressed_angle_deg").value)
        startup_angle_deg = float(self.get_parameter("startup_angle_deg").value)
        self._enabled = bool(self.get_parameter("start_enabled").value)
        self._servo_speed_deg_per_sec = float(
            self.get_parameter("servo_speed_deg_per_sec").value
        )
        command_publish_rate_hz = float(
            self.get_parameter("command_publish_rate_hz").value
        )
        status_publish_rate_hz = float(
            self.get_parameter("status_publish_rate_hz").value
        )

        self._reverse_travel_fraction = float(
            self.get_parameter("reverse_travel_fraction").value
        )
        self._publish_spray_command = bool(
            self.get_parameter("publish_spray_command").value
        )
        self._spray_active_angle_deg = float(
            self.get_parameter("spray_active_angle_deg").value
        )

        if self._min_angle_deg >= self._max_angle_deg:
            raise ValueError("min_angle_deg must be smaller than max_angle_deg")
        if self._pressed_angle_deg == self._center_angle_deg:
            raise ValueError("pressed_angle_deg must differ from center_angle_deg")
        if self._reverse_travel_fraction < 0.0:
            raise ValueError("reverse_travel_fraction must not be negative")
        if self._servo_speed_deg_per_sec <= 0.0:
            raise ValueError("servo_speed_deg_per_sec must be positive")
        if command_publish_rate_hz <= 0.0:
            raise ValueError("command_publish_rate_hz must be positive")

        self._target_angle_deg = self._clamp_angle(startup_angle_deg)
        self._current_angle_deg = self._target_angle_deg
        self._last_spray_command: Optional[bool] = None
        self._last_update_time = self.get_clock().now()

        self._angle_pub = self.create_publisher(Float32, "~/current_angle_deg", 10)
        self._enabled_pub = self.create_publisher(Bool, "~/enabled", 10)
        self._travel_pub = self.create_publisher(Float32, "~/travel", 10)
        self._gz_travel_pub = self.create_publisher(
            Float64, str(self.get_parameter("travel_topic").value), 10
        )
        self._spray_pub = None
        if self._publish_spray_command:
            self._spray_pub = self.create_publisher(
                Bool, str(self.get_parameter("spray_command_topic").value), 10
            )

        self.create_subscription(
            Float32, "~/target_angle_deg", self._handle_target_angle, 10
        )
        self.create_service(SetBool, "~/enable", self._handle_enable)
        self.create_service(Trigger, "~/center", self._handle_center)

        self.create_timer(1.0 / command_publish_rate_hz, self._update_servo)
        if status_publish_rate_hz > 0.0:
            self.create_timer(1.0 / status_publish_rate_hz, self._publish_status)

        self._publish_joint_targets()
        self._publish_status()
        self.get_logger().info(
            "Graffiti servo simulator ready: /servo_node/target_angle_deg -> "
            "/spray/servo/travel"
        )

    def _clamp_angle(self, angle_deg: float) -> float:
        return _clamp(angle_deg, self._min_angle_deg, self._max_angle_deg)

    def _servo_travel(self) -> float:
        press_span = self._pressed_angle_deg - self._center_angle_deg
        travel = (self._current_angle_deg - self._center_angle_deg) / press_span
        return _clamp(travel, -self._reverse_travel_fraction, 1.0)

    def _publish_float64(self, publisher: Any, value: float) -> None:
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)

    def _publish_joint_targets(self) -> None:
        travel = self._servo_travel()
        self._publish_float64(self._gz_travel_pub, travel)

        if self._spray_pub is not None:
            spray_active = bool(
                self._enabled and self._current_angle_deg >= self._spray_active_angle_deg
            )
            if spray_active != self._last_spray_command:
                spray_msg = Bool()
                spray_msg.data = spray_active
                self._spray_pub.publish(spray_msg)
                self._last_spray_command = spray_active

    def _publish_status(self) -> None:
        angle_msg = Float32()
        angle_msg.data = float(self._current_angle_deg)
        self._angle_pub.publish(angle_msg)

        enabled_msg = Bool()
        enabled_msg.data = bool(self._enabled)
        self._enabled_pub.publish(enabled_msg)

        travel_msg = Float32()
        travel_msg.data = float(self._servo_travel())
        self._travel_pub.publish(travel_msg)

    def _set_target_angle(self, angle_deg: float, enable: bool = True) -> float:
        self._target_angle_deg = self._clamp_angle(angle_deg)
        if enable:
            self._enabled = True
        self._publish_status()
        return self._target_angle_deg

    def _handle_target_angle(self, msg: Float32) -> None:
        angle_deg = self._set_target_angle(float(msg.data), enable=True)
        self.get_logger().info("Servo target angle set to %.1f deg" % angle_deg)

    def _handle_enable(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self._enabled = bool(request.data)
        self._publish_joint_targets()
        self._publish_status()
        response.success = True
        response.message = "enabled" if self._enabled else "disabled"
        return response

    def _handle_center(
        self, _: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        angle_deg = self._set_target_angle(self._center_angle_deg, enable=True)
        response.success = True
        response.message = "centered at %.1f deg" % angle_deg
        return response

    def _update_servo(self) -> None:
        now = self.get_clock().now()
        dt = max(0.0, (now - self._last_update_time).nanoseconds / 1_000_000_000.0)
        self._last_update_time = now

        if self._enabled:
            delta = self._target_angle_deg - self._current_angle_deg
            max_step = self._servo_speed_deg_per_sec * dt
            if abs(delta) <= max_step:
                self._current_angle_deg = self._target_angle_deg
            elif delta > 0.0:
                self._current_angle_deg += max_step
            else:
                self._current_angle_deg -= max_step

        self._publish_joint_targets()

    def destroy_node(self) -> bool:
        if self._spray_pub is not None:
            spray_msg = Bool()
            spray_msg.data = False
            self._spray_pub.publish(spray_msg)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraffitiServoSimNode()
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
