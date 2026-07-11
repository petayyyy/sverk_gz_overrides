#!/usr/bin/env python3
"""Hardware-free equivalent of led_control for Gazebo SITL.

Event ordering and effect timings intentionally mirror the real LED node.
``/led/state`` keeps the public 10 Hz API, while ``/led/sim/frame`` publishes
every rendered frame for the Gazebo material plugin.
"""

import colorsys
import time
from dataclasses import dataclass
from typing import Any, Optional

import rclpy
import yaml
from led_interfaces.msg import LEDState, LEDStateArray
from led_interfaces.srv import SetLEDEffect, SetLEDs
from px4_msgs.msg import BatteryStatus, VehicleStatus
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32

EFFECT_FILL = "fill"
EFFECT_BLINK = "blink"
EFFECT_BLINK_FAST = "blink_fast"
EFFECT_FADE = "fade"
EFFECT_WIPE = "wipe"
EFFECT_FLASH = "flash"
EFFECT_RAINBOW = "rainbow"
EFFECT_RAINBOW_FILL = "rainbow_fill"
EFFECTS = (
    EFFECT_FILL, EFFECT_BLINK, EFFECT_BLINK_FAST, EFFECT_FADE,
    EFFECT_WIPE, EFFECT_FLASH, EFFECT_RAINBOW, EFFECT_RAINBOW_FILL,
)

ARMING_STATE_ARMED = 2
NAV_STATE_TO_EVENT = {
    0: "stabilized", 1: "altctl", 2: "posctl", 3: "mission",
    4: "mission", 5: "rtl", 6: "position_slow", 8: "altitude_cruise",
    10: "acro", 12: "descend", 13: "termination", 14: "offboard",
    15: "stabilized", 17: "takeoff", 18: "land", 19: "follow_target",
    20: "precland", 21: "orbit", 22: "vtol_takeoff",
}


@dataclass
class Color:
    r: int = 0
    g: int = 0
    b: int = 0


class MemoryStrip:
    """Subset of the hardware strip API used by led_control."""

    def __init__(self, count: int):
        self._pixels = [Color() for _ in range(count)]
        self.brightness = 1.0

    def set_brightness(self, value: float) -> None:
        self.brightness = max(0.0, min(1.0, float(value)))

    def set_pixel_color(self, index: int, color: Color) -> None:
        if 0 <= index < len(self._pixels):
            self._pixels[index] = Color(color.r, color.g, color.b)

    def set_all_pixels(self, color: Color) -> None:
        self._pixels = [Color(color.r, color.g, color.b) for _ in self._pixels]

    def clear(self) -> None:
        self.set_all_pixels(Color())

    def show(self) -> None:
        pass


def hsv_to_rgb(h: float, s: float, v: float) -> tuple[int, int, int]:
    r, g, b = colorsys.hsv_to_rgb(h % 1.0, s, v)
    return int(r * 255), int(g * 255), int(b * 255)


def parse_events_config(events_yaml: str) -> dict[str, dict[str, Any]]:
    if not events_yaml or not events_yaml.strip():
        return {}
    try:
        data = yaml.safe_load(events_yaml)
    except yaml.YAMLError:
        return {}
    if not isinstance(data, dict):
        return {}
    return {
        key: value for key, value in data.items()
        if isinstance(value, dict)
        and str(value.get("effect") or "").strip().lower() != "none"
    }


class LEDStripSimNode(Node):
    def __init__(self) -> None:
        super().__init__("led_strip_sim_node")
        for name, default in (
            ("led_count", 112), ("brightness", 30.0),
            ("brightness_low_battery", 1.0), ("state_publish_rate", 10.0),
            ("animation_rate", 30.0), ("led_notify", True),
            ("fmu_out_prefix", "/fmu/out"), ("vehicle_status_timeout", 2.0),
            ("battery_min_voltage_per_cell", 3.5), ("battery_min_voltage", 6.0),
            ("events", ""),
        ):
            self.declare_parameter(name, default)

        self._n = int(self.get_parameter("led_count").value)
        self._strip = MemoryStrip(self._n)
        self._brightness_normal = max(0.0, min(100.0, float(self.get_parameter("brightness").value)))
        self._brightness_low_battery = max(
            0.0, min(100.0, float(self.get_parameter("brightness_low_battery").value))
        )
        self._brightness_low_active = True
        self._strip.set_brightness(self._brightness_low_battery / 100.0)

        self._effect = EFFECT_FILL
        self._effect_rgb = (255, 255, 255)
        self._previous_effect = EFFECT_FILL
        self._previous_rgb = (255, 255, 255)
        self._manual_leds: dict[int, tuple[int, int, int]] = {}
        self._effect_start_time = time.monotonic()
        self._rainbow_hue = 0.0
        self._fade_start_pixels: Optional[list[tuple[int, int, int]]] = None
        self._last_pixels = [(0, 0, 0)] * self._n

        self._events_config = parse_events_config(str(self.get_parameter("events").value))
        self._led_notify = bool(self.get_parameter("led_notify").value)
        self._fmu_out_prefix = str(self.get_parameter("fmu_out_prefix").value).rstrip("/")
        self._vehicle_status_timeout = float(self.get_parameter("vehicle_status_timeout").value)
        self._battery_min_voltage_per_cell = float(self.get_parameter("battery_min_voltage_per_cell").value)
        self._battery_min_voltage = float(self.get_parameter("battery_min_voltage").value)
        self._last_vehicle_status_time: Optional[float] = None
        self._connected = False
        self._last_nav_event: Optional[str] = None
        self._last_arming_event: Optional[str] = None
        self._low_battery_active = False
        self._logged_no_vehicle_status = False
        self._error_active_until = 0.0  # Kept for exact real-node semantics; intentionally unread.

        self._state_pub = self.create_publisher(LEDStateArray, "led/state", 10)
        self._frame_pub = self.create_publisher(LEDStateArray, "led/sim/frame", 10)
        self._brightness_pub = self.create_publisher(Float32, "led/sim/brightness", 10)
        self._set_effect_srv = self.create_service(SetLEDEffect, "led/set_effect", self._cb_set_effect)
        self._set_leds_srv = self.create_service(SetLEDs, "led/set_leds", self._cb_set_leds)

        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        if self._led_notify and self._events_config:
            self._apply_event("startup")
            self._vehicle_status_sub = self.create_subscription(
                VehicleStatus, f"{self._fmu_out_prefix}/vehicle_status",
                self._cb_vehicle_status, qos_px4,
            )
            # PX4 publishes the versioned topic in the current SITL DDS
            # profile, while other target profiles still use the unversioned
            # name. Subscribe to both so the simulation follows either FC.
            self._vehicle_status_v1_sub = self.create_subscription(
                VehicleStatus, f"{self._fmu_out_prefix}/vehicle_status_v1",
                self._cb_vehicle_status, qos_px4,
            )
            self._battery_status_sub = self.create_subscription(
                BatteryStatus, f"{self._fmu_out_prefix}/battery_status",
                self._cb_battery_status, qos_px4,
            )
            self._rosout_sub = self.create_subscription(Log, "/rosout", self._cb_rosout, 10)
            self._watchdog_timer = self.create_timer(1.0, self._watchdog_cb)

        self._state_timer = self.create_timer(
            1.0 / float(self.get_parameter("state_publish_rate").value), self._publish_state
        )
        animation_rate = float(self.get_parameter("animation_rate").value)
        self._animation_timer = self.create_timer(1.0 / animation_rate, self._animation_tick)
        self.get_logger().info(
            f"Simulated LED strip started: {self._n} logical LEDs, {animation_rate:.1f} Hz"
        )

    def _set_effect(self, effect: str, rgb: tuple[int, int, int]) -> None:
        self._previous_effect = self._effect
        self._previous_rgb = self._effect_rgb
        self._effect = effect
        self._effect_rgb = rgb
        self._effect_start_time = time.monotonic()
        self._manual_leds.clear()
        self._fade_start_pixels = None

    def _apply_event(self, event_name: str) -> None:
        cfg = self._events_config.get(event_name)
        if not cfg:
            return
        effect = str(cfg.get("effect", EFFECT_FILL))
        if effect not in EFFECTS:
            effect = EFFECT_FILL
        rgb = tuple(max(0, min(255, int(cfg.get(c, 0)))) for c in ("r", "g", "b"))
        self._set_effect(effect, rgb)
        self.get_logger().info(f"[LED/event] {event_name} -> {effect} rgb={rgb}")

    def _cb_vehicle_status(self, msg: VehicleStatus) -> None:
        self._last_vehicle_status_time = time.monotonic()
        if not self._connected:
            self._connected = True
            self._apply_event("connected")
        arming_event = "armed" if msg.arming_state == ARMING_STATE_ARMED else "disarmed"
        if arming_event != self._last_arming_event:
            self._last_arming_event = arming_event
            self._apply_event(arming_event)
        nav_event = NAV_STATE_TO_EVENT.get(msg.nav_state)
        if nav_event and nav_event != self._last_nav_event:
            self._last_nav_event = nav_event
            self._apply_event(nav_event)

    def _cb_battery_status(self, msg: BatteryStatus) -> None:
        voltage_v = float(msg.voltage_v or 0.0)
        cell_count = int(msg.cell_count or 0)
        if voltage_v <= 0.0 or cell_count <= 0:
            self._low_battery_active = False
            if not self._brightness_low_active:
                self._brightness_low_active = True
                self._strip.set_brightness(self._brightness_low_battery / 100.0)
            return
        is_low = voltage_v / cell_count < self._battery_min_voltage_per_cell
        if is_low and not self._low_battery_active:
            self._low_battery_active = True
            self._apply_event("low_battery")
        elif not is_low:
            self._low_battery_active = False
        if voltage_v < self._battery_min_voltage:
            if not self._brightness_low_active:
                self._brightness_low_active = True
                self._strip.set_brightness(self._brightness_low_battery / 100.0)
        elif self._brightness_low_active:
            self._brightness_low_active = False
            self._strip.set_brightness(self._brightness_normal / 100.0)

    def _cb_rosout(self, msg: Log) -> None:
        if msg.level == Log.ERROR:
            self._error_active_until = time.monotonic() + 5.0
            self._apply_event("error")

    def _watchdog_cb(self) -> None:
        now = time.monotonic()
        if not self._logged_no_vehicle_status and self._last_vehicle_status_time is None and now > 3.0:
            self._logged_no_vehicle_status = True
            self.get_logger().warning("No PX4 vehicle_status received")
        if not self._led_notify or not self._connected or self._last_vehicle_status_time is None:
            return
        if now - self._last_vehicle_status_time > self._vehicle_status_timeout:
            self._connected = False
            self._last_vehicle_status_time = None
            self._last_nav_event = None
            self._last_arming_event = None
            self._apply_event("disconnected")

    def _cb_set_effect(self, request, response):
        effect = (request.effect or EFFECT_FILL).strip().lower()
        if effect not in EFFECTS:
            effect = EFFECT_FILL
        rgb = tuple(max(0, min(255, int(v))) for v in (request.r, request.g, request.b))
        self._set_effect(effect, rgb)
        response.success = True
        response.message = ""
        return response

    def _cb_set_leds(self, request, response):
        for led in request.leds:
            if 0 <= led.index < self._n:
                self._manual_leds[led.index] = (int(led.r), int(led.g), int(led.b))
        response.success = True
        return response

    def _make_frame_message(self) -> LEDStateArray:
        msg = LEDStateArray()
        msg.leds = [
            LEDState(index=i, r=rgb[0], g=rgb[1], b=rgb[2])
            for i, rgb in enumerate(self._last_pixels)
        ]
        return msg

    def _publish_state(self) -> None:
        self._state_pub.publish(self._make_frame_message())

    def _publish_render_frame(self) -> None:
        self._frame_pub.publish(self._make_frame_message())
        self._brightness_pub.publish(Float32(data=float(self._strip.brightness)))

    def _sync_pixels(self) -> None:
        self._last_pixels = [(p.r, p.g, p.b) for p in self._strip._pixels]

    def _animation_tick(self) -> None:
        if self._manual_leds:
            for index, rgb in self._manual_leds.items():
                self._strip.set_pixel_color(index, Color(*rgb))
            self._strip.show()
            self._sync_pixels()
            self._publish_render_frame()
            return

        t = time.monotonic() - self._effect_start_time
        r, g, b = self._effect_rgb
        if self._effect == EFFECT_FILL:
            self._strip.set_all_pixels(Color(r, g, b))
        elif self._effect == EFFECT_BLINK:
            self._strip.set_all_pixels(Color(r, g, b) if int(t * 2) % 2 == 0 else Color())
        elif self._effect == EFFECT_BLINK_FAST:
            self._strip.set_all_pixels(Color(r, g, b) if int(t * 6) % 2 == 0 else Color())
        elif self._effect == EFFECT_FADE:
            if self._fade_start_pixels is None:
                self._fade_start_pixels = list(self._last_pixels)
            k = min(1.0, t)
            for i, (sr, sg, sb) in enumerate(self._fade_start_pixels):
                self._strip.set_pixel_color(
                    i, Color(int(sr + (r - sr) * k), int(sg + (g - sg) * k), int(sb + (b - sb) * k))
                )
            if k >= 1.0:
                self._fade_start_pixels = None
        elif self._effect == EFFECT_WIPE:
            pos = int((t * 15) * self._n) % (self._n + 1)
            for i in range(self._n):
                self._strip.set_pixel_color(i, Color(r, g, b) if i < pos else Color())
        elif self._effect == EFFECT_FLASH:
            if t < 0.1:
                self._strip.set_all_pixels(Color(r, g, b))
            elif t < 0.2:
                self._strip.clear()
            elif t < 0.3:
                self._strip.set_all_pixels(Color(r, g, b))
            elif t < 0.4:
                self._strip.clear()
            else:
                self._effect = self._previous_effect
                self._effect_rgb = self._previous_rgb
                self._effect_start_time = time.monotonic()
                self._strip.set_all_pixels(Color(*self._previous_rgb))
        elif self._effect == EFFECT_RAINBOW:
            self._rainbow_hue += 0.005
            for i in range(self._n):
                self._strip.set_pixel_color(
                    i, Color(*hsv_to_rgb(self._rainbow_hue + i / self._n, 1.0, 1.0))
                )
        elif self._effect == EFFECT_RAINBOW_FILL:
            self._rainbow_hue += 0.01
            self._strip.set_all_pixels(Color(*hsv_to_rgb(self._rainbow_hue, 1.0, 1.0)))
        else:
            self._strip.set_all_pixels(Color(r, g, b))
        self._strip.show()
        self._sync_pixels()
        self._publish_render_frame()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LEDStripSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
