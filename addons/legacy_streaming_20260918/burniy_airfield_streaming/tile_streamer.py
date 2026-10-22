#!/usr/bin/env python3
"""Stream Burniy mesh tiles into a running Gazebo Sim world.

This process deliberately uses Gazebo Transport's public create/remove
services instead of patching Sverk or rebuilding its image.  Tile entities are
visual-only and share the existing Burniy mesh/material directories.
"""

from __future__ import annotations

import argparse
import json
import math
import logging
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Set, Tuple

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.scene_pb2 import Scene
from gz.transport13 import Node


GRID_COUNT = 24
MAP_MIN = -1500.0
MAP_MAX = 1500.0
TILE_SIZE = (MAP_MAX - MAP_MIN) / GRID_COUNT
TILE_MODEL_PREFIX = "burniy_stream_tile"

Tile = Tuple[int, int]
Point = Tuple[float, float]


@dataclass
class Window:
    side: int
    origin_x: int
    origin_y: int

    def tiles(self) -> Set[Tile]:
        return {
            (x, y)
            for x in range(self.origin_x, self.origin_x + self.side)
            for y in range(self.origin_y, self.origin_y + self.side)
        }


class TileStreamer:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.node = Node()
        self.lock = threading.Lock()
        self.stop_requested = threading.Event()
        self.vehicle_position: Point = (args.vehicle_x, args.vehicle_y)
        self.gui_position: Point = (args.gui_x, args.gui_y)
        self.gui_forward: Point = (1.0, 0.0)
        self.vehicle_seen = False
        self.gui_seen = False
        self.vehicle_window: Optional[Window] = None
        self.gui_window: Optional[Window] = None
        self.loaded: Set[Tile] = set()
        self.pending_remove: dict[Tile, float] = {}
        self.mesh_dir = self._resolve_mesh_dir()
        manifest = json.loads((Path(args.model_root) / "tiles.json").read_text())
        if (manifest['grid_count'], manifest['map_min'], manifest['tile_size']) != (GRID_COUNT, MAP_MIN, TILE_SIZE):
            raise RuntimeError("Tile manifest grid does not match this streamer")
        self.available_tiles = {(t['x'], t['y']) for t in manifest['tiles']}
        self.target_tiles = None
        self.create_service = f"/world/{args.world}/create"
        self.remove_service = f"/world/{args.world}/remove"
        self.scene_info_service = f"/world/{args.world}/scene/info"
        self.pose_topic = f"/world/{args.world}/dynamic_pose/info"
        self.node.subscribe(Pose_V, self.pose_topic, self._on_dynamic_pose)
        if args.gui_grid_side:
            self.node.subscribe(Pose, args.gui_pose_topic, self._on_gui_pose)

    def _resolve_mesh_dir(self) -> str:
        if self.args.mesh_dir != "auto":
            return self.args.mesh_dir
        if self.args.texture_mode == "off":
            for candidate in ("meshes_glb_untextured", "meshes_norm_untextured"):
                if (Path(self.args.model_root) / candidate).is_dir():
                    return candidate
            raise RuntimeError(
                "Textureless meshes are not installed; reinstall the addon before using "
                "--texture-mode off"
            )
        model_sdf = Path(self.args.model_root) / "model.sdf"
        try:
            content = model_sdf.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            raise RuntimeError(f"Cannot read installed Burniy model SDF: {model_sdf}: {exc}") from exc
        if "/meshes_glb/" in content:
            return "meshes_glb"
        if "/meshes_norm/" in content:
            return "meshes_norm"
        raise RuntimeError(f"Cannot identify the active mesh directory in {model_sdf}")

    @staticmethod
    def _tile_index(value: float) -> int:
        return max(0, min(GRID_COUNT - 1, int((value - MAP_MIN) // TILE_SIZE)))

    @staticmethod
    def _window_origin(index: int, side: int) -> int:
        return max(0, min(GRID_COUNT - side, index - side // 2))

    def _new_window(self, point: Point, side: int) -> Window:
        return Window(
            side=side,
            origin_x=self._window_origin(self._tile_index(point[0]), side),
            origin_y=self._window_origin(self._tile_index(point[1]), side),
        )

    def _advance_window(self, window: Window, point: Point) -> Window:
        """Move one tile toward a point-centred window.

        A one-tile step bounds the temporary old/new union after a camera
        teleport.  Unlike the previous wide inner-core hysteresis, continuing
        until the centred origin is reached prevents a stationary camera from
        being left only one tile away from a rendered edge.
        """
        target = self._new_window(point, window.side)

        def step(current: int, wanted: int) -> int:
            return current + (wanted > current) - (wanted < current)

        return Window(
            window.side,
            step(window.origin_x, target.origin_x),
            step(window.origin_y, target.origin_y),
        )

    def _on_dynamic_pose(self, message: Pose_V) -> None:
        for pose in message.pose:
            # Dynamic pose names are normally the model's unscoped name. The
            # suffix check keeps this robust if Gazebo scopes it in a later
            # release.
            if pose.name == self.args.vehicle_model or pose.name.endswith(
                f"::{self.args.vehicle_model}"
            ):
                with self.lock:
                    self.vehicle_position = (pose.position.x, pose.position.y)
                    self.vehicle_seen = True
                return

    def _on_gui_pose(self, message: Pose) -> None:
        # Gazebo's rendering camera looks along its local +X axis. Project
        # that axis into the world XY plane; retaining its length naturally
        # reduces look-ahead as the camera approaches a top-down view.
        qx = message.orientation.x
        qy = message.orientation.y
        qz = message.orientation.z
        qw = message.orientation.w
        forward = (
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy + qw * qz),
        )
        with self.lock:
            self.gui_position = (message.position.x, message.position.y)
            if all(math.isfinite(value) for value in forward):
                self.gui_forward = forward
            self.gui_seen = True

    @staticmethod
    def _gui_interest_point(position: Point, forward: Point, side: int) -> Point:
        # Reserve roughly one tile behind the camera and spend the remaining
        # half-window surplus in the viewing direction. For a 6x6 window this
        # is a 250 m look-ahead, reduced automatically by steep camera pitch.
        look_ahead = max(0.0, side / 2.0 - 1.0) * TILE_SIZE
        return (
            position[0] + forward[0] * look_ahead,
            position[1] + forward[1] * look_ahead,
        )

    @staticmethod
    def _tile_name(tile: Tile) -> str:
        return f"{TILE_MODEL_PREFIX}_{tile[0]:02d}_{tile[1]:02d}"

    def _mesh_uri(self, tile: Tile) -> str:
        extension = "glb" if self.mesh_dir.startswith("meshes_glb") else "obj"
        return (
            f"model://{self.args.model_name}/{self.mesh_dir}/"
            f"chunk_{tile[0]:02d}_{tile[1]:02d}.{extension}"
        )

    def _tile_sdf(self, tile: Tile) -> str:
        name = self._tile_name(tile)
        return (
            '<sdf version="1.9">'
            f'<model name="{name}"><static>true</static><self_collide>false</self_collide>'
            '<link name="tile"><visual name="mesh"><cast_shadows>false</cast_shadows>'
            f'<geometry><mesh><uri>{self._mesh_uri(tile)}</uri></mesh></geometry>'
            '</visual></link></model></sdf>'
        )

    def _request(self, service: str, request, request_type) -> bool:
        try:
            executed, response = self.node.request(
                service, request, request_type, Boolean, self.args.service_timeout_ms
            )
        except Exception as exc:  # Transport also raises while the server starts.
            logging.debug("Service %s unavailable: %s", service, exc)
            return False
        return bool(executed and response.data)

    def _scene_has_tile(self, tile: Tile) -> Optional[bool]:
        """Confirm an entity when UserCommands replied after our timeout.

        Gazebo accepts an asynchronous create command before the Ogre client
        finishes importing the GLB.  Its Boolean response can therefore time
        out even though the model is already present in world state.
        """
        try:
            executed, scene = self.node.request(
                self.scene_info_service,
                Empty(),
                Empty,
                Scene,
                self.args.service_timeout_ms,
            )
        except Exception:
            return None
        if not executed:
            return None
        return any(model.name == self._tile_name(tile) for model in scene.model)

    def _create(self, tile: Tile) -> bool:
        request = EntityFactory()
        request.sdf = self._tile_sdf(tile)
        if self._request(self.create_service, request, EntityFactory):
            logging.info("loaded tile %02d,%02d", *tile)
            return True
        # Let the accepted command reach the next simulation update, then
        # inspect authoritative world state. This keeps initial tile loading
        # fast even if mesh import delays the service response.
        self.stop_requested.wait(0.1)
        if self._scene_has_tile(tile):
            logging.info("loaded tile %02d,%02d", *tile)
            return True
        logging.warning("could not create tile %02d,%02d; will retry", *tile)
        return False

    def _remove(self, tile: Tile, quiet: bool = False) -> bool:
        request = Entity()
        request.name = self._tile_name(tile)
        request.type = Entity.MODEL
        if self._request(self.remove_service, request, Entity):
            logging.info("unloaded tile %02d,%02d", *tile)
            return True
        self.stop_requested.wait(0.1)
        if self._scene_has_tile(tile) is False:
            logging.info("unloaded tile %02d,%02d", *tile)
            return True
        if not quiet:
            logging.warning("could not remove tile %02d,%02d; will retry", *tile)
        return False

    def _desired_tiles(self) -> Set[Tile]:
        with self.lock:
            vehicle_position = self.vehicle_position
            gui_position = self.gui_position
            gui_forward = self.gui_forward

        desired: Set[Tile] = set()
        # A shift creates the incoming row/column first and holds the old one
        # briefly. Do not begin another shift during that overlap, even if a
        # GUI camera teleports several tiles, or the temporary union could
        # grow without bound.
        transition_in_progress = bool(self.pending_remove) or (self.target_tiles is not None and not self.target_tiles <= self.loaded)
        if self.args.vehicle_grid_side:
            if self.vehicle_window is None:
                self.vehicle_window = self._new_window(vehicle_position, self.args.vehicle_grid_side)
            elif not transition_in_progress:
                self.vehicle_window = self._advance_window(self.vehicle_window, vehicle_position)
            desired.update(self.vehicle_window.tiles())
        if self.args.gui_grid_side:
            gui_interest = self._gui_interest_point(
                gui_position, gui_forward, self.args.gui_grid_side
            )
            if self.gui_window is None:
                self.gui_window = self._new_window(gui_interest, self.args.gui_grid_side)
            elif not transition_in_progress:
                self.gui_window = self._advance_window(self.gui_window, gui_interest)
            desired.update(self.gui_window.tiles())

        desired &= self.available_tiles
        self.target_tiles = desired
        if len(desired) > self.args.loaded_tile_limit:
            raise RuntimeError(
                f"Desired {len(desired)} tiles exceeds loaded_tile_limit "
                f"{self.args.loaded_tile_limit}. Increase the limit or reduce a grid side."
            )
        return desired

    def _reconcile(self, desired: Set[Tile]) -> None:
        # Cancel an unload if the interest point returned before it happened.
        for tile in desired:
            self.pending_remove.pop(tile, None)

        to_add = sorted(desired - self.loaded)
        overlap_count = len(self.loaded | desired)
        if overlap_count > self.args.loaded_tile_limit:
            logging.warning(
                "Holding window shift: transition would need %d tiles, above limit %d",
                overlap_count,
                self.args.loaded_tile_limit,
            )
            return

        # Creation is intentionally first. The previous row/column stays in
        # place during this grace period so a moving camera has no map gap.
        for tile in to_add:
            if self.stop_requested.is_set():
                return
            if self._create(tile):
                self.loaded.add(tile)

        if not desired <= self.loaded:
            return  # Keep the old window until every replacement was accepted.

        remove_after = time.monotonic() + self.args.unload_grace_seconds
        for tile in self.loaded - desired:
            self.pending_remove.setdefault(tile, remove_after)

        now = time.monotonic()
        for tile, deadline in list(self.pending_remove.items()):
            if self.stop_requested.is_set():
                return
            if deadline > now or tile in desired:
                continue
            if self._remove(tile):
                self.loaded.discard(tile)
                self.pending_remove.pop(tile, None)
            else:
                self.pending_remove[tile] = now + 1.0

    def _clear_previous_tiles(self) -> None:
        executed, scene = self.node.request(
            self.scene_info_service, Empty(), Empty, Scene, self.args.service_timeout_ms
        )
        if not executed:
            raise RuntimeError("Cannot inspect previous tiles; refusing to lose entity accounting")
        names = {model.name for model in scene.model}
        for tile in self.available_tiles:
            if self._tile_name(tile) in names and not self._remove(tile, quiet=True):
                self.loaded.add(tile)

    def _wait_for_world_services(self) -> bool:
        """Wait for the server with a read-only SceneBroadcaster request."""
        deadline = time.monotonic() + self.args.startup_timeout_seconds
        while not self.stop_requested.is_set() and time.monotonic() < deadline:
            try:
                executed, _response = self.node.request(
                    self.scene_info_service, Empty(), Empty, Scene, 250
                )
                if executed:
                    return True
            except Exception:
                pass
            self.stop_requested.wait(0.5)
        return False

    def run(self) -> int:
        logging.info(
            "streamer world=%s vehicle_grid=%d gui_grid=%d textures=%s mesh_dir=%s limit=%d",
            self.args.world,
            self.args.vehicle_grid_side,
            self.args.gui_grid_side,
            self.args.texture_mode,
            self.mesh_dir,
            self.args.loaded_tile_limit,
        )
        if not self._wait_for_world_services():
            logging.error("Gazebo create/remove services for world %s did not become ready", self.args.world)
            return 1
        if self.args.clear_previous:
            logging.info("clearing possible tiles from an interrupted previous session")
            self._clear_previous_tiles()

        while not self.stop_requested.is_set():
            try:
                self._reconcile(self._desired_tiles())
            except RuntimeError as exc:
                logging.error("%s", exc)
                return 2
            self.stop_requested.wait(1.0 / self.args.update_hz)

        if self.args.remove_on_exit:
            logging.info("removing streamed tiles on exit")
            for tile in sorted(self.loaded):
                self._remove(tile)
        return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True)
    parser.add_argument("--vehicle-grid-side", type=int, default=0)
    parser.add_argument("--gui-grid-side", type=int, default=0)
    parser.add_argument("--vehicle-model", default="")
    parser.add_argument("--vehicle-x", type=float, default=105.718)
    parser.add_argument("--vehicle-y", type=float, default=-412.481)
    parser.add_argument("--gui-x", type=float, default=105.718)
    parser.add_argument("--gui-y", type=float, default=-412.481)
    parser.add_argument("--gui-pose-topic", default="/burniy_stream/gui_pose")
    parser.add_argument("--model-name", default="burniy_airfield_chunked")
    parser.add_argument(
        "--model-root",
        default="/home/sverk/PX4-Autopilot/Tools/simulation/gz/models/burniy_airfield_chunked",
    )
    parser.add_argument("--texture-mode", choices=("on", "off"), default="on")
    parser.add_argument(
        "--mesh-dir",
        choices=("auto", "meshes_norm", "meshes_glb", "meshes_norm_untextured", "meshes_glb_untextured"),
        default="auto",
    )
    parser.add_argument("--update-hz", type=float, default=4.0)
    parser.add_argument("--unload-grace-seconds", type=float, default=2.0)
    parser.add_argument("--service-timeout-ms", type=int, default=500)
    parser.add_argument("--startup-timeout-seconds", type=float, default=90.0)
    parser.add_argument(
        "--loaded-tile-limit",
        type=int,
        default=0,
        help="0 derives a transition-safe limit from the selected grid sizes.",
    )
    parser.add_argument("--clear-previous", action="store_true")
    parser.add_argument("--remove-on-exit", action="store_true")
    parser.add_argument("--log-level", default="INFO", choices=("DEBUG", "INFO", "WARNING", "ERROR"))
    args = parser.parse_args()

    for option in ("vehicle_grid_side", "gui_grid_side"):
        value = getattr(args, option)
        if not 0 <= value <= GRID_COUNT:
            parser.error(f"--{option.replace('_', '-')} must be between 0 and {GRID_COUNT}")
    if not args.vehicle_grid_side and not args.gui_grid_side:
        parser.error("at least one grid side must be positive")
    if args.vehicle_grid_side and not args.vehicle_model:
        parser.error("--vehicle-model is required when --vehicle-grid-side is positive")
    if not math.isfinite(args.update_hz) or args.update_hz <= 0:
        parser.error("--update-hz must be positive")
    if not math.isfinite(args.unload_grace_seconds) or args.unload_grace_seconds < 0:
        parser.error("--unload-grace-seconds cannot be negative")
    for option in ('vehicle_x', 'vehicle_y', 'gui_x', 'gui_y', 'startup_timeout_seconds'):
        if not math.isfinite(getattr(args, option)):
            parser.error(f"{option} must be finite")
    if args.service_timeout_ms <= 0 or args.startup_timeout_seconds <= 0:
        parser.error("Service and startup timeouts must be positive")
    safe_limit = min(GRID_COUNT ** 2, sum(side ** 2 + 2 * side - 1 for side in
                     (args.vehicle_grid_side, args.gui_grid_side) if side))
    if args.loaded_tile_limit and args.loaded_tile_limit < safe_limit:
        parser.error(f"--loaded-tile-limit must be at least {safe_limit} for safe transitions")
    if args.loaded_tile_limit == 0:
        # Desired union plus one incoming row/column for every independent
        # window. A diagonal shift adds (2 * side - 1) tiles per window.
        transition_margin = sum(
            2 * side - 1
            for side in (args.vehicle_grid_side, args.gui_grid_side)
            if side
        )
        args.loaded_tile_limit = (
            args.vehicle_grid_side ** 2
            + args.gui_grid_side ** 2
            + transition_margin
        )
    if args.loaded_tile_limit < max(args.vehicle_grid_side ** 2, args.gui_grid_side ** 2):
        parser.error("--loaded-tile-limit is smaller than an enabled individual grid")
    return args


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(message)s",
    )
    streamer = TileStreamer(args)

    def request_stop(_signum, _frame):
        # Python logging is not reentrant and is unsafe inside a signal handler.
        streamer.stop_requested.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    return streamer.run()


if __name__ == "__main__":
    sys.exit(main())
