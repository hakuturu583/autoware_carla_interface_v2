# Copyright 2026 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""rclpy node that runs an Autoware session on CARLA 1.0's native ROS 2 interface.

CARLA 1.0 talks to Autoware without a bridge: the server publishes the sensor
data, ``/vehicle/status/*`` and ``/clock``, and applies
``/control/command/control_cmd`` to a vehicle spawned with
``ros2_autoware_control``. This node does the client side that is left:

* connects through typesafe_carla (LibCarla 1.0) and loads the level;
* applies synchronous mode and fixed time step, and ticks the world at a
  real-time pace from its own thread (it is the only client that ticks);
* spawns the ego vehicle with ``ros2_autoware_control``, the
  ``sensor.other.vehicle_status`` sensor and the sensor kit from
  ``config/sensor_kit.yaml`` with each sensor's ``ros_name`` /
  ``ros_topic_name`` and ``enable_for_ros()``;
* applies the turn-indicator and hazard commands, which the server receives
  but does not apply;
* publishes the simulator's traffic-light states on Autoware's external
  traffic-signal input (optional);
* moves the ego to an ``/initialpose`` set in RViz;
* destroys what it spawned and restores the world settings on shutdown.

With ``scenario_mode`` it runs beside autoware_carla_scenario, which owns the
level, the ego (role name ``Ego``) and the clock: the node waits for that ego,
spawns the sensor kit on it and neither loads, ticks nor teleports. An ego that
was not spawned for CARLA's native Autoware interface (``ros2_autoware_control``
and the role name ``ego`` / ``hero``) gets Autoware's vehicle interface from
:mod:`.vehicle_interface` instead.
"""

import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from autoware_perception_msgs.msg import TrafficLightElement
from autoware_perception_msgs.msg import TrafficLightGroup
from autoware_perception_msgs.msg import TrafficLightGroupArray
from autoware_vehicle_msgs.msg import HazardLightsCommand
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from .carla_client import CARLA_REF
from .carla_client import as_sensor
from .carla_client import as_traffic_light
from .carla_client import as_vehicle
from .carla_client import carla
from . import traffic_lights
from .sensor_kit import load_sensor_kit
from .traffic_light_matcher import parse_id_map_override
from .transforms import Pose
from .transforms import quaternion_to_rpy
from .vehicle_interface import ClientSideVehicleInterface

EGO_ROLE_NAMES = ("ego", "hero")


def to_carla_transform(pose: Pose):
    """A ROS-convention Pose as a carla.Transform."""
    (x, y, z), (roll, pitch, yaw) = pose.to_carla_values()
    return carla.Transform(
        carla.Location(x=x, y=y, z=z), carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)
    )


def from_carla_transform(transform) -> Pose:
    """A carla.Transform as a ROS-convention Pose."""
    location, rotation = transform.location, transform.rotation
    return Pose.from_carla_values(
        (location.x, location.y, location.z), (rotation.roll, rotation.pitch, rotation.yaw)
    )


def light_state_with_commands(current, turn_command, hazard_command):
    """The vehicle light bits with the blinkers set from Autoware's commands.

    Hazard wins over the turn indicator; other bits (position, brake, ...) are
    kept. Returns an int: typesafe_carla's VehicleLightState is an IntEnum
    whose members are single flags, so a combination is not a member.
    """
    left = int(carla.VehicleLightState.LeftBlinker)
    right = int(carla.VehicleLightState.RightBlinker)
    state = int(current) & ~left & ~right
    if hazard_command == HazardLightsCommand.ENABLE:
        state |= left | right
    elif turn_command == TurnIndicatorsCommand.ENABLE_LEFT:
        state |= left
    elif turn_command == TurnIndicatorsCommand.ENABLE_RIGHT:
        state |= right
    return state


def parse_spawn_point(text):
    """'x,y,z,roll,pitch,yaw' in CARLA's convention (meters, degrees) -> carla.Transform."""
    values = [float(v) for v in text.replace(" ", "").split(",") if v != ""]
    if len(values) != 6:
        raise ValueError(f"spawn_point needs 6 comma-separated values, got '{text}'")
    x, y, z, roll, pitch, yaw = values
    return carla.Transform(
        carla.Location(x=x, y=y, z=z), carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)
    )


class CarlaAutowareNode(Node):
    """See the module docstring."""

    def __init__(self, **kwargs):
        super().__init__("autoware_carla_interface_v2", **kwargs)
        self._declare_parameters()
        self._client = None
        self._world = None
        self._original_settings = None
        self._ego = None
        self._spawned = []  # every actor this node spawned, ego last
        self._carla_lock = threading.Lock()
        self._running = threading.Event()
        self._tick_thread = None
        self._turn_command = TurnIndicatorsCommand.DISABLE
        self._hazard_command = HazardLightsCommand.DISABLE
        self._traffic_light_actors = None
        self._traffic_light_groups = None
        self._sensor_kit = None
        self._vehicle_interface = None
        self._spawned_ego = False

    # ------------------------------------------------------------------ setup
    def _declare_parameters(self):
        share = get_package_share_directory("autoware_carla_interface_v2")
        defaults = {
            "host": "localhost",
            "port": 2000,
            "timeout": 20.0,
            "carla_map": "",
            "force_load_world": False,
            "sync_mode": True,
            "fixed_delta_seconds": 0.05,
            "time_scale": 1.0,
            "resync": True,
            "no_rendering_mode": False,
            "publish_tf": False,
            # Attach to the ego, world and clock of autoware_carla_scenario
            # instead of providing them (see start()).
            "scenario_mode": False,
            "ego_wait_timeout": 60.0,
            # "" -> "Ego" in scenario mode (autoware_carla_scenario's name), "ego" otherwise.
            "ego_role_name": "",
            # auto: CARLA's native Autoware interface when the ego is registered
            # for it, the client-side one otherwise; native / client force one.
            "vehicle_interface": "auto",
            "vehicle_blueprint": "",
            "spawn_point": "",
            "spawn_index": -1,
            "sensor_kit_file": os.path.join(share, "config", "sensor_kit.yaml"),
            "map_origin_x": 0.0,
            "map_origin_y": 0.0,
            "initial_pose_z_offset": 0.5,
            "apply_light_commands": True,
            "traffic_light.publish": False,
            "traffic_light.topic": "/perception/traffic_light_recognition/external/traffic_signals",
            "traffic_light.rate_hz": 10.0,
            "traffic_light.map_path": "",
            "traffic_light.match_distance": 5.0,
            "traffic_light.match_ratio": 0.6,
            "traffic_light.id_map": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def param(self, name):
        return self.get_parameter(name).value

    def role_name(self):
        role = self.param("ego_role_name").strip()
        if role:
            return role
        return "Ego" if self.param("scenario_mode") else "ego"

    def start(self):
        """Connect, prepare the world and the ego, and start ticking.

        Standalone, the node owns the session: it loads the level, applies
        synchronous mode, spawns the ego and ticks the world. With
        ``scenario_mode`` (autoware_carla_scenario runs the session) it only
        attaches: it waits for the ego the scenario spawned, leaves the level,
        the settings and the clock alone, and spawns the sensor kit on that ego.
        """
        self._connect()
        self._sensor_kit = load_sensor_kit(self.param("sensor_kit_file"))
        if self.param("scenario_mode"):
            self._world = self._client.get_world()
            self._attach_ego()
        else:
            self._load_world()
            self._apply_settings()
            self._spawn_ego()
        self._spawn_base_link_and_status()
        self._spawn_sensors()
        self._create_interfaces()
        self._running.set()
        if not self.param("scenario_mode") and self.param("sync_mode"):
            self._tick_thread = threading.Thread(target=self._tick_loop, daemon=True)
            self._tick_thread.start()

    def _attach_ego(self):
        """Wait for the vehicle with our role name (spawned by the scenario)."""
        role = self.role_name()
        deadline = time.monotonic() + float(self.param("ego_wait_timeout"))
        self.get_logger().info(f"scenario_mode: waiting for the ego (role_name '{role}')")
        while True:
            for actor in self._world.get_actors().filter("vehicle.*"):
                if actor.attributes.get("role_name") == role:
                    self._ego = as_vehicle(actor)
                    self.get_logger().info(f"Attached to ego {actor.type_id} (id {actor.id})")
                    return
            if time.monotonic() >= deadline:
                raise RuntimeError(f"No vehicle with role_name '{role}' appeared")
            time.sleep(0.5)

    def _uses_native_vehicle_interface(self):
        """Whether CARLA itself serves Autoware's vehicle interface for the ego."""
        mode = self.param("vehicle_interface")
        if mode in ("native", "client"):
            return mode == "native"
        attributes = self._ego.attributes
        return (
            attributes.get("ros2_autoware_control") == "true"
            and attributes.get("role_name") in EGO_ROLE_NAMES
        )

    def _connect(self):
        host, port = self.param("host"), int(self.param("port"))
        self._client = carla.Client(host, port)
        self._client.set_timeout(float(self.param("timeout")))
        try:
            server = self._client.get_server_version()
        except RuntimeError as error:
            raise RuntimeError(f"No CARLA server answered at {host}:{port}: {error}") from error
        client = self._client.get_client_version()
        self.get_logger().info(f"CARLA server {server}, LibCarla (typesafe_carla) {client}")
        if ".".join(str(server).split("-")[0].split(".")[:2]) != CARLA_REF:
            self.get_logger().warning(
                f"This node targets CARLA {CARLA_REF}, but the server reports {server}"
            )

    def _load_world(self):
        wanted = self.param("carla_map").strip()
        if self.param("force_load_world") and wanted:
            self.get_logger().info(f"Loading '{wanted}'")
            self._client.load_world(wanted)
        elif wanted:
            # Returns the new world, or None when the level is already loaded.
            if self._client.load_world_if_different(wanted) is None:
                self.get_logger().info(f"'{wanted}' is already loaded")
            else:
                self.get_logger().info(f"Loaded '{wanted}'")
        self._world = self._client.get_world()
        self.get_logger().info(f"Level: {self._world.get_map().name}")

    def _apply_settings(self):
        self._original_settings = self._world.get_settings()
        settings = self._world.get_settings()
        settings.synchronous_mode = bool(self.param("sync_mode"))
        settings.fixed_delta_seconds = float(self.param("fixed_delta_seconds"))
        settings.no_rendering_mode = bool(self.param("no_rendering_mode"))
        self._world.apply_settings(settings)
        # Autoware publishes the vehicle's TF tree from its URDF.
        self._world.set_publish_tf(bool(self.param("publish_tf")))

    def _spawn_point(self):
        text = self.param("spawn_point").strip()
        if text:
            return parse_spawn_point(text)
        points = list(self._world.get_ego_spawn_points()) or list(
            self._world.get_map().get_spawn_points()
        )
        if not points:
            raise RuntimeError("The level has no spawn points; set spawn_point")
        index = int(self.param("spawn_index"))
        if index < 0:
            index = int.from_bytes(os.urandom(2), "little") % len(points)
        return points[index % len(points)]

    def _spawn(self, blueprint, transform, attach_to=None):
        actor = self._world.spawn_actor(blueprint, transform, attach_to=attach_to)
        self._spawned.append(actor)
        return actor

    def _spawn_ego(self):
        library = self._world.get_blueprint_library()
        name = self.param("vehicle_blueprint").strip() or self._sensor_kit.vehicle_blueprint
        blueprint = library.find(name)
        role = self.role_name()
        if role not in EGO_ROLE_NAMES:
            self.get_logger().warning(
                f"ego_role_name '{role}': CARLA only drives a vehicle named one of {EGO_ROLE_NAMES}"
            )
        blueprint.set_attribute("role_name", role)
        if blueprint.has_attribute("ros2_autoware_control"):
            blueprint.set_attribute("ros2_autoware_control", "true")
        else:
            self.get_logger().error(
                f"'{name}' has no ros2_autoware_control attribute: the server does not "
                "support Autoware control (CARLA 1.0 started with -ros2 is needed)"
            )
        spawn_point = self._spawn_point()
        actor = self._world.try_spawn_actor(blueprint, spawn_point)
        if actor is None:
            raise RuntimeError(f"Could not spawn '{name}' at {spawn_point}")
        self._ego = as_vehicle(actor)
        self._spawned_ego = True
        self.get_logger().info(f"Spawned ego '{name}' (id {actor.id}) at {spawn_point}")

    def _spawn_base_link_and_status(self):
        library = self._world.get_blueprint_library()

        if not self._uses_native_vehicle_interface():
            self.get_logger().info(
                "The ego is not registered for CARLA's Autoware interface: applying "
                "control_cmd (as Ackermann control) and publishing /vehicle/status/* here"
            )
            self._vehicle_interface = ClientSideVehicleInterface(
                self, self._world, self._ego, carla, self._carla_lock
            )
            return
        # The vehicle status sensor reports from the rear axle (base_link) and
        # publishes /vehicle/status/* through a global publisher.
        base_link = self._spawn(
            library.find("util.actor.empty"),
            to_carla_transform(self._sensor_kit.base_link),
            attach_to=self._ego,
        )
        self._spawn(library.find("sensor.other.vehicle_status"), carla.Transform(), base_link)

    def _spawn_sensors(self):
        library = self._world.get_blueprint_library()
        skip_rendered = bool(self.param("no_rendering_mode"))
        for spec in self._sensor_kit.sensors:
            if skip_rendered and spec.needs_rendering:
                # A renderer-less server (-nullrhi) crashes when a camera is spawned.
                self.get_logger().warning(f"no_rendering_mode: not spawning camera '{spec.name}'")
                continue
            try:
                blueprint = library.find(spec.blueprint)
            except (IndexError, RuntimeError):
                self.get_logger().error(f"Sensor '{spec.name}': no blueprint '{spec.blueprint}'")
                continue
            for key, value in spec.attributes.items():
                if blueprint.has_attribute(key):
                    blueprint.set_attribute(key, value)
                else:
                    self.get_logger().warning(f"Sensor '{spec.name}': no attribute '{key}'")
            blueprint.set_attribute("ros_name", spec.frame_id)
            if spec.topic:
                blueprint.set_attribute("ros_topic_name", spec.topic)
            transform = to_carla_transform(self._sensor_kit.sensor_pose_in_vehicle(spec))
            sensor = as_sensor(self._spawn(blueprint, transform, attach_to=self._ego))
            sensor.enable_for_ros()
            self.get_logger().info(f"Sensor '{spec.name}' ({spec.blueprint}) -> {spec.topic}")

    def _create_interfaces(self):
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        if self.param("apply_light_commands"):
            self.create_subscription(
                TurnIndicatorsCommand,
                "/control/command/turn_indicators_cmd",
                self._on_turn_indicators,
                qos,
            )
            self.create_subscription(
                HazardLightsCommand, "/control/command/hazard_lights_cmd", self._on_hazard, qos
            )
        if not self.param("scenario_mode"):
            # The scenario places its ego; a teleport would fight it.
            self.create_subscription(
                PoseWithCovarianceStamped, "/initialpose", self._on_initial_pose, 1
            )
        if self.param("traffic_light.publish"):
            self._traffic_light_pub = self.create_publisher(
                TrafficLightGroupArray, self.param("traffic_light.topic"), 1
            )
            self.create_timer(
                1.0 / max(float(self.param("traffic_light.rate_hz")), 0.1),
                self._publish_traffic_lights,
            )

    # ------------------------------------------------------------- the clock
    def _tick_loop(self):
        """Tick the synchronous world at fixed_delta_seconds / time_scale of wall time."""
        period = float(self.param("fixed_delta_seconds")) / max(
            float(self.param("time_scale")), 1e-3
        )
        resync = bool(self.param("resync"))
        next_tick = time.monotonic()
        while self._running.is_set():
            wait = next_tick - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            try:
                with self._carla_lock:
                    self._world.tick()
            except RuntimeError as error:
                self.get_logger().error(f"world.tick() failed: {error}", throttle_duration_sec=5.0)
            next_tick += period
            lag = time.monotonic() - next_tick
            if lag > period:
                self.get_logger().warning(
                    f"Simulation is {lag * 1000:.0f} ms behind real time",
                    throttle_duration_sec=10.0,
                )
                if resync:
                    next_tick = time.monotonic()

    # ------------------------------------------------------- vehicle commands
    def _on_turn_indicators(self, msg):
        self._turn_command = msg.command
        self._apply_lights()

    def _on_hazard(self, msg):
        self._hazard_command = msg.command
        self._apply_lights()

    def _apply_lights(self):
        if self._ego is None:
            return
        with self._carla_lock:
            current = self._ego.get_light_state()
            wanted = light_state_with_commands(current, self._turn_command, self._hazard_command)
            if wanted != int(current):
                self._ego.set_light_state(wanted)

    def _on_initial_pose(self, msg):
        """Move the ego so that base_link sits at the pose set in RViz."""
        if self._ego is None:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        _, _, yaw = quaternion_to_rpy(q.x, q.y, q.z, q.w)
        base_link = Pose(
            p.x - float(self.param("map_origin_x")),
            p.y - float(self.param("map_origin_y")),
            p.z,
            yaw=yaw,
        )
        vehicle = base_link.compose(self._sensor_kit.base_link.inverse())
        transform = to_carla_transform(vehicle)
        with self._carla_lock:
            ground = self._world.ground_projection(
                carla.Location(x=transform.location.x, y=transform.location.y, z=1000.0), 2000.0
            )
            if ground is not None:
                transform.location.z = ground.location.z
            transform.location.z += float(self.param("initial_pose_z_offset"))
            self._ego.set_target_velocity(carla.Vector3D())
            self._ego.set_target_angular_velocity(carla.Vector3D())
            self._ego.set_transform(transform)
        self.get_logger().info(f"Moved the ego to the initial pose: {transform}")

    # --------------------------------------------------------- traffic lights
    def _map_xy(self, x, y):
        return x + float(self.param("map_origin_x")), -y + float(self.param("map_origin_y"))

    def _publish_traffic_lights(self):
        with self._carla_lock:
            if self._traffic_light_actors is None:
                self._resolve_traffic_lights()
            states = traffic_lights.group_states(
                self._traffic_light_actors, self._traffic_light_groups
            )
        msg = TrafficLightGroupArray()
        msg.stamp = self.get_clock().now().to_msg()
        for group_id, (color, status) in sorted(states.items()):
            group = TrafficLightGroup()
            group.traffic_light_group_id = int(group_id)
            element = TrafficLightElement()
            element.color = color
            element.shape = TrafficLightElement.CIRCLE
            element.status = status
            element.confidence = 1.0
            group.elements.append(element)
            msg.traffic_light_groups.append(group)
        self._traffic_light_pub.publish(msg)

    def _resolve_traffic_lights(self):
        actors = self._world.get_actors().filter("traffic.traffic_light*")
        self._traffic_light_actors = [
            light for light in (as_traffic_light(a) for a in actors) if light is not None
        ]
        override = parse_id_map_override(
            self.param("traffic_light.id_map"),
            on_invalid=lambda message: self.get_logger().warning(
                f"traffic_light.id_map: {message}"
            ),
        )
        self._traffic_light_groups, result = traffic_lights.resolve_groups(
            self._traffic_light_actors,
            self._map_xy,
            map_path=self.param("traffic_light.map_path").strip(),
            override=override,
            match_distance=float(self.param("traffic_light.match_distance")),
            match_ratio=float(self.param("traffic_light.match_ratio")),
        )
        unmatched = len(self._traffic_light_actors) - len(self._traffic_light_groups)
        self.get_logger().info(
            f"Traffic lights: {len(self._traffic_light_groups)} of "
            f"{len(self._traffic_light_actors)} assigned to Autoware groups"
            + (f", {unmatched} unmatched (pin them with traffic_light.id_map)" if unmatched else "")
        )

    # --------------------------------------------------------------- shutdown
    def shutdown(self):
        """Stop ticking, destroy the spawned actors and restore the world settings."""
        self._running.clear()
        if self._tick_thread is not None:
            self._tick_thread.join(timeout=5.0)
        if self._vehicle_interface is not None:
            self._vehicle_interface.close()
            self._vehicle_interface = None
        if self._world is None:
            return
        for actor in reversed(self._spawned):
            sensor = as_sensor(actor)
            try:
                if sensor is not None:
                    sensor.disable_for_ros()
                actor.destroy()
            except RuntimeError as error:
                self.get_logger().warning(f"Could not destroy actor {actor.id}: {error}")
        if self._ego is not None and self._spawned_ego:
            try:
                self._ego.destroy()
            except RuntimeError as error:
                self.get_logger().warning(f"Could not destroy the ego: {error}")
        self._spawned = []
        self._ego = None
        if self._original_settings is not None:
            try:
                self._world.apply_settings(self._original_settings)
            except RuntimeError as error:
                self.get_logger().warning(f"Could not restore the world settings: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = CarlaAutowareNode()
    code = 0
    try:
        node.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError, OSError) as error:
        node.get_logger().fatal(str(error))
        code = 1
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
