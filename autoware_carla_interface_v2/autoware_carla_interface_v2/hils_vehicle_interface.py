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

"""Autoware as the ECU under test of autoware_carla_scenario's HILS bridge.

``scenario-hils`` (autoware_carla_scenario, docs/hils.md) follows the ego of a
scenario run and sends what a vehicle's buses and sensors would: here the
vehicle CAN described by ``config/hils/carla_autoware.dbc`` and a Velodyne
VLP-16's UDP packets, which Autoware's own LiDAR driver (nebula) decodes. This
node is the vehicle interface on Autoware's side of that CAN, as it would be in
a car:

- ``VehicleStatus``, ``ImuData``, ``ImuRates``, ``GnssPosition`` / ``GnssAttitude`` (the
  simulated GNSS: the ego's pose) become ``/vehicle/status/*``, the IMU and the
  GNSS pose Autoware localizes from;
- ``/control/command/control_cmd`` goes back as ``ControlCmd``, Autoware's
  target speed, acceleration and tire angle, which the bridge applies to the
  ego as CARLA Ackermann control (``commands.mode: ackermann``);
- ``PowerMode.Ignition`` is the scenario's lifecycle. When it comes on the node
  answers ``EcuStatus.Ready`` as soon as Autoware's AD API is up -- the
  bridge's ``scenario_start`` step waits for it, and the bridge attaches its
  sensors only after that step, so the ECU is ready before it sees any sensor
  data, as a car's ECUs are awake before they drive. Then, with the LiDAR
  streaming, it initializes localization at the GNSS pose, sets the route to
  ``goal`` and engages; ``EcuStatus.Engaged`` says Autoware drives (autonomous
  mode), which the bridge's ``warmup`` step waits for before the scenario's
  clock starts. When the ignition goes off the route is cleared and Autoware
  stopped.

Nothing here talks to CARLA: the node only sees the CAN, as the ECU would.
"""

from dataclasses import dataclass
import math
import os
import threading
import time
from typing import Callable
from typing import Optional
from typing import Tuple

from .transforms import yaw_to_quaternion
from .vehicle_interface import GEAR_DRIVE
from .vehicle_interface import ackermann_values
from .vehicle_interface import reported_gear

GRAVITY = 9.80665
#: CARLA's Lincoln MKZ turns its front wheels 70 degrees at full steer.
MKZ_MAX_STEER_ANGLE = math.radians(70.0)


# ------------------------------------------------------------ conversions
def map_pose_from_carla(x: float, y: float, z: float, yaw_deg: float, base_link_x: float) -> Tuple[float, float, float, float]:
    """The pose of ``base_link`` in Autoware's map frame, from CARLA's ego pose.

    CARLA is left-handed (y to the right, yaw clockwise); the map frame is
    right-handed, so y and yaw change sign. ``base_link_x`` is where base_link
    sits along the vehicle's x from CARLA's vehicle origin (negative: behind).
    """
    yaw = -math.radians(yaw_deg)
    map_x = x + base_link_x * math.cos(yaw)
    map_y = -y + base_link_x * math.sin(yaw)
    return map_x, map_y, z, yaw


def tire_angle_from_steer(steer: float, max_steer_angle: float = MKZ_MAX_STEER_ANGLE) -> float:
    """Autoware's steering tire angle (rad, left-positive) from CARLA's steer (right-positive)."""
    return -float(steer) * max_steer_angle


def control_cmd_fields(steering_tire_angle, velocity, acceleration, jerk, gear) -> dict:
    """Autoware's Control (+ gear) as the DBC's ``ControlCmd`` fields.

    The same Ackermann mapping the client-side interface applies itself
    (``ackermann_values``), with the tire angle kept left-positive on the bus:
    the bridge's input negates it into CARLA's right-positive steer.
    """
    values = ackermann_values(steering_tire_angle, velocity, acceleration, jerk, gear)
    return {
        "TargetSpeed": values["speed"],
        "Accel": values["acceleration"],
        "SteerAngle": -values["steer"],
        "Jerk": values["jerk"],
    }


@dataclass
class EgoState:
    speed: float = 0.0
    steer: float = 0.0
    reverse: int = 0
    yaw_rate: float = 0.0
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    accel: Tuple[float, float, float] = (0.0, 0.0, GRAVITY)
    position: Optional[Tuple[float, float]] = None
    z: float = 0.0
    yaw_deg: Optional[float] = None


# ------------------------------------------------------------------ node
class Settle:
    """True once a condition has held without a break for ``hold`` seconds."""

    def __init__(self, hold: float):
        self.hold = hold
        self.since: Optional[float] = None

    def update(self, ok: bool, now: float) -> bool:
        if not ok:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= self.hold


def _package_file(*parts) -> str:
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory("autoware_carla_interface_v2"), *parts)


def main(args=None):
    import hilspy
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy

    from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
    from autoware_adapi_v1_msgs.msg import OperationModeState
    from autoware_adapi_v1_msgs.msg import RouteState
    from autoware_adapi_v1_msgs.srv import ChangeOperationMode
    from autoware_adapi_v1_msgs.srv import ClearRoute
    from autoware_adapi_v1_msgs.srv import InitializeLocalization
    from autoware_adapi_v1_msgs.srv import SetRoutePoints
    from autoware_control_msgs.msg import Control
    from autoware_vehicle_msgs.msg import ControlModeReport
    from autoware_vehicle_msgs.msg import GearCommand
    from autoware_vehicle_msgs.msg import GearReport
    from autoware_vehicle_msgs.msg import HazardLightsReport
    from autoware_vehicle_msgs.msg import SteeringReport
    from autoware_vehicle_msgs.msg import TurnIndicatorsReport
    from autoware_vehicle_msgs.msg import VelocityReport
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from sensor_msgs.msg import Imu

    from .sensor_kit import load_sensor_kit

    class HilsVehicleInterface(Node):
        def __init__(self):
            super().__init__("hils_vehicle_interface")
            p = self.declare_parameter
            self.can_url = p("can_url", "can://vcan0").value
            self.dbc_file = p("dbc_file", "").value or _package_file("config", "hils", "carla_autoware.dbc")
            sensor_kit_file = p("sensor_kit_file", "").value or _package_file("config", "sensor_kit.yaml")
            self.base_link_x = load_sensor_kit(sensor_kit_file).base_link.x
            self.max_steer_angle = p("max_steer_angle", MKZ_MAX_STEER_ANGLE).value
            self.command_rate = p("command_rate_hz", 50.0).value
            self.goal = (p("goal_x", 0.0).value, p("goal_y", 0.0).value, p("goal_yaw", 0.0).value)
            self.has_goal = p("goal_set", False).value
            self.auto_engage = p("auto_engage", True).value
            self.ignition_timeout = p("ignition_timeout_s", 2.0).value
            self.status_rate = p("status_rate_hz", 50.0).value
            self.step_timeout = p("step_timeout_s", 120.0).value
            # Engage only once the controller's command has been flowing and
            # autonomous mode has stayed available this long: right after
            # localization the mode flickers available while the controller is
            # still silent, and engaging then times the command source out
            # into an MRM.
            self.engage_settle = p("engage_settle_s", 2.0).value

            self.state = EgoState()
            self.lock = threading.Lock()
            self.control: Optional[Control] = None
            self.gear_cmd = GEAR_DRIVE
            self.ignition = False
            self.ignition_seen = 0.0
            self.ready = False
            self.localization_state = 0
            self.route_state = 0
            self.operation_mode: Optional[OperationModeState] = None
            self.follower_seen = 0.0  # monotonic time of the last trajectory follower command
            self.session = 0  # bumped at every ignition edge; a stale operator gives up

            group = ReentrantCallbackGroup()
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            pub = lambda t, name: self.create_publisher(t, name, qos)  # noqa: E731
            self.pub_velocity = pub(VelocityReport, "/vehicle/status/velocity_status")
            self.pub_steering = pub(SteeringReport, "/vehicle/status/steering_status")
            self.pub_gear = pub(GearReport, "/vehicle/status/gear_status")
            self.pub_mode = pub(ControlModeReport, "/vehicle/status/control_mode")
            self.pub_turn = pub(TurnIndicatorsReport, "/vehicle/status/turn_indicators_status")
            self.pub_hazard = pub(HazardLightsReport, "/vehicle/status/hazard_lights_status")
            self.pub_imu = self.create_publisher(Imu, "/sensing/imu/tamagawa/imu_raw", 10)
            self.pub_gnss = pub(PoseWithCovarianceStamped, "/sensing/gnss/pose_with_covariance")

            self.create_subscription(Control, "/control/command/control_cmd", self._on_control, qos, callback_group=group)
            self.create_subscription(GearCommand, "/control/command/gear_cmd", self._on_gear, qos, callback_group=group)
            self.create_subscription(
                Control, "/control/trajectory_follower/control_cmd", self._on_follower, qos, callback_group=group
            )
            self.create_subscription(
                LocalizationInitializationState, "/api/localization/initialization_state",
                lambda m: setattr(self, "localization_state", m.state), latched, callback_group=group)
            self.create_subscription(
                RouteState, "/api/routing/state", lambda m: setattr(self, "route_state", m.state), latched,
                callback_group=group)
            self.create_subscription(
                OperationModeState, "/api/operation_mode/state", lambda m: setattr(self, "operation_mode", m),
                latched, callback_group=group)

            self.srv_initialize = self.create_client(InitializeLocalization, "/api/localization/initialize", callback_group=group)
            self.srv_route = self.create_client(SetRoutePoints, "/api/routing/set_route_points", callback_group=group)
            self.srv_clear = self.create_client(ClearRoute, "/api/routing/clear_route", callback_group=group)
            self.srv_autonomous = self.create_client(ChangeOperationMode, "/api/operation_mode/change_to_autonomous", callback_group=group)
            self.srv_stop = self.create_client(ChangeOperationMode, "/api/operation_mode/change_to_stop", callback_group=group)

            self.channel = hilspy.open(self.can_url, f"dbc:{self.dbc_file}")
            self.get_logger().info(f"HILS CAN {self.can_url} ({self.dbc_file})")
            self.running = True
            self.receiver = threading.Thread(target=self._receive, daemon=True)
            self.receiver.start()
            self.create_timer(1.0 / self.command_rate, self._send_command, callback_group=group)
            self.create_timer(0.1, self._send_status, callback_group=group)
            # The CAN carries VehicleStatus and ImuData at 100 Hz: the receiver
            # only stores them, and Autoware gets them at these rates, so the
            # receiver keeps up and PowerMode is never read late.
            self.create_timer(1.0 / self.status_rate, self._publish_vehicle_status, callback_group=group)
            self.create_timer(0.1, self._publish_gnss, callback_group=group)
            self.imu_fresh = False

        # ------------------------------------------------------- Autoware -> CAN
        def _on_control(self, msg):
            self.control = msg

        def _on_gear(self, msg):
            self.gear_cmd = msg.command

        def _send_command(self):
            control = self.control
            if control is None or not self.ignition:
                return
            fields = control_cmd_fields(
                control.lateral.steering_tire_angle, control.longitudinal.velocity,
                control.longitudinal.acceleration, control.longitudinal.jerk, self.gear_cmd)
            self._send("ControlCmd", fields)

        def _send_status(self):
            mode = self.operation_mode
            engaged = bool(
                mode and mode.mode == OperationModeState.AUTONOMOUS and mode.is_autoware_control_enabled)
            self._send("EcuStatus", {"Ready": int(self.ready and self.ignition), "Engaged": int(engaged)})
            if self.ignition and time.monotonic() - self.ignition_seen > self.ignition_timeout:
                self._set_ignition(False)

        def _send(self, name, fields):
            try:
                self.channel.send(name, fields)
            except Exception as error:  # noqa: BLE001 - the bus may be down; keep running
                self.get_logger().warning(f"CAN send {name} failed: {error}", throttle_duration_sec=5.0)

        # ------------------------------------------------------- CAN -> Autoware
        def _receive(self):
            while self.running:
                try:
                    message = self.channel.recv(timeout=0.2)
                except Exception as error:  # noqa: BLE001
                    self.get_logger().error(f"CAN receive failed: {error}", throttle_duration_sec=5.0)
                    time.sleep(0.5)
                    continue
                if message is not None:
                    self._dispatch(message.name, message.fields)

        def _dispatch(self, name, fields):
            s = self.state
            if name == "VehicleStatus":
                s.speed, s.steer, s.reverse = fields["Speed"], fields["Steer"], int(fields["Reverse"])
            elif name == "ImuData":
                s.yaw_rate = fields["YawRate"]
                s.accel = (fields["AccelX"], fields["AccelY"], fields["AccelZ"])
                self.imu_fresh = True
            elif name == "ImuRates":
                s.roll_rate, s.pitch_rate = fields["RollRate"], fields["PitchRate"]
            elif name == "GnssPosition":
                s.position = (fields["X"], fields["Y"])
            elif name == "GnssAttitude":
                s.z, s.yaw_deg = fields["Z"], fields["Yaw"]
            elif name == "PowerMode":
                if fields["Ignition"]:
                    self.ignition_seen = time.monotonic()
                self._set_ignition(bool(fields["Ignition"]))

        def _publish_imu(self, stamp):
            s = self.state
            if not self.imu_fresh:
                return
            self.imu_fresh = False
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = stamp, "base_link"
            imu.orientation_covariance[0] = -1.0  # no orientation estimate
            imu.angular_velocity.x = s.roll_rate
            imu.angular_velocity.y = s.pitch_rate
            imu.angular_velocity.z = s.yaw_rate
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = s.accel
            for i in (0, 4, 8):
                imu.angular_velocity_covariance[i] = 1e-4
                imu.linear_acceleration_covariance[i] = 1e-2
            self.pub_imu.publish(imu)

        def _publish_vehicle_status(self):
            s = self.state
            stamp = self.get_clock().now().to_msg()
            self._publish_imu(stamp)
            velocity = VelocityReport()
            velocity.header.stamp, velocity.header.frame_id = stamp, "base_link"
            velocity.longitudinal_velocity = float(s.speed)
            velocity.heading_rate = float(s.yaw_rate)
            self.pub_velocity.publish(velocity)
            steering = SteeringReport(stamp=stamp)
            steering.steering_tire_angle = tire_angle_from_steer(s.steer, self.max_steer_angle)
            self.pub_steering.publish(steering)
            self.pub_gear.publish(GearReport(stamp=stamp, report=reported_gear(self.gear_cmd, bool(s.reverse) or s.speed < -0.1)))
            self.pub_mode.publish(ControlModeReport(stamp=stamp, mode=ControlModeReport.AUTONOMOUS))
            self.pub_turn.publish(TurnIndicatorsReport(stamp=stamp, report=TurnIndicatorsReport.DISABLE))
            self.pub_hazard.publish(HazardLightsReport(stamp=stamp, report=HazardLightsReport.DISABLE))

        def _gnss_pose(self, stamp):
            s = self.state
            if s.position is None or s.yaw_deg is None:
                return None
            x, y, z, yaw = map_pose_from_carla(s.position[0], s.position[1], s.z, s.yaw_deg, self.base_link_x)
            pose = PoseWithCovarianceStamped()
            pose.header.stamp, pose.header.frame_id = stamp, "map"
            pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z = x, y, z
            pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = yaw_to_quaternion(yaw)
            for i, value in zip((0, 7, 14, 21, 28, 35), (0.04, 0.04, 0.1, 0.01, 0.01, 0.01)):
                pose.pose.covariance[i] = value
            return pose

        def _publish_gnss(self):
            pose = self._gnss_pose(self.get_clock().now().to_msg())
            if pose is not None:
                self.pub_gnss.publish(pose)

        # ------------------------------------------------------------ lifecycle
        def _set_ignition(self, on):
            if on == self.ignition:
                return
            self.ignition = on
            self.session += 1
            self.ready = False
            self.get_logger().info(f"Ignition {'on: starting Autoware' if on else 'off: stopping Autoware'}")
            target = self._start_session if on else self._end_session
            threading.Thread(target=target, args=(self.session,), daemon=True).start()

        def _call(self, client, request, what, session):
            deadline = time.monotonic() + self.step_timeout
            while time.monotonic() < deadline and session == self.session:
                if not client.wait_for_service(timeout_sec=1.0):
                    continue
                future = client.call_async(request)
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(0.05)
                response = future.result() if future.done() else None
                if response is not None and response.status.success:
                    self.get_logger().info(f"{what} accepted")
                    return True
                message = response.status.message if response is not None else "timed out"
                self.get_logger().info(f"{what} rejected ({message}); retrying", throttle_duration_sec=5.0)
                time.sleep(1.0)
            return False

        def _wait(self, predicate: Callable[[], bool], session) -> bool:
            deadline = time.monotonic() + self.step_timeout
            while time.monotonic() < deadline and session == self.session:
                if predicate():
                    return True
                time.sleep(0.1)
            return False

        def _on_follower(self, _msg):
            self.follower_seen = time.monotonic()

        def _engage_ready(self, now: float) -> bool:
            available = bool(self.operation_mode and self.operation_mode.is_autonomous_mode_available)
            return available and now - self.follower_seen < 0.5

        def _wait_settled(self, predicate: Callable[[float], bool], hold: float, session) -> bool:
            """Wait until ``predicate`` has held without a break for ``hold`` seconds."""
            settle = Settle(hold)
            return self._wait(lambda: settle.update(predicate(now := time.monotonic()), now), session)

        def _start_session(self, session):
            # Awake: the AD API answers. The bridge attaches the sensors after
            # this, so localization can only start once Ready has gone out.
            services = (self.srv_initialize, self.srv_route, self.srv_autonomous)
            if not self._wait(lambda: all(c.service_is_ready() for c in services), session):
                self.get_logger().error("Autoware's AD API is not up")
                return
            self.ready = True
            self.get_logger().info("Autoware awake: EcuStatus.Ready = 1")
            if not self._wait(lambda: self.state.position is not None and self.state.yaw_deg is not None, session):
                self.get_logger().error("No GNSS pose on the CAN; cannot initialize localization")
                return
            request = InitializeLocalization.Request()
            pose = self._gnss_pose(self.get_clock().now().to_msg())
            request.pose = [pose]
            if not self._call(self.srv_initialize, request, "localization initialize", session):
                return
            initialized = LocalizationInitializationState.INITIALIZED
            if not self._wait(lambda: self.localization_state == initialized, session):
                self.get_logger().error("Localization did not initialize")
                return
            if self.has_goal:
                if self.route_state != RouteState.UNSET:
                    self._call(self.srv_clear, ClearRoute.Request(), "clear route", session)
                route = SetRoutePoints.Request()
                route.header.frame_id = "map"
                route.header.stamp = self.get_clock().now().to_msg()
                route.goal.position.x, route.goal.position.y = self.goal[0], self.goal[1]
                route.goal.orientation.z, route.goal.orientation.w = yaw_to_quaternion(self.goal[2])
                if not self._call(self.srv_route, route, "route set", session):
                    return
            if self.auto_engage:
                if not self._wait_settled(self._engage_ready, self.engage_settle, session):
                    self.get_logger().error("Autonomous mode never became available")
                    return
                if not self._call(self.srv_autonomous, ChangeOperationMode.Request(), "change to autonomous", session):
                    return
            if session == self.session:
                self.get_logger().info("Autoware engaged")

        def _end_session(self, session):
            self._call(self.srv_stop, ChangeOperationMode.Request(), "change to stop", session)
            if self.route_state not in (RouteState.UNSET, 0):
                self._call(self.srv_clear, ClearRoute.Request(), "clear route", session)

        def close(self):
            self.running = False
            self.receiver.join(timeout=1.0)
            self.channel.close()

    rclpy.init(args=args)
    node = HilsVehicleInterface()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
