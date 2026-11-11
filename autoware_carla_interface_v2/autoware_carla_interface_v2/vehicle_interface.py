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

"""Autoware's vehicle interface for an ego CARLA does not serve natively.

CARLA 1.0 applies ``/control/command/control_cmd`` and publishes
``/vehicle/status/*`` only for a vehicle spawned with the
``ros2_autoware_control`` attribute and the role name ``ego`` or ``hero``.
An ego spawned by someone else without them -- autoware_carla_scenario spawns
its ego as ``Ego`` -- gets neither. For such an ego this module does both from
the client: Autoware's ``Control`` is applied as CARLA Ackermann control (the
server turns target speed, acceleration and steering angle into pedals and
steering, so no actuation maps are needed), and the status reports are built
from the actor on every simulation frame.

The conversions are plain functions so they can be tested without CARLA.
"""

import math
from typing import Tuple

# autoware_vehicle_msgs gear values.
GEAR_DRIVE = 2
GEAR_REVERSE = 20
GEAR_PARK = 22
GEAR_NEUTRAL = 1



def reported_gear(commanded: int, reverse: bool) -> int:
    """The GearReport for the commanded gear: reverse when backing, park when parked, else drive."""
    return GEAR_REVERSE if reverse else (commanded if commanded == GEAR_PARK else GEAR_DRIVE)

def ackermann_values(steering_tire_angle, velocity, acceleration, jerk, gear) -> dict:
    """Autoware Control (+ gear) as VehicleAckermannControl fields.

    Autoware steers counter-clockwise positive, CARLA clockwise positive.
    In reverse the target speed is negative; in park or neutral the target is
    a standstill.
    """
    speed = abs(float(velocity))
    if gear == GEAR_REVERSE:
        speed = -speed
    elif gear in (GEAR_PARK, GEAR_NEUTRAL):
        speed = 0.0
    return {
        "steer": -float(steering_tire_angle),
        "steer_speed": 0.0,
        "speed": speed,
        "acceleration": abs(float(acceleration)),
        "jerk": abs(float(jerk)),
    }


def velocity_report_values(
    velocity, forward, right, angular_velocity_z_deg
) -> Tuple[float, float, float]:
    """(longitudinal, lateral, heading_rate) in base_link from CARLA world vectors.

    ``velocity``, ``forward`` and ``right`` are (x, y, z) in CARLA's world
    frame; CARLA's yaw rate is in deg/s and clockwise positive.
    """
    longitudinal = sum(v * f for v, f in zip(velocity, forward))
    lateral = -sum(v * r for v, r in zip(velocity, right))
    return longitudinal, lateral, -math.radians(angular_velocity_z_deg)


def steering_report_value(front_wheel_angles_deg) -> float:
    """Steering tire angle [rad] (CCW positive) from CARLA's front wheel angles [deg]."""
    angles = list(front_wheel_angles_deg)
    if not angles:
        return 0.0
    return -math.radians(sum(angles) / len(angles))


def blinker_reports(light_state, left_bit, right_bit) -> Tuple[int, int]:
    """(TurnIndicatorsReport, HazardLightsReport) values from CARLA light bits."""
    left = bool(int(light_state) & left_bit)
    right = bool(int(light_state) & right_bit)
    if left and right:
        return 1, 2  # DISABLE, hazard ENABLE
    if left:
        return 2, 1  # ENABLE_LEFT
    if right:
        return 3, 1  # ENABLE_RIGHT
    return 1, 1


class ClientSideVehicleInterface:
    """Subscribes to Autoware's commands and publishes the vehicle status for ``ego``."""

    def __init__(self, node, world, ego, carla, lock):
        from autoware_control_msgs.msg import Control
        from autoware_vehicle_msgs.msg import ControlModeReport
        from autoware_vehicle_msgs.msg import GearCommand
        from autoware_vehicle_msgs.msg import GearReport
        from autoware_vehicle_msgs.msg import HazardLightsReport
        from autoware_vehicle_msgs.msg import SteeringReport
        from autoware_vehicle_msgs.msg import TurnIndicatorsReport
        from autoware_vehicle_msgs.msg import VelocityReport
        from rclpy.qos import QoSProfile
        from rclpy.qos import ReliabilityPolicy

        self._node, self._world, self._ego, self._carla, self._lock = node, world, ego, carla, lock
        self._msgs = {
            "velocity": VelocityReport,
            "steering": SteeringReport,
            "gear": GearReport,
            "control_mode": ControlModeReport,
            "turn": TurnIndicatorsReport,
            "hazard": HazardLightsReport,
        }
        self._gear = GEAR_DRIVE
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        topics = {
            "velocity": "/vehicle/status/velocity_status",
            "steering": "/vehicle/status/steering_status",
            "gear": "/vehicle/status/gear_status",
            "control_mode": "/vehicle/status/control_mode",
            "turn": "/vehicle/status/turn_indicators_status",
            "hazard": "/vehicle/status/hazard_lights_status",
        }
        self._pubs = {k: node.create_publisher(self._msgs[k], t, qos) for k, t in topics.items()}
        node.create_subscription(Control, "/control/command/control_cmd", self._on_control, qos)
        node.create_subscription(GearCommand, "/control/command/gear_cmd", self._on_gear, qos)
        self._tick_id = world.on_tick(self._on_tick)
        self._left = int(carla.VehicleLightState.LeftBlinker)
        self._right = int(carla.VehicleLightState.RightBlinker)

    def close(self):
        if self._tick_id is not None:
            try:
                self._world.remove_on_tick(self._tick_id)
            except RuntimeError:
                pass
            self._tick_id = None

    def _on_gear(self, msg):
        self._gear = msg.command

    def _on_control(self, msg):
        values = ackermann_values(
            msg.lateral.steering_tire_angle,
            msg.longitudinal.velocity,
            msg.longitudinal.acceleration,
            msg.longitudinal.jerk,
            self._gear,
        )
        with self._lock:
            self._ego.apply_ackermann_control(self._carla.VehicleAckermannControl(**values))

    def _on_tick(self, snapshot):
        try:
            self._publish_status(snapshot.timestamp.elapsed_seconds)
        except RuntimeError as error:  # e.g. the ego was destroyed by the scenario
            self._node.get_logger().warning(
                f"Vehicle status skipped: {error}", throttle_duration_sec=5.0
            )

    def _publish_status(self, elapsed_seconds):
        from builtin_interfaces.msg import Time

        stamp = Time(sec=int(elapsed_seconds), nanosec=int((elapsed_seconds % 1.0) * 1e9))
        ego, carla = self._ego, self._carla
        transform = ego.get_transform()
        v = ego.get_velocity()
        f = transform.get_forward_vector()
        r = transform.get_right_vector()
        longitudinal, lateral, heading_rate = velocity_report_values(
            (v.x, v.y, v.z), (f.x, f.y, f.z), (r.x, r.y, r.z), ego.get_angular_velocity().z
        )
        velocity = self._msgs["velocity"]()
        velocity.header.stamp = stamp
        velocity.header.frame_id = "base_link"
        velocity.longitudinal_velocity = longitudinal
        velocity.lateral_velocity = lateral
        velocity.heading_rate = heading_rate
        self._pubs["velocity"].publish(velocity)

        steering = self._msgs["steering"](stamp=stamp)
        steering.steering_tire_angle = steering_report_value(
            ego.get_wheel_steer_angle(location)
            for location in (
                carla.VehicleWheelLocation.FL_Wheel,
                carla.VehicleWheelLocation.FR_Wheel,
            )
        )
        self._pubs["steering"].publish(steering)

        gear = reported_gear(self._gear, self._gear == GEAR_REVERSE or longitudinal < -0.1)
        self._pubs["gear"].publish(self._msgs["gear"](stamp=stamp, report=gear))
        self._pubs["control_mode"].publish(
            self._msgs["control_mode"](stamp=stamp, mode=self._msgs["control_mode"].AUTONOMOUS)
        )
        turn, hazard = blinker_reports(ego.get_light_state(), self._left, self._right)
        self._pubs["turn"].publish(self._msgs["turn"](stamp=stamp, report=turn))
        self._pubs["hazard"].publish(self._msgs["hazard"](stamp=stamp, report=hazard))
