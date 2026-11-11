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

"""The HILS vehicle interface's conversions between the CAN and Autoware."""

import math
import os

import pytest

from autoware_carla_interface_v2.hils_vehicle_interface import MKZ_MAX_STEER_ANGLE
from autoware_carla_interface_v2.hils_vehicle_interface import Settle
from autoware_carla_interface_v2.hils_vehicle_interface import control_cmd_fields
from autoware_carla_interface_v2.hils_vehicle_interface import gear_report
from autoware_carla_interface_v2.hils_vehicle_interface import map_pose_from_carla
from autoware_carla_interface_v2.hils_vehicle_interface import tire_angle_from_steer
from autoware_carla_interface_v2.vehicle_interface import GEAR_DRIVE
from autoware_carla_interface_v2.vehicle_interface import GEAR_REVERSE

DBC = os.path.join(os.path.dirname(__file__), "..", "config", "hils", "carla_autoware.dbc")


def test_map_pose_flips_y_and_yaw_and_moves_to_base_link():
    # Heading CARLA +y (yaw +90 deg, clockwise) is map -y (yaw -90 deg).
    x, y, z, yaw = map_pose_from_carla(10.0, 20.0, 0.5, 90.0, base_link_x=-1.5)
    assert yaw == pytest.approx(-math.pi / 2)
    # base_link is 1.5 m behind the origin, i.e. towards map +y here.
    assert (x, y, z) == pytest.approx((10.0, -20.0 + 1.5, 0.5))


def test_map_pose_at_zero_yaw_only_shifts_x():
    assert map_pose_from_carla(1.0, -2.0, 0.0, 0.0, -1.0) == pytest.approx((0.0, 2.0, 0.0, 0.0))


def test_tire_angle_is_left_positive_and_scaled_to_the_wheel_limit():
    assert tire_angle_from_steer(1.0) == pytest.approx(-MKZ_MAX_STEER_ANGLE)
    assert tire_angle_from_steer(-0.5, 1.0) == pytest.approx(0.5)


def test_control_cmd_keeps_the_tire_angle_left_positive_on_the_bus():
    fields = control_cmd_fields(0.2, 3.0, -1.5, 0.1, GEAR_DRIVE)
    assert fields == pytest.approx({"TargetSpeed": 3.0, "Accel": 1.5, "SteerAngle": 0.2, "Jerk": 0.1})


def test_control_cmd_reverses_the_target_speed_in_reverse():
    assert control_cmd_fields(0.0, 2.0, 0.5, 0.0, GEAR_REVERSE)["TargetSpeed"] == pytest.approx(-2.0)


def test_gear_report_follows_reverse_and_backward_motion():
    assert gear_report(0, 1.0) == GEAR_DRIVE
    assert gear_report(1, 0.0) == GEAR_REVERSE
    assert gear_report(0, -0.5) == GEAR_REVERSE


def test_the_dbc_round_trips_the_messages_the_bridge_and_the_node_exchange():
    hilspy = pytest.importorskip("hilspy")
    with hilspy.open("mem://hils-test", f"dbc:{DBC}") as tx, hilspy.open("mem://hils-test", f"dbc:{DBC}") as rx:
        tx.send("ControlCmd", control_cmd_fields(-0.1234, 4.2, 0.5, 0.0, GEAR_DRIVE))
        tx.send("GnssPosition", {"X": -107.39, "Y": 58.15})
        control = rx.recv(timeout=1.0)
        gnss = rx.recv(timeout=1.0)
    assert control.name == "ControlCmd"
    assert control.fields["SteerAngle"] == pytest.approx(-0.1234, abs=1e-4)
    assert control.fields["TargetSpeed"] == pytest.approx(4.2, abs=0.01)
    assert gnss.fields == pytest.approx({"X": -107.39, "Y": 58.15}, abs=1e-3)


def test_settle_needs_the_condition_to_hold_unbroken():
    settle = Settle(2.0)
    assert not settle.update(True, 10.0)
    assert not settle.update(True, 11.9)
    assert settle.update(True, 12.0)
    # A single break (the mode flickering unavailable) starts the wait over.
    assert not settle.update(False, 12.1)
    assert not settle.update(True, 12.2)
    assert not settle.update(True, 14.1)
    assert settle.update(True, 14.2)
