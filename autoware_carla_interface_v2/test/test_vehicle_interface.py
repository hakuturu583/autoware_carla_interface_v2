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

"""Client-side vehicle interface conversions (no CARLA, no ROS)."""

import math

import pytest

from autoware_carla_interface_v2.vehicle_interface import GEAR_DRIVE
from autoware_carla_interface_v2.vehicle_interface import GEAR_PARK
from autoware_carla_interface_v2.vehicle_interface import GEAR_REVERSE
from autoware_carla_interface_v2.vehicle_interface import ackermann_values
from autoware_carla_interface_v2.vehicle_interface import blinker_reports
from autoware_carla_interface_v2.vehicle_interface import steering_report_value
from autoware_carla_interface_v2.vehicle_interface import velocity_report_values


def test_control_becomes_ackermann_with_carla_steering_sign():
    values = ackermann_values(0.2, 5.0, -1.5, 0.3, GEAR_DRIVE)
    assert values == {
        "steer": pytest.approx(-0.2),
        "steer_speed": 0.0,
        "speed": 5.0,
        "acceleration": 1.5,
        "jerk": pytest.approx(0.3),
    }


def test_reverse_and_park_set_the_target_speed():
    assert ackermann_values(0.0, 2.0, 1.0, 0.0, GEAR_REVERSE)["speed"] == -2.0
    assert ackermann_values(0.0, -2.0, 1.0, 0.0, GEAR_REVERSE)["speed"] == -2.0
    assert ackermann_values(0.0, 2.0, 1.0, 0.0, GEAR_PARK)["speed"] == 0.0


def test_velocity_report_is_in_base_link():
    # Heading +y in CARLA (90 deg yaw): forward = (0, 1, 0), right = (-1, 0, 0).
    longitudinal, lateral, heading_rate = velocity_report_values(
        (0.5, 3.0, 0.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), 10.0
    )
    assert longitudinal == pytest.approx(3.0)
    assert lateral == pytest.approx(0.5)  # moving to CARLA -x = the vehicle's left
    assert heading_rate == pytest.approx(-math.radians(10.0))


def test_steering_report_averages_front_wheels_and_flips_sign():
    assert steering_report_value([10.0, 20.0]) == pytest.approx(-math.radians(15.0))
    assert steering_report_value([]) == 0.0


def test_blinker_reports():
    left, right = 0b01, 0b10
    assert blinker_reports(0, left, right) == (1, 1)
    assert blinker_reports(left, left, right) == (2, 1)
    assert blinker_reports(right, left, right) == (3, 1)
    assert blinker_reports(left | right, left, right) == (1, 2)
