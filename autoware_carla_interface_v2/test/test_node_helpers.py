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

"""Node helpers that need the CARLA client but no server."""

import math

from autoware_vehicle_msgs.msg import HazardLightsCommand
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
import pytest

from autoware_carla_interface_v2.carla_client import carla
from autoware_carla_interface_v2.node import from_carla_transform
from autoware_carla_interface_v2.node import light_state_with_commands
from autoware_carla_interface_v2.node import parse_spawn_point
from autoware_carla_interface_v2.node import to_carla_transform
from autoware_carla_interface_v2.transforms import Pose

LEFT = int(carla.VehicleLightState.LeftBlinker)
RIGHT = int(carla.VehicleLightState.RightBlinker)
POSITION = int(carla.VehicleLightState.Position)


def test_hazard_sets_both_blinkers_and_keeps_other_lights():
    state = light_state_with_commands(
        POSITION, TurnIndicatorsCommand.ENABLE_LEFT, HazardLightsCommand.ENABLE
    )
    assert state == POSITION | LEFT | RIGHT
    assert type(state) is int


def test_turn_indicator_replaces_the_other_blinker():
    state = light_state_with_commands(
        POSITION | RIGHT, TurnIndicatorsCommand.ENABLE_LEFT, HazardLightsCommand.DISABLE
    )
    assert state == POSITION | LEFT


def test_no_command_turns_the_blinkers_off():
    assert (
        light_state_with_commands(
            LEFT | RIGHT, TurnIndicatorsCommand.DISABLE, HazardLightsCommand.DISABLE
        )
        == 0
    )


def test_spawn_point_is_in_carla_coordinates():
    transform = parse_spawn_point("1, -2, 0.5, 0, 0, 90")
    assert (transform.location.x, transform.location.y, transform.location.z) == pytest.approx(
        (1.0, -2.0, 0.5)
    )
    assert transform.rotation.yaw == pytest.approx(90.0)
    with pytest.raises(ValueError):
        parse_spawn_point("1,2,3")


def test_carla_transform_round_trip():
    pose = Pose(1.0, 2.0, 0.3, 0.0, 0.0, math.pi / 3)
    transform = to_carla_transform(pose)
    assert transform.location.y == pytest.approx(-2.0)
    assert transform.rotation.yaw == pytest.approx(-60.0)
    back = from_carla_transform(transform)
    assert (back.x, back.y, back.z, back.yaw) == pytest.approx((1.0, 2.0, 0.3, math.pi / 3))
