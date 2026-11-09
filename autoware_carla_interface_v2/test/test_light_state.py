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

"""apply_light_state sends combined blinker flags without building an enum member."""

import threading

from autoware_carla_interface_v2.carla_ros import HazardLightsCommand
from autoware_carla_interface_v2.carla_ros import TurnIndicatorsCommand
from autoware_carla_interface_v2.carla_ros import carla_ros2_interface
from autoware_carla_interface_v2.modules.carla_client import carla

LEFT = int(carla.VehicleLightState.LeftBlinker)
RIGHT = int(carla.VehicleLightState.RightBlinker)
POSITION = int(carla.VehicleLightState.Position)


class _Ego:
    def __init__(self, state):
        self.state = state
        self.sent = []

    def get_light_state(self):
        return self.state

    def set_light_state(self, state):
        self.sent.append(state)


def _bridge(ego, turn, hazard):
    bridge = carla_ros2_interface.__new__(carla_ros2_interface)
    bridge._state_lock = threading.Lock()
    bridge.ego_actor = ego
    bridge.current_turn_indicator = turn
    bridge.current_hazard_lights = hazard
    return bridge


def test_hazard_sets_both_blinkers_and_keeps_other_lights():
    ego = _Ego(POSITION)
    _bridge(ego, TurnIndicatorsCommand.DISABLE, HazardLightsCommand.ENABLE).apply_light_state()
    assert ego.sent == [POSITION | LEFT | RIGHT]
    assert type(ego.sent[0]) is int


def test_left_indicator_clears_the_right_blinker():
    ego = _Ego(POSITION | RIGHT)
    _bridge(ego, TurnIndicatorsCommand.ENABLE_LEFT, HazardLightsCommand.DISABLE).apply_light_state()
    assert ego.sent == [POSITION | LEFT]


def test_no_command_turns_the_blinkers_off():
    ego = _Ego(LEFT | RIGHT)
    _bridge(ego, TurnIndicatorsCommand.DISABLE, HazardLightsCommand.DISABLE).apply_light_state()
    assert ego.sent == [0]
