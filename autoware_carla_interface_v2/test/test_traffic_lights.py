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

"""CARLA traffic-light states as Autoware groups (fake actors, no simulator)."""

from types import SimpleNamespace

from test_traffic_light_matcher import _OSM

from autoware_carla_interface_v2 import traffic_lights


class _Light:
    def __init__(self, actor_id, opendrive_id, x, y, state=traffic_lights.CARLA_RED, boxes=True):
        self.id = actor_id
        self._opendrive_id = opendrive_id
        self._state = state
        self._location = SimpleNamespace(x=x, y=y)
        self._boxes = [SimpleNamespace(location=SimpleNamespace(x=x, y=y))] if boxes else []

    def get_opendrive_id(self):
        return self._opendrive_id

    def get_state(self):
        return self._state

    def get_light_boxes(self):
        return self._boxes

    def get_location(self):
        return self._location


def _to_map(x, y):
    # The node's conversion: CARLA y is mirrored in the map frame.
    return x, -y


def test_states_map_to_autoware_elements():
    assert traffic_lights.state_to_element(traffic_lights.CARLA_RED) == (
        traffic_lights.RED,
        traffic_lights.SOLID_ON,
    )
    assert traffic_lights.state_to_element(traffic_lights.CARLA_YELLOW)[0] == traffic_lights.AMBER
    assert traffic_lights.state_to_element(traffic_lights.CARLA_GREEN)[0] == traffic_lights.GREEN
    assert traffic_lights.state_to_element(traffic_lights.CARLA_OFF) == (
        traffic_lights.UNKNOWN_COLOR,
        traffic_lights.SOLID_OFF,
    )
    assert traffic_lights.state_to_element(4) == (
        traffic_lights.UNKNOWN_COLOR,
        traffic_lights.UNKNOWN_STATUS,
    )


def test_without_a_map_the_opendrive_id_is_the_group():
    lights = [_Light(1, "100", 0, 0), _Light(2, "not-a-number", 0, 0)]
    assignments, result = traffic_lights.resolve_groups(lights, _to_map)
    assert assignments == {1: [100]}
    assert result is None


def test_the_override_wins():
    lights = [_Light(1, "100", 0, 0), _Light(2, "7", 0, 0)]
    assignments, _ = traffic_lights.resolve_groups(lights, _to_map, override={7: [41, 42]})
    assert assignments == {1: [100], 2: [41, 42]}


def test_lights_are_matched_to_the_lanelet2_map(tmp_path):
    osm = tmp_path / "map.osm"
    osm.write_text(_OSM)
    # Head 200 sits at map (11, 0) and belongs to group 501 only.
    lights = [_Light(5, "300", 11.0, 0.0), _Light(6, "301", 500.0, 0.0, boxes=False)]
    assignments, result = traffic_lights.resolve_groups(lights, _to_map, map_path=str(osm))
    assert assignments == {5: [501]}
    assert result is not None


def test_group_states_follow_the_light_states():
    lights = [_Light(1, "1", 0, 0, traffic_lights.CARLA_GREEN), _Light(2, "2", 0, 0)]
    states = traffic_lights.group_states(lights, {1: [10, 11], 2: [12]})
    assert states[10] == (traffic_lights.GREEN, traffic_lights.SOLID_ON)
    assert states[11] == (traffic_lights.GREEN, traffic_lights.SOLID_ON)
    assert states[12] == (traffic_lights.RED, traffic_lights.SOLID_ON)
