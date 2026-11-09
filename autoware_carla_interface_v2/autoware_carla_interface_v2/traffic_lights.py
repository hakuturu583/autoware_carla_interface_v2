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

"""CARLA traffic-light states as Autoware traffic-light groups.

CARLA 1.0 publishes no traffic-light state on ROS 2, and Autoware's camera
classifier does not recognise CARLA's rendered lights. The node therefore
reads each light's state from the simulator and publishes it on Autoware's
external traffic-signal input, keyed by the lanelet2 regulatory element
(``traffic_light_group_id``) the light belongs to.

The functions here take the CARLA objects as duck-typed arguments (``id``,
``get_state()``, ``get_opendrive_id()``, ``get_light_boxes()``,
``get_location()``) so they can be tested without a simulator.
"""

from typing import Callable
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Tuple

from .traffic_light_matcher import load_map_traffic_lights
from .traffic_light_matcher import match_traffic_lights

# carla.TrafficLightState values (Red, Yellow, Green, Off, Unknown).
CARLA_RED = 0
CARLA_YELLOW = 1
CARLA_GREEN = 2
CARLA_OFF = 3

# autoware_perception_msgs/TrafficLightElement constants (color and status
# share UNKNOWN = 0).
UNKNOWN_COLOR = 0
RED = 1
AMBER = 2
GREEN = 3
UNKNOWN_STATUS = 0
SOLID_OFF = 1
SOLID_ON = 2
CIRCLE = 1


def state_to_element(state) -> Tuple[int, int]:
    """Map a carla.TrafficLightState to (color, status) of a TrafficLightElement."""
    state = int(state)
    if state == CARLA_RED:
        return RED, SOLID_ON
    if state == CARLA_YELLOW:
        return AMBER, SOLID_ON
    if state == CARLA_GREEN:
        return GREEN, SOLID_ON
    if state == CARLA_OFF:
        return UNKNOWN_COLOR, SOLID_OFF
    return UNKNOWN_COLOR, UNKNOWN_STATUS


def opendrive_id(light) -> Optional[int]:
    """The light's OpenDRIVE signal id as an int, or None when it is not numeric."""
    try:
        return int(light.get_opendrive_id())
    except (ValueError, RuntimeError, TypeError):
        return None


def head_position(light, to_map: Callable) -> Tuple[float, float]:
    """The light head's (x, y) in the map frame.

    The mean of the light-box centres (the physical heads) rather than the
    actor origin at the pole base, which is offset from the heads the lanelet2
    map records. ``to_map`` turns a carla.Location into map-frame (x, y).
    """
    try:
        boxes = list(light.get_light_boxes())
    except RuntimeError:
        boxes = []
    if boxes:
        n = float(len(boxes))
        x = sum(box.location.x for box in boxes) / n
        y = sum(box.location.y for box in boxes) / n
        return to_map(x, y)
    location = light.get_location()
    return to_map(location.x, location.y)


def resolve_groups(
    lights: Iterable,
    to_map: Callable,
    map_path: str = "",
    override: Optional[Dict[int, List[int]]] = None,
    match_distance: float = 5.0,
    match_ratio: float = 0.6,
):
    """Assign each light (by actor id) its Autoware traffic_light_group_id(s).

    1. An ``override`` entry (OpenDRIVE id -> group ids) wins.
    2. With a lanelet2 ``map_path``, lights are matched to the map's light heads
       by position; ambiguous or distant lights are left out (see the
       returned match result) rather than guessed.
    3. Without a map, the OpenDRIVE signal id is used as the group id.

    Returns (assignments, match_result_or_None).
    """
    lights = list(lights)
    override = override or {}
    assignments: Dict[int, List[int]] = {}
    remaining = []
    for light in lights:
        signal = opendrive_id(light)
        if signal is not None and signal in override:
            assignments[light.id] = list(override[signal])
        else:
            remaining.append(light)

    if not map_path:
        for light in remaining:
            signal = opendrive_id(light)
            if signal is not None:
                assignments[light.id] = [signal]
        return assignments, None

    heads = [(light.id, opendrive_id(light), head_position(light, to_map)) for light in remaining]
    result = match_traffic_lights(
        heads,
        load_map_traffic_lights(map_path),
        distance_threshold=match_distance,
        ambiguity_ratio=match_ratio,
    )
    assignments.update(result.assignments)
    return assignments, result


def group_states(lights: Iterable, assignments: Dict[int, List[int]]) -> Dict[int, Tuple[int, int]]:
    """Group id -> (color, status) for the current light states."""
    states = {}
    for light in lights:
        for group_id in assignments.get(light.id, ()):
            states[group_id] = state_to_element(light.get_state())
    return states
