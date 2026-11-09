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

"""Leave out the test files whose runtime dependencies are not installed.

The bridge's runtime dependencies (the typesafe_carla client, the message
packages the Python node imports) are exec_depends, so a build-only
environment such as the Autoware Index CI does not install them. The files
that import them are not collected there instead of failing.
"""

import importlib.util
from pathlib import Path

import pytest


def _missing(*modules):
    return any(importlib.util.find_spec(module) is None for module in modules)


_CLIENT = ("typesafe_carla",)
_NODE = _CLIENT + (
    "autoware_perception_msgs",
    "autoware_vehicle_msgs",
    "cv2",
    "cv_bridge",
    "tier4_vehicle_msgs",
    "transforms3d",
)

_IGNORED = set()
if _missing(*_NODE):
    _IGNORED |= {"test_light_state.py", "test_no_rendering_mode.py"}
if _missing(*_CLIENT, "rclpy"):
    _IGNORED |= {"test_carla_client.py", "test_carla_server.py", "test_sensor_interface.py"}


@pytest.hookimpl(tryfirst=True)
def pytest_ignore_collect(collection_path: Path, config):
    # A hook rather than collect_ignore: pytest_ignore_collect returns the first
    # answer, and a plugin installed with ROS can answer before pytest reads
    # collect_ignore.
    if collection_path.parent == Path(__file__).parent and collection_path.name in _IGNORED:
        return True
    return None
