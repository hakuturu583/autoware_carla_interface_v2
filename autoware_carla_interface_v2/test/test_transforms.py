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

"""ROS <-> CARLA pose conversions."""

import math

import pytest

from autoware_carla_interface_v2.transforms import Pose
from autoware_carla_interface_v2.transforms import chain
from autoware_carla_interface_v2.transforms import quaternion_to_rpy


def _close(a, b, tol=1e-9):
    return all(
        abs(getattr(a, k) - getattr(b, k)) < tol for k in ("x", "y", "z", "roll", "pitch", "yaw")
    )


def test_ros_to_carla_mirrors_y_pitch_and_yaw():
    location, rotation = Pose(1.0, 2.0, 3.0, 0.1, 0.2, math.pi / 2).to_carla_values()
    assert location == (1.0, -2.0, 3.0)
    assert rotation == pytest.approx((math.degrees(0.1), -math.degrees(0.2), -90.0))


def test_carla_round_trip():
    pose = Pose(1.0, -2.0, 0.5, 0.01, -0.02, 1.2)
    assert _close(Pose.from_carla_values(*pose.to_carla_values()), pose)


def test_compose_rotates_the_child_offset():
    parent = Pose(1.0, 0.0, 0.0, yaw=math.pi / 2)
    child = Pose(2.0, 0.0, 0.0)
    assert _close(parent.compose(child), Pose(1.0, 2.0, 0.0, yaw=math.pi / 2))


def test_inverse_undoes_the_pose():
    pose = Pose(0.9, 0.1, 2.0, -0.001, 0.015, -0.0364)
    assert _close(pose.compose(pose.inverse()), Pose())
    assert _close(pose.inverse().compose(pose), Pose())


def test_chain_is_left_to_right():
    a, b, c = Pose(x=1.0, yaw=math.pi), Pose(x=1.0), Pose(y=1.0)
    assert _close(chain([a, b, c]), a.compose(b).compose(c))
    assert _close(chain([]), Pose())


def test_from_mapping_rejects_unknown_keys():
    assert Pose.from_mapping({"x": 1, "yaw": 0.5}) == Pose(x=1.0, yaw=0.5)
    with pytest.raises(ValueError):
        Pose.from_mapping({"heading": 1.0})


def test_quaternion_to_rpy():
    half = math.pi / 4
    assert quaternion_to_rpy(0.0, 0.0, math.sin(half), math.cos(half)) == pytest.approx(
        (0.0, 0.0, math.pi / 2)
    )
