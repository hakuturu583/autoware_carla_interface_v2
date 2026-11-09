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

"""Sensor kit description loading."""

import math
import os

import pytest

from autoware_carla_interface_v2.sensor_kit import load_sensor_kit
from autoware_carla_interface_v2.sensor_kit import parse_sensor_kit
from autoware_carla_interface_v2.transforms import Pose

DEFAULT_KIT = os.path.join(os.path.dirname(__file__), "..", "config", "sensor_kit.yaml")


def test_the_default_kit_loads():
    kit = load_sensor_kit(DEFAULT_KIT)
    assert kit.vehicle_blueprint == "vehicle.lincoln.mkz"
    names = {sensor.name: sensor for sensor in kit.sensors}
    assert set(names) == {"lidar_top", "traffic_light_camera", "imu", "gnss"}
    assert names["lidar_top"].topic == "/sensing/lidar/top/pointcloud_raw_ex"
    assert names["lidar_top"].frame_id == "velodyne_top"
    assert names["lidar_top"].attributes["channels"] == "16"
    assert names["traffic_light_camera"].needs_rendering
    assert not names["imu"].needs_rendering


def test_poses_follow_the_frame_chain():
    kit = parse_sensor_kit(
        {
            "vehicle": {"base_link": {"x": -1.0}},
            "frames": {"kit": {"pose": {"x": 1.0, "z": 2.0, "yaw": math.pi / 2}}},
            "sensors": {
                "lidar": {"blueprint": "sensor.lidar.ray_cast", "parent": "kit", "pose": {"x": 1.0}}
            },
        }
    )
    lidar = kit.sensors[0]
    assert lidar.pose_in_base_link.x == pytest.approx(1.0)
    assert lidar.pose_in_base_link.y == pytest.approx(1.0)
    assert lidar.pose_in_base_link.z == pytest.approx(2.0)
    in_vehicle = kit.sensor_pose_in_vehicle(lidar)
    assert (in_vehicle.x, in_vehicle.y) == pytest.approx((0.0, 1.0))
    assert lidar.frame_id == "lidar"  # defaults to the sensor name
    assert lidar.topic is None


def test_boolean_attributes_become_carla_strings():
    kit = parse_sensor_kit(
        {
            "sensors": {
                "cam": {
                    "blueprint": "sensor.camera.rgb",
                    "attributes": {"enable_postprocess_effects": True},
                }
            }
        }
    )
    assert kit.sensors[0].attributes == {"enable_postprocess_effects": "true"}
    assert kit.base_link == Pose()


@pytest.mark.parametrize(
    "data, message",
    [
        ({}, "no sensors"),
        ({"sensors": {"a": {"parent": "base_link"}}}, "no blueprint"),
        ({"sensors": {"a": {"blueprint": "x", "parent": "nowhere"}}}, "unknown parent"),
        (
            {
                "frames": {"f": {"parent": "g"}, "g": {"parent": "f"}},
                "sensors": {"a": {"blueprint": "x", "parent": "f"}},
            },
            "cycle",
        ),
        ({"sensors": {"a": {"blueprint": "x", "pose": {"heading": 1}}}}, "unknown pose keys"),
        ({"frames": {"a": {}}, "sensors": {"a": {"blueprint": "x"}}}, "reuses the name"),
    ],
)
def test_bad_descriptions_are_rejected(data, message):
    with pytest.raises(ValueError, match=message):
        parse_sensor_kit(data)
