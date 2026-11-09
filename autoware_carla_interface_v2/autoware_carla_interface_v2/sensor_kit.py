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

"""The sensor kit CARLA publishes natively, described in YAML.

CARLA 1.0 publishes sensor data on ROS 2 itself: a sensor spawned with the
``ros_name`` (its frame_id) and ``ros_topic_name`` blueprint attributes and
then ``enable_for_ros()``-ed publishes on Autoware's topic. This module turns a
YAML description of the kit into that list of sensors; the node spawns them.

Format (all poses in the ROS convention, meters and radians)::

    vehicle:
      blueprint: vehicle.lincoln.mkz
      base_link: {x: -1.39706787}        # CARLA vehicle origin -> base_link
    frames:                               # optional intermediate frames
      sensor_kit_base_link:
        parent: base_link
        pose: {x: 0.9, z: 2.0}
    sensors:
      lidar_top:
        blueprint: sensor.lidar.ray_cast
        parent: sensor_kit_base_link      # base_link, a frame, or another sensor
        pose: {yaw: 1.575}
        frame_id: velodyne_top            # -> the ros_name attribute
        topic: /sensing/lidar/top/pointcloud_raw_ex   # -> ros_topic_name
        attributes: {channels: 16}

Every sensor is attached to the vehicle with its pose resolved through the
frame chain, so an IMU reports the vehicle body's own angular velocity.
Camera topics get CARLA's ``/image`` and ``/camera_info`` suffixes; LiDAR
topics are used verbatim.
"""

from dataclasses import dataclass
from dataclasses import field
from typing import Dict
from typing import List
from typing import Mapping
from typing import Optional

import yaml

from .transforms import Pose
from .transforms import chain

BASE_LINK = "base_link"

# Sensor types that need the renderer (see the node's no_rendering_mode).
_RENDERED_PREFIXES = ("sensor.camera.",)


@dataclass(frozen=True)
class SensorSpec:
    """One sensor to spawn."""

    name: str
    blueprint: str
    pose_in_base_link: Pose
    frame_id: str
    topic: Optional[str]
    attributes: Dict[str, str] = field(default_factory=dict)

    @property
    def needs_rendering(self) -> bool:
        return self.blueprint.startswith(_RENDERED_PREFIXES)


@dataclass(frozen=True)
class SensorKit:
    """The vehicle and the sensors on it."""

    vehicle_blueprint: str
    base_link: Pose
    sensors: List[SensorSpec]

    def sensor_pose_in_vehicle(self, sensor: SensorSpec) -> Pose:
        """The sensor's pose relative to the CARLA vehicle origin (ROS convention)."""
        return self.base_link.compose(sensor.pose_in_base_link)


def load_sensor_kit(path: str) -> SensorKit:
    """Load a sensor kit YAML file."""
    with open(path, "r", encoding="utf-8") as stream:
        return parse_sensor_kit(yaml.safe_load(stream) or {})


def parse_sensor_kit(data: Mapping) -> SensorKit:
    """Build a :class:`SensorKit` from the parsed YAML; raises ValueError on a bad description."""
    if not isinstance(data, Mapping):
        raise ValueError("the sensor kit must be a mapping")
    vehicle = data.get("vehicle") or {}
    blueprint = str(vehicle.get("blueprint", "vehicle.lincoln.mkz"))
    base_link = Pose.from_mapping(vehicle.get("base_link") or {})

    parents: Dict[str, str] = {}
    poses: Dict[str, Pose] = {}
    for name, frame in (data.get("frames") or {}).items():
        parents[name], poses[name] = _parent_and_pose(name, frame)

    sensors_data = data.get("sensors") or {}
    if not isinstance(sensors_data, Mapping) or not sensors_data:
        raise ValueError("the sensor kit has no sensors")
    for name, sensor in sensors_data.items():
        if name in parents or name == BASE_LINK:
            raise ValueError(f"sensor '{name}' reuses the name of a frame")
        parents[name], poses[name] = _parent_and_pose(name, sensor)

    sensors = []
    for name, sensor in sensors_data.items():
        if "blueprint" not in sensor:
            raise ValueError(f"sensor '{name}' has no blueprint")
        sensors.append(
            SensorSpec(
                name=name,
                blueprint=str(sensor["blueprint"]),
                pose_in_base_link=_resolve(name, parents, poses),
                frame_id=str(sensor.get("frame_id", name)),
                topic=str(sensor["topic"]) if sensor.get("topic") else None,
                attributes={
                    str(k): _attribute(v) for k, v in (sensor.get("attributes") or {}).items()
                },
            )
        )
    return SensorKit(vehicle_blueprint=blueprint, base_link=base_link, sensors=sensors)


def _parent_and_pose(name, entry):
    if not isinstance(entry, Mapping):
        raise ValueError(f"'{name}' must be a mapping")
    try:
        pose = Pose.from_mapping(entry.get("pose") or {})
    except ValueError as error:
        raise ValueError(f"'{name}': {error}") from None
    return str(entry.get("parent", BASE_LINK)), pose


def _resolve(name, parents, poses):
    """The pose of ``name`` in base_link, following the parent chain."""
    path = []
    current = name
    while current != BASE_LINK:
        if current in path:
            raise ValueError(f"frame cycle through '{current}'")
        if current not in parents:
            raise ValueError(f"'{path[-1] if path else name}' has unknown parent '{current}'")
        path.append(current)
        current = parents[current]
    return chain(poses[frame] for frame in reversed(path))


def _attribute(value):
    """Blueprint attributes are strings; YAML booleans become CARLA's 'true' / 'false'."""
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)
