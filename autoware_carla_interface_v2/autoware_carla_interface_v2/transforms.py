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

"""Pose conversions between ROS (Autoware) and CARLA, independent of the CARLA client.

ROS is right-handed (x forward, y left, z up; angles in radians), CARLA is
left-handed (x forward, y right, z up; angles in degrees). The conversion
negates y, keeps roll and negates pitch and yaw, as the CARLA ros-bridge and
CARLA's own ``autoware_demo.py`` do.

Poses here are plain :class:`Pose` values in the ROS convention; ``to_carla``
in the node turns them into ``carla.Transform`` objects.
"""

from dataclasses import dataclass
import math
from typing import Iterable
from typing import Mapping
from typing import Tuple


@dataclass(frozen=True)
class Pose:
    """A rigid transform in the ROS convention (meters, radians)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    @staticmethod
    def from_mapping(values: Mapping) -> "Pose":
        """Build a pose from a mapping with any of x, y, z, roll, pitch, yaw."""
        unknown = set(values) - {"x", "y", "z", "roll", "pitch", "yaw"}
        if unknown:
            raise ValueError(f"unknown pose keys: {sorted(unknown)}")
        return Pose(**{key: float(value) for key, value in values.items()})

    def compose(self, other: "Pose") -> "Pose":
        """Return ``self * other``: ``other`` expressed in the parent frame of ``self``."""
        r1 = _rpy_to_matrix(self.roll, self.pitch, self.yaw)
        r2 = _rpy_to_matrix(other.roll, other.pitch, other.yaw)
        rotation = _mat_mul(r1, r2)
        offset = _mat_vec(r1, (other.x, other.y, other.z))
        roll, pitch, yaw = _matrix_to_rpy(rotation)
        return Pose(self.x + offset[0], self.y + offset[1], self.z + offset[2], roll, pitch, yaw)

    def inverse(self) -> "Pose":
        """Return the transform that undoes this one."""
        r = _rpy_to_matrix(self.roll, self.pitch, self.yaw)
        rt = tuple(tuple(r[j][i] for j in range(3)) for i in range(3))
        offset = _mat_vec(rt, (self.x, self.y, self.z))
        roll, pitch, yaw = _matrix_to_rpy(rt)
        return Pose(-offset[0], -offset[1], -offset[2], roll, pitch, yaw)

    def to_carla_values(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """Return ((x, y, z), (roll, pitch, yaw)) in CARLA's convention (meters, degrees)."""
        return (
            (self.x, -self.y, self.z),
            (math.degrees(self.roll), -math.degrees(self.pitch), -math.degrees(self.yaw)),
        )

    @staticmethod
    def from_carla_values(location, rotation) -> "Pose":
        """Inverse of :meth:`to_carla_values` (location (x, y, z), rotation (roll, pitch, yaw))."""
        x, y, z = location
        roll, pitch, yaw = rotation
        return Pose(x, -y, z, math.radians(roll), -math.radians(pitch), -math.radians(yaw))


def chain(poses: Iterable[Pose]) -> Pose:
    """Compose poses left to right (parent first)."""
    result = Pose()
    for pose in poses:
        result = result.compose(pose)
    return result



def yaw_to_quaternion(yaw: float) -> Tuple[float, float]:
    """(z, w) of a rotation about z."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """ROS quaternion to roll, pitch, yaw [rad]."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def _rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _matrix_to_rpy(m):
    pitch = math.asin(max(-1.0, min(1.0, -m[2][0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    else:  # gimbal lock: fold roll into yaw
        roll = 0.0
        yaw = math.atan2(-m[0][1], m[1][1])
    return roll, pitch, yaw


def _mat_mul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3)
    )


def _mat_vec(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))
