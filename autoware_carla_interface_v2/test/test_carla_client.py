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

"""Unit tests for the typesafe_carla adaptation layer (no CARLA server needed)."""

import os

import pytest

from autoware_carla_interface_v2 import carla_client
from autoware_carla_interface_v2.carla_client import as_sensor
from autoware_carla_interface_v2.carla_client import as_traffic_light
from autoware_carla_interface_v2.carla_client import as_vehicle
from autoware_carla_interface_v2.carla_client import carla
from autoware_carla_interface_v2.carla_client import to_measurement


class _FakeActor:
    """Stands in for a typesafe_carla Actor: kind predicates plus as_<kind>()."""

    def __init__(self, kind):
        self.kind = kind
        self.is_vehicle = kind == "vehicle"
        self.is_sensor = kind == "sensor"
        self.is_traffic_light = kind == "traffic_light"

    def as_vehicle(self):
        assert self.is_vehicle
        return ("vehicle", self)

    def as_sensor(self):
        assert self.is_sensor
        return ("sensor", self)

    def as_traffic_light(self):
        assert self.is_traffic_light
        return ("traffic_light", self)


def test_the_client_is_typesafe_carla():
    assert carla.__name__ == "typesafe_carla.carla"


def test_carla_1_0_is_selected_by_default():
    assert carla_client.CARLA_REF == "1.0"
    assert os.environ["TYPESAFE_CARLA_CARLA_REF"]


@pytest.mark.skipif(
    os.environ.get("TYPESAFE_CARLA_CARLA_REF", "1.0") != "1.0",
    reason="another CARLA ref was selected explicitly",
)
def test_the_loaded_libcarla_is_carla_1_0():
    # Creating a client does not connect; the version is LibCarla's own.
    client = carla.Client("127.0.0.1", 2000)
    assert client.get_client_version().startswith("1.0")


def test_actor_conversions_pick_the_matching_kind():
    vehicle = _FakeActor("vehicle")
    sensor = _FakeActor("sensor")
    light = _FakeActor("traffic_light")
    assert as_vehicle(vehicle) == ("vehicle", vehicle)
    assert as_sensor(sensor) == ("sensor", sensor)
    assert as_traffic_light(light) == ("traffic_light", light)


def test_actor_conversions_reject_other_kinds_and_none():
    sensor = _FakeActor("sensor")
    assert as_vehicle(sensor) is None
    assert as_traffic_light(sensor) is None
    assert as_sensor(_FakeActor("vehicle")) is None
    assert as_vehicle(None) is None
    assert as_sensor(None) is None
    assert as_traffic_light(None) is None


def test_measurements_that_are_not_sensor_data_pass_through():
    marker = object()
    assert to_measurement(marker) is marker


def test_vehicle_light_state_combinations_are_not_enum_members():
    # The reason apply_light_state hands set_light_state a plain int.
    both = int(carla.VehicleLightState.LeftBlinker) | int(carla.VehicleLightState.RightBlinker)
    with pytest.raises(ValueError):
        carla.VehicleLightState(both)


def test_nested_transform_fields_are_writable():
    # The spawn-point code edits transform.location in place.
    transform = carla.Transform()
    transform.location.x = 1.5
    transform.rotation.yaw = 90.0
    assert transform.location.x == pytest.approx(1.5)
    assert transform.rotation.yaw == pytest.approx(90.0)
