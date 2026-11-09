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

"""The CARLA client this package talks to the simulator with.

Every module imports ``carla`` from here instead of importing the official
``carla`` package. The client is typesafe_carla
(https://github.com/hakuturu583/typesafe_carla), used from CPython through
``typesafe_carla.carla``, which has the CARLA Python API's names. Its wheel
ships LibCarla for several CARLA refs; ``TYPESAFE_CARLA_CARLA_REF`` selects
one, and this module selects ``1.0`` (the CARLA 1.0 release, UE 5.8) unless the
environment already chose another.

typesafe_carla differs from the Boost.Python ``carla`` module in a few ways
the rest of the package has to respect, collected here as helpers:

* Actors come back as plain ``carla.Actor`` objects. ``isinstance(actor,
  carla.Vehicle)`` is False for a vehicle, and the Vehicle / Sensor /
  TrafficLight methods are only on the object ``as_vehicle()``,
  ``as_sensor()`` or ``as_traffic_light()`` returns. ``as_vehicle`` and its
  siblings below do the conversion and accept an already converted object.
* A sensor callback receives a generic ``carla.SensorData``;
  ``to_measurement`` turns it into the Image, LidarMeasurement,
  GnssMeasurement or IMUMeasurement the bridge works with.
* ``carla.VehicleLightState`` is an ``IntEnum``: a combination of flags is not
  a member, so ``carla.VehicleLightState(flags)`` raises. Pass the int.
* Errors from the simulator are ``RuntimeError``, as with the official module.
"""

import os

# The CARLA release this package is built for. typesafe_carla checks that
# the server's version matches the LibCarla it loads.
CARLA_REF = "1.0"

os.environ.setdefault("TYPESAFE_CARLA_CARLA_REF", CARLA_REF)

import typesafe_carla.carla as carla  # noqa: E402

# carla.SensorData.type values (typesafe_carla's SensorDataType).
SENSOR_DATA_IMAGE = 1
SENSOR_DATA_LIDAR = 2
SENSOR_DATA_GNSS = 3
SENSOR_DATA_IMU = 4

_MEASUREMENT_CONVERTERS = {
    SENSOR_DATA_IMAGE: "as_image",
    SENSOR_DATA_LIDAR: "as_lidar",
    SENSOR_DATA_GNSS: "as_gnss",
    SENSOR_DATA_IMU: "as_imu",
}


def _convert(actor, kind, predicate):
    if actor is None:
        return None
    if isinstance(actor, getattr(carla, kind)):
        return actor
    if not getattr(actor, predicate, False):
        return None
    return getattr(actor, "as_" + _SNAKE[kind])()


_SNAKE = {"Vehicle": "vehicle", "Sensor": "sensor", "TrafficLight": "traffic_light"}


def as_vehicle(actor):
    """Return ``actor`` as a ``carla.Vehicle``, or None if it is not a vehicle."""
    return _convert(actor, "Vehicle", "is_vehicle")


def as_sensor(actor):
    """Return ``actor`` as a ``carla.Sensor``, or None if it is not a sensor."""
    return _convert(actor, "Sensor", "is_sensor")


def as_traffic_light(actor):
    """Return ``actor`` as a ``carla.TrafficLight``, or None if it is not one."""
    return _convert(actor, "TrafficLight", "is_traffic_light")


def to_measurement(data):
    """Return a sensor callback's data as its measurement type.

    A ``carla.SensorData`` of a camera, LiDAR, GNSS or IMU is converted to
    ``carla.Image``, ``carla.LidarMeasurement``, ``carla.GnssMeasurement`` or
    ``carla.IMUMeasurement``. Anything else (another sensor's data, or an
    object that is already a measurement) is returned unchanged.
    """
    if not isinstance(data, carla.SensorData):
        return data
    converter = _MEASUREMENT_CONVERTERS.get(int(data.type))
    if converter is None:
        return data
    return getattr(data, converter)()


def server_version(client):
    """The server's version string, e.g. ``"1.0"``."""
    return client.get_server_version()


def libcarla_version(client):
    """The version of the LibCarla the client was built from, e.g. ``"1.0"``."""
    return client.get_client_version()


__all__ = [
    "CARLA_REF",
    "as_sensor",
    "as_traffic_light",
    "as_vehicle",
    "carla",
    "libcarla_version",
    "server_version",
    "to_measurement",
]
