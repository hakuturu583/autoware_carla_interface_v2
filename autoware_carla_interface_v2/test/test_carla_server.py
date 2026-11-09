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

"""Integration test against a running CARLA 1.0 server.

Skipped unless CARLA_HOST is set. Start the server, e.g. headless:

    ./CarlaUnreal.sh -nullrhi -nosound      # CARLA_1.0.tar.gz, no GPU needed
    CARLA_HOST=localhost CARLA_PORT=2000 python3 -m pytest test/test_carla_server.py -s

It drives the same code paths the bridge uses: the version check, the
data provider's ego spawn (Actor -> Vehicle), a sensor spawned through
SensorWrapper (Actor -> Sensor, SensorData -> IMUMeasurement), synchronous
ticking, and cleanup.
"""

import os

import pytest

HOST = os.environ.get("CARLA_HOST")
PORT = int(os.environ.get("CARLA_PORT", "2000"))

pytestmark = pytest.mark.skipif(not HOST, reason="CARLA_HOST is not set (no CARLA server)")


class _Agent:
    """The part of carla_ros2_interface SensorWrapper reads."""

    def __init__(self):
        from autoware_carla_interface_v2.modules.carla_wrapper import SensorInterface

        self.sensor_interface = SensorInterface()
        self.sensors = {
            "sensors": [
                {
                    "type": "sensor.other.imu",
                    "id": "imu",
                    "spawn_point": {"x": 0, "y": 0, "z": 1, "roll": 0, "pitch": 0, "yaw": 0},
                }
            ]
        }


@pytest.fixture
def client():
    from autoware_carla_interface_v2.modules.carla_client import carla

    client = carla.Client(HOST, PORT)
    client.set_timeout(30.0)
    return client


def test_server_is_carla_1_0(client):
    assert client.get_server_version().startswith("1.0")
    assert client.get_client_version().startswith("1.0")


def test_spawn_ego_with_sensor_and_tick(client):
    from autoware_carla_interface_v2.modules.carla_client import carla
    from autoware_carla_interface_v2.modules.carla_data_provider import CarlaDataProvider
    from autoware_carla_interface_v2.modules.carla_wrapper import SensorWrapper

    world = client.get_world()
    original = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    wrapper = None
    try:
        CarlaDataProvider.set_client(client)
        CarlaDataProvider.set_world(world)
        ego = CarlaDataProvider.request_new_actor(
            "vehicle.*", carla.Transform(), "ego_vehicle", random_location=True
        )
        assert ego is not None
        assert isinstance(ego, carla.Vehicle)
        ego.apply_control(carla.VehicleControl(throttle=0.3))
        assert ego.get_physics_control().wheels

        agent = _Agent()
        wrapper = SensorWrapper(agent)
        wrapper.setup_sensors(ego)
        measurements = []
        for _ in range(20):
            frame = world.tick()
            measurements += agent.sensor_interface.get_data(frame)
        assert measurements, "the IMU delivered nothing in 20 ticks"
        tag, _, data = measurements[0]
        assert tag == "imu"
        assert isinstance(data, carla.IMUMeasurement)
    finally:
        if wrapper is not None:
            wrapper.cleanup()
        CarlaDataProvider.cleanup()
        world.apply_settings(original)
