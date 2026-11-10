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

"""The node against a running CARLA 1.0 server started with --ros2.

Skipped unless CARLA_HOST is set:

    ./CarlaUnreal.sh --ros2 -nullrhi -nosound     # no GPU needed
    CARLA_HOST=localhost CARLA_PORT=2000 python3 -m pytest test/test_carla_server.py -s

It runs the node's own start-up (version check, settings, ego with
ros2_autoware_control, vehicle status sensor, the non-camera part of the
default sensor kit), lets it tick, and checks that shutdown removes
everything it spawned and restores the settings.
"""

import os
import time

import pytest

HOST = os.environ.get("CARLA_HOST")
PORT = int(os.environ.get("CARLA_PORT", "2000"))

pytestmark = pytest.mark.skipif(not HOST, reason="CARLA_HOST is not set (no CARLA server)")


def _alive_ids(world):
    """Ids of the actors the world holds after the next frame.

    typesafe_carla's World.get_actor(id) and get_actors(ids) still return an
    actor after it was destroyed, so liveness is read off get_actors().
    """
    if world.get_settings().synchronous_mode:
        world.tick()
    else:
        world.wait_for_tick()
    return {actor.id for actor in world.get_actors()}


@pytest.fixture
def node():
    import rclpy
    from rclpy.parameter import Parameter

    from autoware_carla_interface_v2.node import CarlaAutowareNode

    rclpy.init()
    node = CarlaAutowareNode(
        parameter_overrides=[
            Parameter("host", value=HOST),
            Parameter("port", value=PORT),
            Parameter("timeout", value=60.0),
            Parameter("no_rendering_mode", value=True),  # works on a -nullrhi server too
            Parameter("traffic_light.publish", value=True),
        ]
    )
    yield node
    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()


def test_server_is_carla_1_0():
    from autoware_carla_interface_v2.carla_client import carla

    client = carla.Client(HOST, PORT)
    client.set_timeout(60.0)
    assert client.get_server_version().startswith("1.0")
    assert client.get_client_version().startswith("1.0")


def test_node_spawns_ticks_and_cleans_up(node):
    import rclpy

    node.start()
    world = node._world
    ego_id = node._ego.id
    spawned_ids = [actor.id for actor in node._spawned]
    assert node._ego.attributes.get("role_name") == "ego"
    types = {actor.type_id for actor in world.get_actors(spawned_ids)}
    assert "sensor.other.vehicle_status" in types
    assert "sensor.lidar.ray_cast" in types
    assert "sensor.other.imu" in types

    start = world.get_snapshot().frame
    deadline = time.monotonic() + 10.0
    while world.get_snapshot().frame < start + 20 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert world.get_snapshot().frame >= start + 20, "the node did not tick the world"
    assert node._traffic_light_groups is not None

    settings_before = node._original_settings.synchronous_mode
    node.shutdown()
    alive = _alive_ids(world)
    assert ego_id not in alive
    assert alive.isdisjoint(spawned_ids)
    assert world.get_settings().synchronous_mode == settings_before


def test_scenario_mode_attaches_to_an_existing_ego():
    """As beside autoware_carla_scenario: the ego exists (role 'Ego'), someone else ticks."""
    import rclpy
    from rclpy.parameter import Parameter

    from autoware_carla_interface_v2.carla_client import carla
    from autoware_carla_interface_v2.node import CarlaAutowareNode

    client = carla.Client(HOST, PORT)
    client.set_timeout(60.0)
    world = client.get_world()
    blueprint = world.get_blueprint_library().find("vehicle.lincoln.mkz")
    blueprint.set_attribute("role_name", "Ego")
    points = list(world.get_map().get_spawn_points())
    ego = next(
        actor
        for actor in (world.try_spawn_actor(blueprint, p) for p in points)
        if actor is not None
    )
    rclpy.init()
    node = CarlaAutowareNode(
        parameter_overrides=[
            Parameter("host", value=HOST),
            Parameter("port", value=PORT),
            Parameter("timeout", value=60.0),
            Parameter("scenario_mode", value=True),
            Parameter("no_rendering_mode", value=True),
        ]
    )
    try:
        node.start()
        assert node._ego.id == ego.id
        assert node._tick_thread is None  # the scenario owns the clock
        assert node._vehicle_interface is not None  # 'Ego' is not served natively
        sensor_ids = [actor.id for actor in node._spawned]
        assert all(a.parent is not None for a in world.get_actors(sensor_ids))
        for _ in range(5):
            if world.get_settings().synchronous_mode:
                world.tick()  # stands in for the scenario's tick
            else:
                world.wait_for_tick()
            rclpy.spin_once(node, timeout_sec=0.1)
        node.shutdown()
        alive = _alive_ids(world)
        assert ego.id in alive  # the scenario's ego is left alone
        assert alive.isdisjoint(sensor_ids)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ego.destroy()


def test_scenario_mode_uses_the_native_interface_for_a_native_ego():
    """As with ego.native_autoware_interface: 'ego' spawned with ros2_autoware_control."""
    import rclpy
    from rclpy.parameter import Parameter

    from autoware_carla_interface_v2.carla_client import carla
    from autoware_carla_interface_v2.node import CarlaAutowareNode

    client = carla.Client(HOST, PORT)
    client.set_timeout(60.0)
    world = client.get_world()
    blueprint = world.get_blueprint_library().find("vehicle.lincoln.mkz")
    if not blueprint.has_attribute("ros2_autoware_control"):
        pytest.skip("the server has no native Autoware interface (start it with --ros2)")
    blueprint.set_attribute("role_name", "ego")
    blueprint.set_attribute("ros2_autoware_control", "true")
    points = list(world.get_map().get_spawn_points())
    ego = next(
        actor
        for actor in (world.try_spawn_actor(blueprint, p) for p in points)
        if actor is not None
    )
    rclpy.init()
    node = CarlaAutowareNode(
        parameter_overrides=[
            Parameter("host", value=HOST),
            Parameter("port", value=PORT),
            Parameter("timeout", value=60.0),
            Parameter("scenario_mode", value=True),
            Parameter("no_rendering_mode", value=True),
        ]
    )
    try:
        node.start()
        assert node._ego.id == ego.id
        assert node._vehicle_interface is None  # CARLA serves control and status
        types = {a.type_id for a in world.get_actors([a.id for a in node._spawned])}
        assert "sensor.other.vehicle_status" in types
        node.shutdown()
        assert ego.id in _alive_ids(world)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ego.destroy()
