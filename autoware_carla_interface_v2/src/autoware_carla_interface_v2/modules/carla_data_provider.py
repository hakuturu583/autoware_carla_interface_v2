# Copyright 2024 Tier IV, Inc.
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

"""Shared CARLA state of the bridge: client, world, spawned actors and game time.

Derived from the CARLA scenario runner's data provider, reduced to what the
bridge uses. With typesafe_carla every query returns a new wrapper object, and
wrappers hash by identity, so actors are tracked by their id rather than used
as dictionary keys.
"""

import datetime
import threading

from numpy import random

from .carla_client import as_vehicle
from .carla_client import carla


class CarlaDataProvider(object):
    _actor_transform_map = {}
    _carla_actor_pool = {}
    _client = None
    _world = None
    _map = None
    _sync_flag = False
    _spawn_points = None
    _blueprint_library = None
    _traffic_manager_port = 8000
    _random_seed = 2000
    _rng = random.RandomState(_random_seed)
    _lock = threading.Lock()

    @staticmethod
    def register_actor(actor, transform=None):
        """Track a spawned actor's transform (updated on every tick)."""
        with CarlaDataProvider._lock:
            if actor.id in CarlaDataProvider._actor_transform_map:
                raise KeyError(f"Actor '{actor.id}' already registered. Cannot register twice!")
            CarlaDataProvider._actor_transform_map[actor.id] = transform

    @staticmethod
    def on_carla_tick():
        """Refresh the tracked actors' transforms."""
        with CarlaDataProvider._lock:
            for actor_id in list(CarlaDataProvider._actor_transform_map):
                actor = CarlaDataProvider._carla_actor_pool.get(actor_id)
                if actor is not None and actor.is_alive:
                    CarlaDataProvider._actor_transform_map[actor_id] = actor.get_transform()
            if CarlaDataProvider._world is None:
                print("WARNING: CarlaDataProvider couldn't find the world")

    @staticmethod
    def get_transform(actor):
        """Return the transform of ``actor`` as of the last tick, or None."""
        return CarlaDataProvider._actor_transform_map.get(actor.id)

    @staticmethod
    def set_client(client):
        """Set the CARLA client."""
        CarlaDataProvider._client = client

    @staticmethod
    def get_client():
        """Get the CARLA client."""
        return CarlaDataProvider._client

    @staticmethod
    def set_world(world):
        """Set the world and cache its settings, map, blueprints and spawn points."""
        CarlaDataProvider._world = world
        CarlaDataProvider._sync_flag = world.get_settings().synchronous_mode
        CarlaDataProvider._map = None
        CarlaDataProvider._load_map()
        CarlaDataProvider._blueprint_library = world.get_blueprint_library()
        CarlaDataProvider.generate_spawn_points()

    @staticmethod
    def get_world():
        """Return world."""
        return CarlaDataProvider._world

    @staticmethod
    def get_map(world=None):
        """Get the current map."""
        if CarlaDataProvider._map is None:
            world = world or CarlaDataProvider._world
            if world is None:
                raise ValueError("class member 'world' not initialized yet")
            CarlaDataProvider._map = world.get_map()
        return CarlaDataProvider._map

    @staticmethod
    def _load_map():
        """Load the OpenDRIVE map, tolerating levels without parseable metadata."""
        try:
            CarlaDataProvider._map = CarlaDataProvider._world.get_map()
        except RuntimeError as error:
            # Some levels expose no parseable OpenDRIVE; the bridge runs without a map.
            print("WARNING: CarlaDataProvider couldn't load the map: {}".format(error))
            CarlaDataProvider._map = None

    @staticmethod
    def generate_spawn_points():
        """Shuffle the current map's recommended spawn points."""
        spawn_points = []
        if CarlaDataProvider._map is not None:
            try:
                spawn_points = list(CarlaDataProvider._map.get_spawn_points())
            except RuntimeError as error:
                print("WARNING: CarlaDataProvider couldn't generate spawn points: {}".format(error))
        order = CarlaDataProvider._rng.permutation(len(spawn_points))
        CarlaDataProvider._spawn_points = [spawn_points[i] for i in order]

    @staticmethod
    def is_sync_mode():
        """Return true if synchronous mode is used."""
        return CarlaDataProvider._sync_flag

    @staticmethod
    def _choice(items):
        """Pick one element; numpy's choice() would try to make an array of CARLA objects."""
        return items[CarlaDataProvider._rng.randint(len(items))]

    @staticmethod
    def create_blueprint(model, rolename="scenario", color=None):
        """Return the blueprint of ``model`` with its color and role name set."""
        blueprints = list(CarlaDataProvider._blueprint_library.filter(model))
        if not blueprints:
            print("WARNING: Actor model {} not available. Using a random vehicle".format(model))
            blueprints = list(CarlaDataProvider._blueprint_library.filter("vehicle.*"))
        if not blueprints:
            raise RuntimeError("The CARLA blueprint library has no vehicle")
        blueprint = CarlaDataProvider._choice(blueprints)

        if blueprint.has_attribute("color"):
            if color:
                blueprint.set_attribute("color", color)
            elif rolename != "hero":
                recommended = list(blueprint.get_attribute("color").recommended_values)
                if recommended:
                    blueprint.set_attribute("color", CarlaDataProvider._choice(recommended))

        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", rolename)
        return blueprint

    @staticmethod
    def _spawn_actor_at(blueprint, model, spawn_point):
        """Spawn an actor at a fixed transform, lifting non-prop models off the ground."""
        z_offset = 0.2 if "prop" not in model else 0
        # Copy: do not modify the caller's spawn point.
        lifted = carla.Transform(
            carla.Location(
                x=spawn_point.location.x,
                y=spawn_point.location.y,
                z=spawn_point.location.z + z_offset,
            ),
            spawn_point.rotation,
        )
        return CarlaDataProvider._world.try_spawn_actor(blueprint, lifted)

    @staticmethod
    def _spawn_actor(blueprint, model, spawn_point, random_location):
        """Spawn an actor; return (actor or None, the transform it was spawned against)."""
        if random_location and not CarlaDataProvider._spawn_points:
            print(
                "WARNING: No map spawn points available for random placement; "
                "falling back to the map origin."
            )
            spawn_point = carla.Transform()
            return CarlaDataProvider._spawn_actor_at(blueprint, model, spawn_point), spawn_point

        if random_location:
            for candidate in CarlaDataProvider._spawn_points:
                actor = CarlaDataProvider._world.try_spawn_actor(blueprint, candidate)
                if actor is not None:
                    return actor, candidate
            return None, CarlaDataProvider._spawn_points[0]

        return CarlaDataProvider._spawn_actor_at(blueprint, model, spawn_point), spawn_point

    @staticmethod
    def request_new_actor(
        model,
        spawn_point,
        rolename="scenario",
        autopilot=False,
        random_location=False,
        color=None,
        tick=True,
    ):
        """Spawn an actor and return it (a ``carla.Vehicle`` for a vehicle), or None."""
        blueprint = CarlaDataProvider.create_blueprint(model, rolename, color)
        actor, spawn_point = CarlaDataProvider._spawn_actor(
            blueprint, model, spawn_point, random_location
        )
        if actor is None:
            print(
                "WARNING: Cannot spawn actor {} at position {}".format(model, spawn_point.location)
            )
            return None

        vehicle = as_vehicle(actor)
        if vehicle is not None:
            actor = vehicle
        if autopilot:
            if vehicle is not None:
                vehicle.set_autopilot(autopilot, CarlaDataProvider._traffic_manager_port)
            else:
                print("WARNING: Tried to set the autopilot of a non vehicle actor")

        # Wait for the actor to be spawned properly before we do anything
        if tick:
            if CarlaDataProvider.is_sync_mode():
                CarlaDataProvider._world.tick()
            else:
                CarlaDataProvider._world.wait_for_tick()

        CarlaDataProvider._carla_actor_pool[actor.id] = actor
        CarlaDataProvider.register_actor(actor, spawn_point)
        return actor

    @staticmethod
    def get_actor_by_id(actor_id):
        """Return a spawned actor by id, or None."""
        return CarlaDataProvider._carla_actor_pool.get(actor_id)

    @staticmethod
    def get_traffic_manager_port():
        """Get the port of the traffic manager."""
        return CarlaDataProvider._traffic_manager_port

    @staticmethod
    def set_traffic_manager_port(tm_port):
        """Set the port to use for the traffic manager."""
        CarlaDataProvider._traffic_manager_port = tm_port

    @staticmethod
    def cleanup():
        """Destroy the spawned actors and reset all state."""
        batch = [
            carla.command.DestroyActor(actor)
            for actor in CarlaDataProvider._carla_actor_pool.values()
            if actor is not None and actor.is_alive
        ]
        if CarlaDataProvider._client and batch:
            try:
                CarlaDataProvider._client.apply_batch_sync(batch)
            except RuntimeError as e:
                if "time-out" not in str(e) and "timeout" not in str(e).lower():
                    raise

        CarlaDataProvider._actor_transform_map.clear()
        CarlaDataProvider._carla_actor_pool = {}
        CarlaDataProvider._map = None
        CarlaDataProvider._world = None
        CarlaDataProvider._sync_flag = False
        CarlaDataProvider._client = None
        CarlaDataProvider._spawn_points = None
        CarlaDataProvider._blueprint_library = None
        CarlaDataProvider._rng = random.RandomState(CarlaDataProvider._random_seed)


class GameTime(object):
    """Provides access to the CARLA game time."""

    _current_game_time = 0.0  # Elapsed game time after starting this Timer
    _carla_time = 0.0
    _last_frame = 0
    _platform_timestamp = 0
    _init = False

    @staticmethod
    def on_carla_tick(timestamp):
        """Advance the game time to ``timestamp`` if its frame is newer than the last one."""
        if GameTime._last_frame < timestamp.frame:
            frames = timestamp.frame - GameTime._last_frame if GameTime._init else 1
            GameTime._current_game_time += timestamp.delta_seconds * frames
            GameTime._last_frame = timestamp.frame
            GameTime._platform_timestamp = datetime.datetime.now()
            GameTime._init = True
            GameTime._carla_time = timestamp.elapsed_seconds

    @staticmethod
    def restart():
        """Reset game timer to 0."""
        GameTime._current_game_time = 0.0
        GameTime._carla_time = 0.0
        GameTime._last_frame = 0
        GameTime._init = False

    @staticmethod
    def get_time():
        """Return elapsed game time."""
        return GameTime._current_game_time

    @staticmethod
    def get_carla_time():
        """Return the simulator's elapsed time of the last frame."""
        return GameTime._carla_time

    @staticmethod
    def get_wall_clock_time():
        """Return the wall-clock time the last frame was received."""
        return GameTime._platform_timestamp

    @staticmethod
    def get_frame():
        """Return the last frame number."""
        return GameTime._last_frame
