# autoware_carla_interface_v2

Runs Autoware on [CARLA 1.0](https://github.com/carla-simulator/carla/releases/tag/1.0)
(Unreal Engine 5.8) through CARLA's native ROS 2 interface.

CARLA 1.0 drives Autoware without a bridge process: started with `--ros2`, the server publishes
the sensor data, `/vehicle/status/*` and `/clock` itself, and applies
`/control/command/control_cmd` to a vehicle spawned with the `ros2_autoware_control` attribute
([CARLA's Autoware page](https://github.com/carla-simulator/carla/blob/1.0/Docs/autoware.md)).
This package is the client side that is left, as a single rclpy node. It talks to the server
through [typesafe_carla](https://github.com/hakuturu583/typesafe_carla) (`typesafe_carla.carla`,
LibCarla 1.0) instead of the official `carla` Python package.

| The CARLA server does                                         | This node does                                                                   |
| ------------------------------------------------------------- | -------------------------------------------------------------------------------- |
| Publishes the sensors on Autoware's topics                    | Spawns the sensor kit from `config/sensor_kit.yaml` and enables it for ROS       |
| Publishes `/vehicle/status/*` (`sensor.other.vehicle_status`) | Spawns the ego with `ros2_autoware_control` and the vehicle status sensor        |
| Applies `/control/command/control_cmd`                        | Applies turn-indicator and hazard commands, which the server ignores             |
| Publishes `/clock`                                            | Loads the level, sets synchronous mode and ticks the world at real-time pace     |
| —                                                             | Publishes CARLA's traffic-light states on Autoware's external input (optional)   |
| —                                                             | Moves the ego to an `/initialpose` set in RViz                                   |
| —                                                             | Destroys what it spawned and restores the world settings on exit                 |

It succeeds
[`autoware_carla_interface`](https://github.com/autowarefoundation/autoware_universe/tree/main/simulator/autoware_carla_interface)
from Autoware Universe (CARLA 0.9.15 / 0.10), which converted sensors and control in Python.
None of that conversion is needed with CARLA 1.0.

## Supported environment

| Ubuntu | ROS 2 |           CARLA           |     CARLA client     | Autoware |
| :----: | :---: | :-----------------------: | :------------------: | :------: |
| 24.04  | Jazzy | 1.0, started with `--ros2` | typesafe-carla 0.8.x |  2.0.0   |

## Install

1. **CARLA 1.0**: download `CARLA_1.0.tar.gz` from the
   [release](https://github.com/carla-simulator/carla/releases/tag/1.0). The official `carla`
   Python package is not needed.
2. **typesafe_carla** is rosdistro's rosdep key `python3-typesafe-carla-pip`, so the
   `rosdep install` below installs it with pip; on Ubuntu 24.04 rosdep only runs pip with
   `PIP_BREAK_SYSTEM_PACKAGES=1` set (PEP 668). The wheel carries LibCarla built for CARLA 1.0,
   so nothing is compiled. The key is not pinned: it installs the latest release, and this
   package is written against 0.8.x (0.8.7 or later). To pin it, install it yourself and skip
   the key: `pip install "typesafe-carla>=0.8.7,<0.9"` (add `--break-system-packages` for
   Ubuntu 24.04's system Python), then `rosdep install ... --skip-keys python3-typesafe-carla-pip`.

3. Build:

   ```bash
   PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install --from-paths src --ignore-src -y
   colcon build --packages-up-to autoware_carla_interface_v2
   ```

## Run

```bash
# 1. CARLA with ROS 2 (add -nullrhi to run without a GPU; cameras are then skipped, see below).
#    Two dashes: the server only reads `--ros2`; `-ros2` starts it with ROS 2 off, silently.
#    -rmw=cyclonedds with Autoware's own Cyclone DDS configuration (the dev container's
#    ~/cyclonedds.xml, loopback only) is what this was run with.
CYCLONEDDS_URI=file://$HOME/cyclonedds.xml ./CarlaUnreal.sh --ros2 -rmw=cyclonedds -RenderOffScreen -nosound

# 2. This node: loads the level, spawns the ego and sensors, ticks the world
ros2 launch autoware_carla_interface_v2 autoware_carla_interface_v2.launch.xml \
    map_path:=$HOME/autoware_data/maps/Town10HD_Opt

# 3. Autoware, with the sensor kit and vehicle the default sensor_kit.yaml mirrors
ros2 launch autoware_launch autoware.launch.xml \
    map_path:=$HOME/autoware_data/maps/Town10HD_Opt \
    vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit use_sim_time:=true
```

Autoware needs a map directory for the level (`pointcloud_map.pcd`, `lanelet2_map.osm`,
`map_projector_info.yaml`); CARLA's `PythonAPI/examples/av_stacks/autoware/map_tools` ships
`Town10HD` and can generate the others.

Raise the UDP socket buffers to 64 MB before starting any ROS 2 process, or the LiDAR stream
overruns them (CARLA's Autoware page explains why):

```bash
sudo sysctl -w net.core.rmem_max=67108864 net.core.wmem_max=67108864
```

`ros2 run autoware_carla_interface_v2 spectator_follow` keeps the CARLA spectator camera behind
the ego.

## With autoware_carla_scenario

[autoware_carla_scenario](https://github.com/autowarefoundation/autoware_carla_scenario) runs the
session, and its Autoware launcher starts and kicks Autoware for every scenario: once the
scenario's ego exists, the launcher runs one `ros2 launch` plus the framework's `scenario_bridge`
node, which initializes localization, sets the route and engages through the AD API. Nothing in
this package initializes or kicks Autoware.

`launch/autoware_carla_scenario.launch.xml` is that one launch: Autoware (`autoware_launch`'s
`e2e_simulator.launch.xml` with `simulator_type:=carla` and `launch_simulator_interface:=false`,
so Universe's interface is not started) plus this node in `scenario_mode`. Point the launcher at
it:

```bash
uv run scenario 'scenario=intersection_passing/*' map=town10hd_opt \
  ego.entity=autoware \
  autoware.launcher.type=docker \
  autoware.launcher.workspace=$HOME/autoware \
  'autoware.launcher.launch=[autoware_carla_interface_v2,autoware_carla_scenario.launch.xml,"map_path:=/home/aw/autoware_map/Town10HD_Opt"]'
```

The workspace (a clone of `autowarefoundation/autoware`) needs this package in `src/`. The docker
launcher refuses a workspace whose launch has no `scenario_mode`; it currently looks only at
`autoware_carla_interface`'s launch file, so it needs the change that also accepts the configured
launch file (this one declares `scenario_mode`). The `command` launcher works without it:

```bash
uv run scenario ego.entity=autoware autoware.launcher.type=command \
  'autoware.launcher.command=[bash,-c,"source ~/autoware/install/setup.bash && ({scenario_bridge} &) && exec ros2 launch autoware_carla_interface_v2 autoware_carla_scenario.launch.xml map_path:=$HOME/autoware_data/maps/Town10HD_Opt"]'
```

In `scenario_mode` the node waits up to `ego_wait_timeout` for the ego the scenario spawned
(role name `Ego`, or `ego` with `ego.native_autoware_interface`), spawns the sensor kit on it and does nothing else to the session: it loads no level,
changes no settings, never ticks, and ignores `/initialpose` (the scenario places the ego). On
shutdown, when the launcher stops the stack at the end of the scenario, it removes only its
sensors.

CARLA serves Autoware's vehicle interface natively only for a vehicle spawned with
`ros2_autoware_control` and the role name `ego` or `hero`. Run the scenario with
`ego.native_autoware_interface=true` (autoware_carla_scenario 26.6.0 and later) and it spawns its
ego that way: the server then applies Autoware's control and publishes the vehicle status, and
the node spawns the `sensor.other.vehicle_status` sensor on the ego. Without it the ego is `Ego`,
which CARLA does not serve, and the node provides the interface from the client
(`vehicle_interface:=auto`, the default, decides from the ego's attributes):

- `/control/command/control_cmd` is applied as CARLA Ackermann control: the server turns the
  target speed, acceleration and steering tire angle into pedals and steering, so no actuation
  maps are needed. `/control/command/gear_cmd` selects reverse and park.
- `/vehicle/status/{velocity,steering,gear,control_mode,turn_indicators,hazard_lights}_status`
  are published on every simulation frame (`World.on_tick`), stamped with the frame's time.

`autoware_carla_scenario.launch.xml` arguments: `map_path` (required), `vehicle_model`
(`sample_vehicle`), `sensor_model` (`awsim_sensor_kit`, matching the default sensor kit),
`sensor_kit_file`, `data_path`, `rviz` (`false`), `host`, `port`, `ego_role_name` (empty: `Ego`, `ego` or `hero`),
`ego_wait_timeout`, `vehicle_interface`, `no_rendering_mode`, `traffic_light.publish` (`true`,
matched against `map_path/lanelet2_map.osm`), `map_origin_x`, `map_origin_y`.

## Parameters

| Name                                                        | Default                                                          | Description                                                                                                                             |
| ----------------------------------------------------------- | ---------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| `host`, `port`, `timeout`                                   | `localhost`, `2000`, `20.0`                                      | CARLA server and RPC timeout [s].                                                                                                       |
| `carla_map`                                                 | `""`                                                             | Level to load (`load_world_if_different`); empty keeps the current level. The launch file derives it from `map_path`.                  |
| `force_load_world`                                          | `false`                                                          | Reload the level even if it is already loaded.                                                                                          |
| `sync_mode`                                                 | `true`                                                           | Synchronous mode, ticked by this node. Run no other ticking client.                                                                     |
| `fixed_delta_seconds`                                       | `0.05`                                                           | Simulation step [s]. CARLA validates Autoware at 20 Hz; sensor rates follow the step.                                                   |
| `time_scale`, `resync`                                      | `1.0`, `true`                                                    | Simulation speed relative to wall time; jump ahead when the loop falls behind.                                                         |
| `no_rendering_mode`                                         | `false`                                                          | Disable rendering. Cameras are then not spawned: a renderer-less (`-nullrhi`) server crashes on them.                                   |
| `publish_tf`                                                | `false`                                                          | Let CARLA publish `/tf`. Off because Autoware publishes the vehicle's tree from its URDF.                                               |
| `scenario_mode`                                             | `false`                                                          | Attach to autoware_carla_scenario's ego, level and clock instead of providing them (see above).                                         |
| `ego_wait_timeout`                                          | `60.0`                                                           | `scenario_mode`: how long to wait for the ego [s].                                                                                      |
| `ego_role_name`                                             | `""`                                                             | Empty: in `scenario_mode` any of `Ego`, `ego`, `hero`; otherwise `ego`. CARLA's native interface only serves `ego` and `hero`.          |
| `vehicle_interface`                                         | `auto`                                                           | `native`: CARLA applies control and publishes the status; `client`: this node does; `auto`: native when the ego is registered for it. |
| `vehicle_blueprint`                                         | `""`                                                             | Ego blueprint; empty uses the sensor kit file's (`vehicle.lincoln.mkz`, the vehicle CARLA tuned steering for).                         |
| `spawn_point`                                               | `""`                                                             | `x,y,z,roll,pitch,yaw` in CARLA coordinates (m, deg). Empty uses `spawn_index`.                                                        |
| `spawn_index`                                               | `-1`                                                             | Index into the level's ego spawn points (or map spawn points); negative picks one at random.                                            |
| `sensor_kit_file`                                           | `config/sensor_kit.yaml`                                         | The sensors to spawn, see below.                                                                                                        |
| `map_origin_x`, `map_origin_y`                              | `0.0`                                                            | CARLA world origin in the Autoware map frame, for `/initialpose` and traffic-light matching.                                            |
| `initial_pose_z_offset`                                     | `0.5`                                                            | Height above the ground the ego is placed at on `/initialpose` [m].                                                                     |
| `apply_light_commands`                                      | `true`                                                           | Apply `/control/command/turn_indicators_cmd` and `hazard_lights_cmd` to the ego's lights.                                               |
| `traffic_light.publish`                                     | `false`                                                          | Publish CARLA's traffic-light states as `TrafficLightGroupArray`.                                                                       |
| `traffic_light.topic`                                       | `/perception/traffic_light_recognition/external/traffic_signals` | Autoware's external input, so the camera classifier, which does not recognise CARLA's lights, is not merged in.                        |
| `traffic_light.rate_hz`                                     | `10.0`                                                           | Publishing rate.                                                                                                                        |
| `traffic_light.map_path`                                    | `""`                                                             | lanelet2 map: match each CARLA light to the nearest regulatory element by position. Empty uses the OpenDRIVE signal id as the group id. |
| `traffic_light.match_distance`, `traffic_light.match_ratio` | `5.0`, `0.6`                                                     | Matching distance [m] and ambiguity ratio; ambiguous lights are reported, not guessed.                                                  |
| `traffic_light.id_map`                                      | `""`                                                             | `opendrive_id:group_id[\|group_id...],...` overrides for lights the matcher leaves out.                                                 |

## Sensor kit

`config/sensor_kit.yaml` describes the vehicle and the sensors CARLA publishes. The default
follows the AWSIM sensor kit, as CARLA's own `PythonAPI/examples/autoware_demo.py` does:
a 16-channel top LiDAR on `/sensing/lidar/top/pointcloud_raw_ex`, the traffic-light camera on
`/sensing/camera/traffic_light/{image,camera_info}`, the IMU on `/sensing/imu/tamagawa/imu_raw`
and `sensor.other.autoware_gnss` on `/sensing/gnss/{pose,pose_with_covariance}`.

```yaml
vehicle:
  blueprint: vehicle.lincoln.mkz
  base_link: {x: -1.39706787} # CARLA vehicle origin -> base_link
frames:
  sensor_kit_base_link: {parent: base_link, pose: {x: 0.9, z: 2.0}}
sensors:
  lidar_top:
    blueprint: sensor.lidar.ray_cast
    parent: sensor_kit_base_link
    pose: {yaw: 1.575} # ROS convention: meters, radians
    frame_id: velodyne_top # the ros_name attribute
    topic: /sensing/lidar/top/pointcloud_raw_ex # the ros_topic_name attribute
    attributes: {channels: 16, sensor_tick: 0.1}
```

Each sensor is attached to the vehicle with its pose resolved through the frame chain. Camera
topics get CARLA's `/image` and `/camera_info` suffixes; LiDAR topics are used verbatim.

## Tests

```bash
colcon test --packages-select autoware_carla_interface_v2
```

Without the runtime dependencies (as in the Autoware Index CI) only the tests that need none
run. `test/test_carla_server.py` runs the node against a real server and is skipped unless
`CARLA_HOST` is set:

```bash
./CarlaUnreal.sh --ros2 -nullrhi -nosound
CARLA_HOST=localhost python3 -m pytest test/test_carla_server.py -s
```

It checks the server and LibCarla versions, starts the node (ego with `ros2_autoware_control`,
vehicle status sensor, the non-camera sensors), lets it tick, and checks that shutdown removes
everything and restores the world settings. A second test spawns an `Ego` as
autoware_carla_scenario does and checks that `scenario_mode` attaches to it, does not tick,
falls back to the client-side vehicle interface and leaves the ego in place on shutdown.

## typesafe_carla notes

`autoware_carla_interface_v2/carla_client.py` selects LibCarla 1.0
(`TYPESAFE_CARLA_CARLA_REF=1.0` unless the environment chose another) and holds the adaptations
the code relies on: actors come back as plain `carla.Actor`s (`as_vehicle()`, `as_sensor()`,
`as_traffic_light()` reach the kind-specific methods), sensor callbacks receive a
`carla.SensorData`, and `carla.VehicleLightState` is an `IntEnum`, so combined flags are passed
as an `int`.

## Known limitations

- With CARLA's native interface, gear, emergency and engage commands are not applied (CARLA
  receives them and does nothing; `control_mode` always reports `AUTONOMOUS`). The client-side
  interface applies reverse and park, and also reports `AUTONOMOUS`.
- Steering is tuned for the Lincoln MKZ (CARLA's measurement); other vehicles track more loosely.
- Run against the CARLA 1.0 nightly (`ue58-dev` d93dfef63) with Autoware universe 0.53.2 beside
  autoware_carla_scenario: the node attaches to the scenario's ego, spawns the sensor kit, and
  Autoware localizes, routes, engages and drives to its goal through both vehicle interfaces.
  Two server-side fixes are needed with current Autoware:
  - [carla-simulator/carla#10206](https://github.com/carla-simulator/carla/pull/10206): without it
    the LiDAR `azimuth` runs against the scan and Autoware 0.53 or later drops every point in its
    ring outlier filter, so localization never initializes.
  - [carla-simulator/carla#10216](https://github.com/carla-simulator/carla/pull/10216): without it
    the native interface receives no command from `control_command_gate` (autoware_launch after
    0.53.0), so the ego does not move; the client-side interface (`vehicle_interface:=client`)
    is not affected.
