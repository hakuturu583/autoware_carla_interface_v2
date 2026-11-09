# autoware_carla_interface_v2

Runs Autoware on the [CARLA 1.0](https://github.com/carla-simulator/carla/releases/tag/1.0)
simulator (Unreal Engine 5.8) through CARLA's native ROS 2 interface. An rclpy node loads the
level, spawns the ego and its sensor kit and ticks the world, talking to CARLA through the
[typesafe_carla](https://github.com/hakuturu583/typesafe_carla) client; the server itself
publishes the sensors and vehicle status and applies Autoware's control commands.

It succeeds
[`autoware_carla_interface`](https://github.com/autowarefoundation/autoware_universe/tree/main/simulator/autoware_carla_interface)
from Autoware Universe, packaged as a standalone repository for the
[Autoware Index](https://github.com/autowarefoundation/autoware-index).

| Package                                                      | Description                       |
| ------------------------------------------------------------ | --------------------------------- |
| [`autoware_carla_interface_v2`](autoware_carla_interface_v2) | rclpy node running Autoware on CARLA 1.0 |

See the [package README](autoware_carla_interface_v2/README.md) for installation, usage and the
differences from the Autoware Universe package.

## Quick start

```bash
mkdir -p ~/carla_ws/src && cd ~/carla_ws/src
git clone https://github.com/hakuturu583/autoware_carla_interface_v2.git
cd ~/carla_ws
pip install "typesafe-carla>=0.8.7,<0.9"   # or use rosdep/typesafe_carla.yaml
rosdep install --from-paths src --ignore-src -y --skip-keys python3-typesafe-carla-pip
colcon build --packages-up-to autoware_carla_interface_v2
```

## License

Apache License 2.0. typesafe_carla is MIT licensed and bundles LibCarla (MIT).
