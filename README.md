# autoware_carla_interface_v2

Autoware bridge for the [CARLA 1.0](https://github.com/carla-simulator/carla/releases/tag/1.0)
simulator (Unreal Engine 5.8), talking to CARLA through the
[typesafe_carla](https://github.com/hakuturu583/typesafe_carla) client.

It is a port of
[`autoware_carla_interface`](https://github.com/autowarefoundation/autoware_universe/tree/main/simulator/autoware_carla_interface)
from Autoware Universe to CARLA 1.0, packaged as a standalone repository for the
[Autoware Index](https://github.com/autowarefoundation/autoware-index).

| Package                                                      | Description                       |
| ------------------------------------------------------------ | --------------------------------- |
| [`autoware_carla_interface_v2`](autoware_carla_interface_v2) | ROS 2 bridge between Autoware and CARLA 1.0 |

See the [package README](autoware_carla_interface_v2/README.md) for installation, usage and the
differences from the Autoware Universe package.

## Quick start

```bash
mkdir -p ~/carla_ws/src && cd ~/carla_ws/src
git clone https://github.com/hakuturu583/autoware_carla_interface_v2.git
cd ~/carla_ws
pip install "typesafe-carla>=0.8.7,<0.9"   # or use rosdep/typesafe_carla.yaml
rosdep install --from-paths src --ignore-src -y --skip-keys python3-typesafe-carla-pip
colcon build --packages-up-to autoware_carla_interface_v2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## License

Apache License 2.0. typesafe_carla is MIT licensed and bundles LibCarla (MIT).
