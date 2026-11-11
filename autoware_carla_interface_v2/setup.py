from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = "autoware_carla_interface_v2"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config/hils", glob("config/hils/*")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Masaya Kataoka",
    maintainer_email="masaya.kataoka@tier4.jp",
    description="Autoware on CARLA 1.0's native ROS 2 interface, driven through typesafe_carla",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "autoware_carla_interface_v2 = autoware_carla_interface_v2.node:main",
            "spectator_follow = autoware_carla_interface_v2.spectator_follow:main",
            "hils_vehicle_interface = autoware_carla_interface_v2.hils_vehicle_interface:main",
        ],
    },
)
