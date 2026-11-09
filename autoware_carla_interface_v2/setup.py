from setuptools import find_packages
from setuptools import setup

package_name = "autoware_carla_interface_v2"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(where="src"),
    install_requires=["setuptools", "typesafe-carla>=0.8.7,<0.9"],
    zip_safe=True,
    maintainer="Masaya Kataoka",
    maintainer_email="masaya.kataoka@tier4.jp",
    description="CARLA 1.0 ROS 2 bridge for Autoware, using the typesafe_carla client",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "autoware_carla_interface_v2 = autoware_carla_interface_v2.carla_autoware:main",
            "multi_camera_combiner = autoware_carla_interface_v2.multi_camera_combiner_node:main",
        ],
    },
    package_dir={"": "src"},
)
