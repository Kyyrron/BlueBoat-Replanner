from glob import glob

from setuptools import find_packages, setup

package_name = "blueboat_replanner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Killian BERTRAND",
    maintainer_email="killian.bertrand2002@gmail.com",
    description="Aspect-aware replanner for the BlueBoat side-scan survey (D6)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "replanner_node = blueboat_replanner.ros.replanner_node:main",
            "replanner_emulator_node = blueboat_replanner.eval.emulator_node:main",
            "replanner-devloop = blueboat_replanner.eval.runner:main",
        ],
    },
)
