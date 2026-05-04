import os
from glob import glob
from setuptools import find_packages, setup

package_name = "hiking_lora_sim"

model_data_files = []
for path in glob("models/**/*", recursive=True):
    if os.path.isfile(path):
        model_data_files.append((os.path.join("share", package_name, os.path.dirname(path)), [path]))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/worlds", glob("worlds/*.sdf")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ]
    + model_data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ammar",
    maintainer_email="ammar@example.com",
    description="Gazebo and ROS 2 simulation of a hiker GPS tracker over LoRa relay nodes.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "hiker_agent = hiking_lora_sim.hiker_agent:main",
            "lora_network = hiking_lora_sim.lora_network:main",
            "base_station_display = hiking_lora_sim.base_station_display:main",
        ],
    },
)
