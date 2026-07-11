from setuptools import find_packages, setup

package_name = "led_strip_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/led_params.yaml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="sverk",
    maintainer_email="petayyyy@gmail.com",
    description="Hardware-free WS2812 LED controller for Obrik Gazebo simulation",
    license="MIT",
    entry_points={
        "console_scripts": ["led_strip_sim_node = led_strip_sim.led_node:main"],
    },
)
