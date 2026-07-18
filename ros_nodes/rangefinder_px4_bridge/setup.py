from setuptools import setup

package_name = "rangefinder_px4_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sverk",
    maintainer_email="petayyyy@gmail.com",
    description="Bridge a simulated downward LaserScan into PX4 DistanceSensor.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rangefinder_px4_bridge = rangefinder_px4_bridge.node:main",
        ],
    },
)
