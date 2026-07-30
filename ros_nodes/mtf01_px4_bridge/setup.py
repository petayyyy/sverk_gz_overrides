from setuptools import setup

package_name = "mtf01_px4_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sverk",
    maintainer_email="petayyyy@gmail.com",
    description="Estimate MTF-01 optical flow and publish PX4 SensorOpticalFlow.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mtf01_px4_bridge = mtf01_px4_bridge.node:main",
        ],
    },
)
