from setuptools import setup

package_name = "graffiti_servo_sim"

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
    description="ROS 2 simulator node for the graffiti spray servo kinematics.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "graffiti_servo_sim_node = graffiti_servo_sim.servo_node:main",
        ],
    },
)
