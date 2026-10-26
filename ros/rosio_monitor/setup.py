from setuptools import setup

package_name = "rosio_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/monitor.launch.py"]),
    ],
    install_requires=["setuptools", "rosio"],
    zip_safe=True,
    tests_require=["pytest"],
    extras_require={"test": ["pytest"]},
    maintainer="Elmir",
    maintainer_email="77459575+sa1ntsinner@users.noreply.github.com",
    description="Live header stamp and time source monitor for ROS 2, part of rosio",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "monitor = rosio_monitor.monitor_node:main",
            "check = rosio_monitor.check:main",
        ],
    },
)
