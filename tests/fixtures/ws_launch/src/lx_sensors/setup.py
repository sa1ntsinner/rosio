from setuptools import setup

package_name = "lx_sensors"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/lidar_driver.yaml"]),
        ("share/" + package_name + "/launch", ["launch/sensors_only.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rosio fixtures",
    maintainer_email="fixtures@example.invalid",
    description="Fixture sensor executables.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lidar_driver = lx_sensors.lidar_driver:main",
            "imu_driver = lx_sensors.imu_driver:main",
            "scan_relay = lx_sensors.scan_relay:main",
            # Entry point name and module name deliberately disagree.
            "health_monitor = lx_sensors.watchdog:main",
            "async_slam = lx_sensors.slam:main_async",
            "localization_slam = lx_sensors.slam:main_localization",
        ],
    },
)
