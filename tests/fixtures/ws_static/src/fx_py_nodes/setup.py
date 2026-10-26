from setuptools import find_packages, setup

package_name = "fx_py_nodes"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fixture",
    maintainer_email="dev@example.invalid",
    description="Python executables whose names differ from their modules.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "fx_watch = fx_py_nodes.watchdog:main",
            "fx_report = fx_py_nodes.report_node:main",
        ],
    },
)
