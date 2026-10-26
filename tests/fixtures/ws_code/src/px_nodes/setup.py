from setuptools import find_packages, setup

package_name = "px_nodes"

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
    description="rclpy endpoint, parameter and TF idioms the coded layer has to read.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "px_literal = px_nodes.literal_node:main",
            "px_param = px_nodes.param_node:main",
            "px_tf = px_nodes.tf_node:main",
            "px_health = px_nodes.wrappers_node:main",
        ],
    },
)
