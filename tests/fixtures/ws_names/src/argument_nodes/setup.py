from setuptools import setup

setup(
    name="argument_nodes",
    entry_points={
        "console_scripts": [
            "py_talker = argument_nodes.talker:main",
            "py_default = argument_nodes.default_node:main",
        ]
    },
)
