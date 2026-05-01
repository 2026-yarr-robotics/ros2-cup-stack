"""Set up the cup_stack ROS 2 package."""

from glob import glob

from setuptools import find_packages, setup


package_name = "cup_stack"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml") + glob("config/*.npy"),
        ),
        (
            "share/" + package_name,
            ["package.xml"] + glob("bringup_*.sh"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ssu",
    maintainer_email="ssu@todo.todo",
    description="ROS 2 cup stacking tasks split from dsr_practice.",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "cup_pyramid = cup_stack.nodes.cup_pyramid_node:main",
            "cup_unstack = cup_stack.nodes.cup_unstack_node:main",
        ],
    },
)
