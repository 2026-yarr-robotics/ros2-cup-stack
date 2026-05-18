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
            ["package.xml"] + glob("bringup_*.sh") + glob("build_*.sh"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ssu",
    maintainer_email="ssu@todo.todo",
    description="ROS 2 speed stacking control tasks for a Doosan M0609 robot.",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "cup_pyramid = cup_stack.nodes.cup_pyramid_node:main",
            "cup_pyramid_skill = "
            "cup_stack.nodes.cup_pyramid_skill_node:main",
            "cup_pyramid_select = "
            "cup_stack.nodes.cup_pyramid_select_node:main",
            "cup_pyramid_web = "
            "cup_stack.nodes.cup_pyramid_web_node:main",
            "cup_unstack = cup_stack.nodes.cup_unstack_node:main",
            "cup_unstack_select = "
            "cup_stack.nodes.cup_unstack_select_node:main",
            "cup_unstack_web = "
            "cup_stack.nodes.cup_unstack_web_node:main",
            "camera_capture = cup_stack.nodes.camera_capture_node:main",
            "move_cartesian = cup_stack.nodes.move_cartesian_node:main",
            "gripper_node = cup_stack.nodes.gripper_node:main",
            "find_vertical_home = cup_stack.nodes.find_vertical_home_node:main",
            "scan = cup_stack.nodes.scan_node:main",
            "scan_skill = cup_stack.nodes.scan_skill_node:main",
            "move_to_pos1 = cup_stack.nodes.move_to_pos1_node:main",
            "skill_api_server = cup_stack.nodes.skill_api_node:main",
            "skill_api_client = cup_stack.api_client:main",
        ],
    },
)
