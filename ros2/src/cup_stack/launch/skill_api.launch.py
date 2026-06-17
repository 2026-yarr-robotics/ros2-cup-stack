"""Launch the CupStack Skill API server with MoveItPy params.

Launch arguments:
  - ``host`` (default ``0.0.0.0``), ``port`` (default ``8765``)
  - ``move_home`` (default ``false``): move the arm to HOME before
    the server starts accepting requests.
  - ``cup_grip_z_offset`` (default ``0.10`` m): vertical distance
    from cup-top centre to the gripper grip point — calibrate to
    the actual cup geometry.
  - ``pick_z_base`` (default ``0.313`` m): gripper Z when picking
    the top of a 1-cup nested source stack. Used by /skill/pick when
    the caller supplies ``nested_count`` instead of an explicit Z.
  - ``nest_inc`` (default ``0.012`` m): rise per additional nested
    cup. ``pick_z = pick_z_base + (nested_count - 1) * nest_inc``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


# Bringup (dsr_bringup2_rviz.launch.py) places controller_manager and all
# controllers under /dsr01. MoveIt's moveit_simple_controller_manager
# creates its action client using the configured controller name as a
# *relative* name (`<controller>/follow_joint_trajectory`). Three
# launch-side tricks tried before — action sub-topic remaps, launch_ros'
# `namespace=`, and `ROS_NAMESPACE` env — all failed because MoveItPy
# rebuilds its own rclcpp context with only its internal launch_arguments
# and ignores the parent process's --ros-args.  The real fix lives in
# CupStackRuntime, which passes `name_space="/dsr01"` plus a `__ns` remap
# to MoveItPy so the new context's global args put both the primary node
# and the internal moveit_simple_controller_manager under /dsr01. The
# `namespace=` here is kept only to keep the skill_api_node rclpy node
# itself addressable as /dsr01/skill_api_node for symmetry with the rest
# of the graph.
ROBOT_NAMESPACE = "/dsr01"


def generate_launch_description():
    """Build the launch description for the skill API server."""

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="m0609",
            package_name="dsr_moveit_config_m0609",
        )
        .robot_description()
        .robot_description_semantic(file_path="config/dsr.srdf")
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution()
        .planning_scene_monitor()
        .sensors_3d()
        .to_moveit_configs()
    )

    moveit_py_params = PathJoinSubstitution(
        [FindPackageShare("cup_stack"), "config", "moveit_py.yaml"]
    )

    # dsr_bringup2_moveit.launch.py (the bringup used in Term1) already loads,
    # configures and activates dsr_moveit_controller on /dsr01. Spawning it again
    # here makes the controller_manager spawner fail the configure transition on
    # an already-active controller and die with exit 1. We therefore do NOT spawn
    # it here. If you instead bring up with dsr_bringup2_rviz.launch.py (which
    # spawns only dsr_controller2), add a controller_manager spawner for
    # dsr_moveit_controller back here.

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8765"),
            DeclareLaunchArgument("move_home", default_value="false"),
            DeclareLaunchArgument(
                "cup_grip_z_offset", default_value="0.10"
            ),
            DeclareLaunchArgument("pick_z_base", default_value="0.313"),
            DeclareLaunchArgument("nest_inc", default_value="0.012"),
            Node(
                package="cup_stack",
                executable="skill_api_server",
                namespace=ROBOT_NAMESPACE,
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    moveit_py_params,
                    {
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "move_home": LaunchConfiguration("move_home"),
                        "cup_grip_z_offset": LaunchConfiguration(
                            "cup_grip_z_offset"
                        ),
                        "pick_z_base": LaunchConfiguration("pick_z_base"),
                        "nest_inc": LaunchConfiguration("nest_inc"),
                    },
                ],
            ),
        ]
    )
