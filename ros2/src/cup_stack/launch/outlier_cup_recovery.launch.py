"""Launch the outlier-cup recovery (fallen + mouth-up) task under the robot namespace.

Wrapper around ``dsr_practice/launch/outlier_cup_recovery.launch.py`` (from the
outlier-cup-recovery repo, symlinked into this workspace) that:

  1. spawns ``dsr_moveit_controller`` against ``/<ns>/controller_manager``
     (bringup only spawns dsr_controller2 — same trick as fallen_cup_recovery.launch.py),
  2. pushes the robot namespace (default ``dsr01``) onto the included node, and
  3. passes ``robot_namespace`` so the node hands ``name_space`` + ``__ns``
     remap to MoveItPy — launch-side namespacing alone does NOT reach
     MoveItPy's internal rclcpp context (see cup_stack/runtime.py).

One-shot orchestrator task: the unified ``outlier_cup_recovery`` node senses
``/fallen_cup/*`` then ``/mouth_up_cup/*`` topics, stands every fallen cup first
(base_link nearest-first), then flips every mouth-up cup, returns HOME and exits.
``multi_cup`` is forced ON by the orchestrator, so it is NOT exposed here.

Launched by the dashboard server as the ``outlier_cup_recovery`` TASK command.
The fallen-only ``fallen_cup_recovery`` command (stand_fallen_cup) is kept
separately — this is the superset orchestrator.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("name")

    inner_launch = PathJoinSubstitution(
        [FindPackageShare("dsr_practice"), "launch", "outlier_cup_recovery.launch.py"]
    )

    # Bringup (dsr_bringup2) spawns only dsr_controller2. MoveIt needs a
    # standard FollowJointTrajectory controller (dsr_moveit_controller).
    # bringup_real_31.sh / skill_api.launch.py usually already spawn+activate it;
    # re-spawning an *active* controller makes the spawner fail at the configure
    # step ("Failed to configure controller", exit 1) — noisy but harmless.
    # So skip the spawn when it is already active; otherwise spawn it. The
    # `timeout` covers the standalone case where controller_manager isn't up yet
    # (query fails -> grep no-match -> spawn).
    dsr_moveit_controller_spawner = ExecuteProcess(
        cmd=[
            "bash", "-c",
            [
                "if timeout 5 ros2 control list_controllers -c /", namespace,
                "/controller_manager 2>/dev/null "
                "| grep -qE 'dsr_moveit_controller.*active'; then "
                "echo '[outlier] dsr_moveit_controller already active -> skip spawn'; "
                "else ros2 run controller_manager spawner dsr_moveit_controller "
                "--controller-manager /", namespace, "/controller_manager; fi",
            ],
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "name",
                default_value="dsr01",
                description="Robot namespace; must match the bringup",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="drop",
                description="fallen lift 후 동작: 'drop' | 'place' (mouth-up 은 무관)",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="false",
                description="approach까지만 (gripper/insert/lift 스킵, 양 스킬 공통)",
            ),
            DeclareLaunchArgument(
                "sim",
                default_value="false",
                description="카메라/그리퍼 HW 우회 (MoveIt virtual용)",
            ),
            DeclareLaunchArgument(
                "cup_yaw_override_deg",
                default_value="nan",
                description="NaN이 아니면 fallen 인식 yaw 무시하고 강제 값 사용",
            ),
            dsr_moveit_controller_spawner,
            GroupAction(
                [
                    PushRosNamespace(namespace),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(inner_launch),
                        launch_arguments={
                            "mode": LaunchConfiguration("mode"),
                            "dry_run": LaunchConfiguration("dry_run"),
                            "sim": LaunchConfiguration("sim"),
                            "cup_yaw_override_deg": LaunchConfiguration(
                                "cup_yaw_override_deg"
                            ),
                            # MoveItPy는 launch namespace를 못 받으므로 노드가
                            # name_space + __ns remap을 직접 넘기도록 전달.
                            "robot_namespace": ["/", namespace],
                            # dsr_practice의 moveit_py.yaml은 루트 /joint_states를
                            # 바라봄 — 이 워크스페이스의 bringup은 /<ns>/joint_states
                            # 로 publish하므로 override (cup_stack moveit_py.yaml과 동일).
                            "joint_state_topic": ["/", namespace, "/joint_states"],
                        }.items(),
                    ),
                ]
            ),
        ]
    )
