"""Launch the fallen-cup recovery (stand-up) task under the robot namespace.

Wrapper around ``dsr_practice/launch/stand_fallen_cup.launch.py`` (from the
fallen-cup-recovery repo, symlinked into this workspace) that:

  1. spawns ``dsr_moveit_controller`` against ``/<ns>/controller_manager``
     (bringup only spawns dsr_controller2 — same trick as skill_api.launch.py),
  2. pushes the robot namespace (default ``dsr01``) onto the included node, and
  3. passes ``robot_namespace`` so the node hands ``name_space`` + ``__ns``
     remap to MoveItPy — launch-side namespacing alone does NOT reach
     MoveItPy's internal rclcpp context (see cup_stack/runtime.py).

One-shot task: senses /fallen_cup/* topics published by the
``fallen_cup_detect`` service, picks up the fallen cup(s), stands them, then
returns HOME and exits.

Launched by the dashboard server as the ``fallen_cup_recovery`` TASK command.
"""

import os

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

# Isaac 디지털 트윈 브링업(start_isaac.sh)이 bringup 에이전트 창에 심는 env —
# 실기(env 미설정)는 기존 기본값 그대로다.
#   FALLEN_CUP_GRIPPER_BACKEND=topic  → 그리퍼를 SimRG 토픽 경로로
#   FALLEN_CUP_PLACE_TILT_DEG=20.0    → stand release 를 기울여서 (중력 세움
#                                       모델; Isaac bridge 가 wobble 로 연출)
#   ROBOT_API_BASE=http://localhost   → pyramid config 폴링을 로컬 서버로 핀
#                                       (기본 Cloudflare PROD 는 sim 에서 오답)
_GRIPPER_BACKEND_DEFAULT = os.environ.get("FALLEN_CUP_GRIPPER_BACKEND", "")
_PLACE_TILT_DEFAULT = os.environ.get("FALLEN_CUP_PLACE_TILT_DEG", "0.0")
# Isaac: /hand_eye/boxes 의 3class 모델이 전도 컵까지 upright 로 과검출해
# 그립 타깃 자리에 유령 장애물을 만든다 → 접근 planning 이 자기 장애물과
# 충돌해 실패. sim 에선 끈다 (실기 기본 true 유지).
_AVOID_UPRIGHT_DEFAULT = os.environ.get("FALLEN_CUP_AVOID_UPRIGHT", "true")
_API_BASE = os.environ.get("ROBOT_API_BASE", "").rstrip("/")
_PYRAMID_CFG_DEFAULT = (
    f"{_API_BASE}/api/robot/config/pyramid" if _API_BASE
    else "https://yarr-api-31.simplyimg.com/api/robot/config/pyramid"
)


def generate_launch_description():
    namespace = LaunchConfiguration("name")

    inner_launch = PathJoinSubstitution(
        [FindPackageShare("dsr_practice"), "launch", "stand_fallen_cup.launch.py"]
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
                "echo '[recovery] dsr_moveit_controller already active -> skip spawn'; "
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
                description="lift 후 동작: 'drop' | 'place'",
            ),
            DeclareLaunchArgument(
                "multi_cup",
                default_value="false",
                description="여러 fallen cup 순차 처리",
            ),
            DeclareLaunchArgument(
                "multi_cup_max_iterations",
                default_value="10",
                description="multi_cup 모드 안전 limit",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="false",
                description="approach까지만 (gripper/descend/lift 스킵)",
            ),
            DeclareLaunchArgument(
                "sim",
                default_value="false",
                description="카메라/그리퍼 HW 우회 (MoveIt virtual용)",
            ),
            DeclareLaunchArgument(
                "cup_yaw_override_deg",
                default_value="nan",
                description="NaN이 아니면 인식 yaw 무시하고 강제 값 사용",
            ),
            DeclareLaunchArgument(
                "gripper_backend",
                default_value=_GRIPPER_BACKEND_DEFAULT,
                description="''(레거시) | onrobot | topic(Isaac SimRG) | none "
                            "— 기본값은 env FALLEN_CUP_GRIPPER_BACKEND",
            ),
            DeclareLaunchArgument(
                "place_cup_tilt_deg",
                default_value=_PLACE_TILT_DEFAULT,
                description="place 모드 release 기울임(deg) — 기본값은 env "
                            "FALLEN_CUP_PLACE_TILT_DEG (Isaac: 20)",
            ),
            DeclareLaunchArgument(
                "pyramid_config_url",
                default_value=_PYRAMID_CFG_DEFAULT,
                description="pyramid center/degree 폴링 엔드포인트 — env "
                            "ROBOT_API_BASE 가 있으면 그쪽으로 핀",
            ),
            DeclareLaunchArgument(
                "avoid_upright_cups",
                default_value=_AVOID_UPRIGHT_DEFAULT,
                description="정상 컵 장애물 등록 — env FALLEN_CUP_AVOID_UPRIGHT "
                            "(Isaac: false — 과검출 유령 장애물 방지)",
            ),
            # sim 모드 가상 컵 포즈 — 검증 툴(verify_recovery.py)이 Isaac GT
            # 포즈를 주입해 인식 우회 E2E 를 돌릴 수 있게 패스스루.
            DeclareLaunchArgument("sim_cup_x", default_value="0.28"),
            DeclareLaunchArgument("sim_cup_y", default_value="0.20"),
            DeclareLaunchArgument("sim_cup_z", default_value="0.10"),
            DeclareLaunchArgument("sim_cup_yaw_deg", default_value="0.0"),
            dsr_moveit_controller_spawner,
            GroupAction(
                [
                    PushRosNamespace(namespace),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(inner_launch),
                        launch_arguments={
                            "mode": LaunchConfiguration("mode"),
                            "multi_cup": LaunchConfiguration("multi_cup"),
                            "multi_cup_max_iterations": LaunchConfiguration(
                                "multi_cup_max_iterations"
                            ),
                            "dry_run": LaunchConfiguration("dry_run"),
                            "sim": LaunchConfiguration("sim"),
                            "gripper_backend": LaunchConfiguration(
                                "gripper_backend"
                            ),
                            "place_cup_tilt_deg": LaunchConfiguration(
                                "place_cup_tilt_deg"
                            ),
                            "pyramid_config_url": LaunchConfiguration(
                                "pyramid_config_url"
                            ),
                            "avoid_upright_cups": LaunchConfiguration(
                                "avoid_upright_cups"
                            ),
                            "sim_cup_x": LaunchConfiguration("sim_cup_x"),
                            "sim_cup_y": LaunchConfiguration("sim_cup_y"),
                            "sim_cup_z": LaunchConfiguration("sim_cup_z"),
                            "sim_cup_yaw_deg": LaunchConfiguration(
                                "sim_cup_yaw_deg"
                            ),
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
