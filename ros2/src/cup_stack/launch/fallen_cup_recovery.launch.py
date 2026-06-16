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
                "place_cup_tilt_deg",
                default_value="8.0",
                description="place 모드 cup tilt(deg). 서버 API가 :=값 으로 전달 → "
                            "inner stand_fallen_cup.launch.py 로 forward.",
            ),
            DeclareLaunchArgument(
                "place_plus_y_cup_tilt_deg",
                default_value="8.0",
                description="place 모드 +Y auto_swing cup tilt(deg). inner launch 로 forward.",
            ),
            # ── 작업영역 컵 회피 / PLACE 위치 (inner launch 로 forward) ──
            #   회피 끄고 실험: avoid_upright_cups:=false (정상컵 궤적회피 OFF),
            #   place_x/place_y 지정(단일컵 고정 PLACE → 스마트 회피선택 OFF) 또는
            #   place_in_place:=true(멀티컵 제자리 → 스마트 회피선택 OFF),
            #   pyramid_avoid:=false (피라미드 회피 OFF).
            DeclareLaunchArgument(
                "avoid_upright_cups", default_value="true",
                description="false 면 정상(세워진) 컵 궤적 충돌회피(실린더 등록) OFF",
            ),
            DeclareLaunchArgument(
                "pyramid_avoid", default_value="true",
                description="false 면 피라미드 영역 회피 OFF",
            ),
            DeclareLaunchArgument(
                "place_in_place", default_value="false",
                description="true 면 멀티컵에서 제자리 세우기(스마트 PLACE 선택 OFF)",
            ),
            DeclareLaunchArgument(
                "place_x", default_value="nan",
                description="단일컵 PLACE X 고정(지정 시 스마트 선택 OFF). nan=자동",
            ),
            DeclareLaunchArgument(
                "place_y", default_value="nan",
                description="단일컵 PLACE Y 고정(지정 시 스마트 선택 OFF). nan=자동",
            ),
            DeclareLaunchArgument(
                "descend_min_z", default_value="nan",
                description="grasp 하강 flange(link_6) 최저 z(m) — 바닥 충돌 방지 하드 "
                            "플로어. nan=테이블 레벨 자동. TCP 최저=이값−0.20.",
            ),
            DeclareLaunchArgument(
                "place_spot_candidates",
                default_value="0.30:0.20,0.30:0.15,0.30:0.10,0.30:0.05,0.30:0.00,"
                              "0.30:-0.05,0.30:-0.10,0.30:-0.15,0.30:-0.20",
                description="빈자리 후보 'x:y,...' (컵 Y측 가장자리부터 검사). ±0.20 기본.",
            ),
            DeclareLaunchArgument(
                "place_spot_avoid_radius_m", default_value="0.09",
                description="후보를 '점유'로 볼 회피 반경(m).",
            ),
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
                            "cup_yaw_override_deg": LaunchConfiguration(
                                "cup_yaw_override_deg"
                            ),
                            "place_cup_tilt_deg": LaunchConfiguration(
                                "place_cup_tilt_deg"
                            ),
                            "place_plus_y_cup_tilt_deg": LaunchConfiguration(
                                "place_plus_y_cup_tilt_deg"
                            ),
                            # 작업영역 컵 회피 / PLACE 위치 (회피 끄기·튜닝용).
                            "avoid_upright_cups": LaunchConfiguration(
                                "avoid_upright_cups"
                            ),
                            "pyramid_avoid": LaunchConfiguration("pyramid_avoid"),
                            "place_in_place": LaunchConfiguration("place_in_place"),
                            "place_x": LaunchConfiguration("place_x"),
                            "place_y": LaunchConfiguration("place_y"),
                            "descend_min_z": LaunchConfiguration("descend_min_z"),
                            "place_spot_candidates": LaunchConfiguration(
                                "place_spot_candidates"
                            ),
                            "place_spot_avoid_radius_m": LaunchConfiguration(
                                "place_spot_avoid_radius_m"
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
