"""Launch the fallen-cup YOLO detection node against this workspace's cameras.

Wrapper around ``speed_stack_yolo_seg/launch/fallen_cup_pose.launch.py``
(from the fallen-cup-recovery repo, symlinked into this workspace) that
remaps the camera topics to the eye-in-hand RealSense launched by
``cameras_only.launch.py`` (``/hand/hand/...``).

Long-lived perception service: publishes
  /fallen_cup/pose2d            (std_msgs/Float32MultiArray)
  /fallen_cup/grasp_pose        (geometry_msgs/PoseStamped, camera optical frame)
  /fallen_cup/cups_pose2d       (std_msgs/Float32MultiArray, multi-cup)
  /fallen_cup/cups_grasp_poses  (geometry_msgs/PoseArray, multi-cup)
  /fallen_cup/debug_image       (sensor_msgs/Image, bgr8 overlay)

Launched by the dashboard server as the ``fallen_cup_detect`` SERVICE command.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    yolo_share = get_package_share_directory("speed_stack_yolo_seg")
    inner_launch = os.path.join(yolo_share, "launch", "fallen_cup_pose.launch.py")
    default_weights = os.path.join(yolo_share, "weights", "best.pt")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "weights_path",
                default_value=default_weights,
                description="YOLOv-seg weights (.pt). 절대경로 권장 — "
                            "기본값은 speed_stack_yolo_seg share의 weights/best.pt",
            ),
            DeclareLaunchArgument(
                "conf",
                default_value="0.70",
                description="YOLO confidence threshold",
            ),
            DeclareLaunchArgument(
                "imgsz",
                default_value="1280",
                description="YOLO input size",
            ),
            DeclareLaunchArgument(
                "use_depth",
                default_value="true",
                description="true면 depth로 3D grasp_pose publish",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="cuda",
                description="'cuda'(GPU) 또는 'cpu'. '0' 은 INTEGER 로 파싱돼 "
                            "STRING 파라미터와 타입 불일치 → 'cuda' 사용.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/hand/hand/color/image_raw",
                description="컬러 이미지 토픽 (기본: eye-in-hand RealSense)",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/hand/hand/aligned_depth_to_color/image_raw",
                description="정렬된 depth 토픽 (기본: eye-in-hand RealSense)",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/hand/hand/color/camera_info",
                description="camera_info 토픽 (기본: eye-in-hand RealSense)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(inner_launch),
                launch_arguments={
                    "weights_path": LaunchConfiguration("weights_path"),
                    "image_topic": LaunchConfiguration("image_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "conf": LaunchConfiguration("conf"),
                    "imgsz": LaunchConfiguration("imgsz"),
                    "use_depth": LaunchConfiguration("use_depth"),
                    "device": LaunchConfiguration("device"),
                }.items(),
            ),
        ]
    )
