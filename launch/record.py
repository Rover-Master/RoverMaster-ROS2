from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node
from os import environ as env
from pathlib import Path
from datetime import datetime
import os, sys, atexit, subprocess

RUN_ID = datetime.now().strftime("%Y%m%d_%H%M%S") + "_record"
HOME = Path(__file__).resolve().parent.parent
RUN_DIR = HOME / "var" / RUN_ID
assert not RUN_DIR.exists()
REC = "recording"

RUN_DIR.mkdir(parents=True, exist_ok=False)


class Perception(Node):
    def __init__(self, executable: str, **kwargs):
        super().__init__(
            **kwargs,
            package="perception",
            executable=executable,
            namespace="perception",
            cwd=str(RUN_DIR),
            ros_arguments=["--log-level", "info"],
        )


# ==================== PERCEPTION NODES ====================
perception = [
    # Perception(
    #     executable="perception",
    #     remappings=[("image", "/spinnaker/camera_0/img")],
    # ),
    # Perception(
    #     executable="correlator",
    # ),
    # Perception(
    #     executable="navigation",
    #     remappings=[
    #         ("halt", "/rover/base/halt"),
    #         ("odometry", "/rover/base/odometry"),
    #         ("motion", "/rover/base/velocity/set"),
    #     ],
    # ),
    Perception(
        executable="recorder",
        remappings=[("image", "/spinnaker/camera_0/img")],
        parameters=[{"dst": REC}],
    ),
]
# ==================== SUPPORTIVE NODES ====================
Rover = Node(
    package="rover",
    executable="base",
    namespace="rover",
    parameters=[
        {"vid": "x0483"},
        {"pid": "x5740"},
    ],
)

Camera = Node(
    package="spinnaker_camera",
    executable="capture",
    namespace="spinnaker",
)

LiDAR = Node(
    package="sllidar_ros2",
    executable="sllidar_node",
    parameters=[
        {
            "channel_type": "serial",
            "serial_port": "/dev/rplidar",
            "serial_baudrate": 115200,
            "frame_id": "laser",
            "inverted": False,
            "angle_compensate": True,
        }
    ],
    ros_arguments=["--log-level", "warn"],
)

SocketAgent = Node(
    package="socket_agent",
    executable="agent",
    remappings=[
        ("vel/get", "/rover/base/velocity/get"),
        ("vel/set", "/rover/base/velocity/set"),
        ("imu", "/rover/base/imu"),
        ("halt", "/rover/base/halt"),
    ],
)

BagRecorder = ExecuteProcess(
    cmd=[
        *["ros2", "bag", "record", "-o", str(RUN_DIR / "bag")],
        "/scan_transformed",
        "/rover/base/imu",
        "/rover/base/odometry",
        "/rover/base/halt",
    ],
)


def generate_launch_description():
    return LaunchDescription(
        [
            # ======== CORE NODES ========
            Rover,
            Camera,
            *perception,
            # ===== SUPPORTIVE NODES =====
            LiDAR,
            SocketAgent,
            BagRecorder,
        ]
    )
