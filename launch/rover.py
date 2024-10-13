from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from os import environ as env
from pathlib import Path
from datetime import datetime

run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
PWD = Path(env["PWD"])
RUN_VAR = PWD / "var" / run_id

nodes = [
    Node(
        package="rover",
        executable="base",
        namespace="rover",
        parameters=[
            {"vid": "x0483"},
            {"pid": "x5740"},
        ],
    ),
    Node(
        package="socket_agent",
        executable="agent",
        remappings=[
            ("vel/get", "/rover/base/velocity/get"),
            ("vel/set", "/rover/base/velocity/set"),
            ("imu", "/rover/base/imu"),
            ("halt", "/rover/base/halt"),
        ],
    ),
    Node(
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
        output="screen",
    ),
    Node(package="lidar_toolbox", executable="scan_transformer", output="screen"),
    Node(package="lidar_toolbox", executable="proximity", output="screen"),
    ExecuteProcess(
        cmd=[
            *["ros2", "bag", "record", "-o", str(RUN_VAR)],
            "/scan_transformed",
            "/rover/base/odometry",
            "/rover/base/halt",
        ],
        cwd=str(PWD / "var"),
        output="screen",
    ),
]


def generate_launch_description():
    return LaunchDescription(nodes)
