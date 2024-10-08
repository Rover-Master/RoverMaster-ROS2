from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.substitutions import LocalSubstitution
from os import environ as env
from pathlib import Path
from datetime import datetime
import os, sys, atexit


run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
PWD = Path(env["PWD"])
RUN_VAR = PWD / "var" / run_id
IMAGES_DIR = str(RUN_VAR / "perception_images")
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
        package="spinnaker_camera",
        executable="capture",
        namespace="spinnaker",
    ),
    Node(
        package="perception",
        executable="ros2",
        namespace="perception",
        remappings=[
            ("image_in", "/spinnaker/camera_0/img"),
            ("motion", "/rover/base/velocity/set"),
            ("halt", "/rover/base/halt"),
            ("imu", "/rover/base/imu"),
        ],
    ),
    Node(
        package="playback",
        executable="recorder",
        parameters=[
            {"src": "/perception/image_out"},
            {"dst": IMAGES_DIR},
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
            "/rover/base/velocity/get",
            "/rover/base/halt",
        ],
        cwd=str(PWD / "var"),
        output="screen",
    ),
]


# Define a custom shutdown callback function to use os.system
def shutdown_callback(*args, **kwargs):
    """Function to be called on shutdown."""
    script_path = str(PWD / "scripts/runtime-bin/encode-video")
    os.system(f"{script_path} {RUN_VAR}/perception_images {RUN_VAR}/perception.mp4")
    return [LogInfo(msg="Shutdown callback executed. Video encoding started.")]


atexit.register(shutdown_callback)


def generate_launch_description():
    return LaunchDescription(nodes)
