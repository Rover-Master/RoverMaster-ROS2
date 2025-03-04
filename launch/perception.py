from launch import LaunchDescription
from launch_ros.actions import Node

from pathlib import Path
HOME = Path(__file__).parent.parent

VAR = HOME / "var"

from shutil import rmtree
rmtree(VAR)
VAR.mkdir()

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
        namespace="capture",
    ),
    Node(
        package="camera_detection",
        executable="perception",
        namespace="capture",
        remappings=[
            ("img", "/capture/camera_0/img")
        ],
    ),
    Node(
        package="camera_detection",
        executable="operation",
    ),
]

def generate_launch_description():
    return LaunchDescription(nodes)
