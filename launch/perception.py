from launch import LaunchDescription
from launch_ros.actions import Node

from pathlib import Path
HOME = Path(__file__).parent.parent

VAR = HOME / "var"

# from shutil import rmtreecd
# rmtree(VAR)

VAR.mkdir(exist_ok=True)

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
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="rover",
        name="camera",
    ),
    Node(
        package="camera_detection",
        executable="perception",
        namespace="capture",
        remappings=[
            ("img", "/rover/camera/color/image_raw")
        ],
    ),
    Node(
        package="camera_detection",
        executable="operation",
    ),
    Node(
        package="camera_detection",
        executable="server",
        remappings=[
            ("img", "/rover/camera/color/image_raw")
        ],
    ),
]

def generate_launch_description():
    return LaunchDescription(nodes)
