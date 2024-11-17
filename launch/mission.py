from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node
from os import environ as env
from pathlib import Path
from datetime import datetime
import os, sys, atexit, subprocess

RUN_ID = datetime.now().strftime("%Y%m%d_%H%M%S")
HOME = Path(__file__).resolve().parent.parent
RUN_DIR = HOME / "var" / RUN_ID
assert not RUN_DIR.exists()
REC = "recording"
RND = "rendering"
VIDEO_PATH = str(RUN_DIR) + ".mp4"

RUN_DIR.mkdir(parents=True, exist_ok=False)
LATEST = HOME / "var" / "latest"
if LATEST.exists():
    LATEST.unlink()
os.symlink(RUN_ID, LATEST, target_is_directory=False)


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
    Perception(
        executable="perception",
        remappings=[("image", "/spinnaker/camera_0/img")],
    ),
    Perception(
        executable="correlator",
    ),
    Perception(
        executable="navigation",
        remappings=[
            ("halt", "/rover/base/halt"),
            ("odometry", "/rover/base/odometry"),
            ("motion", "/rover/base/velocity/set"),
        ],
    ),
    Perception(
        executable="recorder",
        remappings=[("image", "/spinnaker/camera_0/img")],
        parameters=[{"dst": REC}],
    ),
]
# perception = [
#     Perception(
#         "node",
#         remappings=[
#             ("image", "/spinnaker/camera_0/img"),
#             ("halt", "/rover/base/halt"),
#             ("odometry", "/rover/base/odometry"),
#             ("motion", "/rover/base/velocity/set"),
#         ],
#     )
# ]
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

ScanTransformer = Node(package="lidar_toolbox", executable="scan_transformer")

ProximitySwitch = Node(package="lidar_toolbox", executable="proximity")

BagRecorder = ExecuteProcess(
    cmd=[
        *["ros2", "bag", "record", "-o", str(RUN_DIR / "bag")],
        "/scan_transformed",
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
            ScanTransformer,
            ProximitySwitch,
            BagRecorder,
        ]
    )


def confirm(question: str, auto_rej: bool = False) -> bool:
    while True:
        dfl = "(n) " if auto_rej else ""
        match input(f"{question} [Y/n] {dfl}").lower():
            case "y":
                return True
            case "n":
                return False
            case "":
                if auto_rej:
                    return False
        print(f"Please respond with 'Y' or 'n'")


def shutdown_callback(*args, **kwargs):
    try:
        sockets = list(RUN_DIR.glob("*.socket"))
        if len(sockets):
            print("Warning: found dangling sockets")
            for socket in sockets:
                socket.unlink()
                print(f" - Removed: {socket}")
        # Try to render images
        print("Rendering images ...")
        fig_cmd = [
            *"python3 -m ros2.utils.look_around".split(),
            str(RUN_DIR),
            *["--src", REC],
        ]
        vdo_cmd = [
            *"python3 -m ros2.utils.render".split(),
            str(RUN_DIR),
            *["--src", REC],
        ]
        DIR = HOME / "src" / "perception"
        print("=" * 60)
        print("(", "cd", DIR, "&&", " ".join(fig_cmd), "&&", " ".join(vdo_cmd), ")")
        print("=" * 60)
        sh = RUN_DIR / "render.sh"
        with open(sh, "w") as f:
            f.write(f"cd {DIR.absolute()}\n")
            f.write(" ".join(fig_cmd) + "\n")
            f.write(" ".join(vdo_cmd) + "\n")
        sh.chmod(0o755)
        if confirm("Render now?"):
            subprocess.Popen(
                args=fig_cmd,
                cwd=str(DIR),
            ).wait()
            subprocess.Popen(
                args=vdo_cmd,
                cwd=str(DIR),
            ).wait()
        elif confirm("Delete this run?", auto_rej=True):
            from shutil import rmtree

            LATEST.unlink()
            rmtree(RUN_DIR)
        else:
            print(f"Run {sh} to render later")
    except OSError:
        pass
    except KeyboardInterrupt:
        pass


atexit.register(shutdown_callback)
