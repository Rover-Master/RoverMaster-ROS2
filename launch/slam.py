from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node

from pathlib import Path
from math import pi as PI, degrees, radians
import os, sys, atexit, yaml, json

SELF = Path(__file__).stem
HOME = Path(__file__).resolve().parent.parent
RUN_VAR = HOME / "var"

RUN_ID = os.environ.get("RUN_ID", None)
SLAM_MODE = os.environ.get("SLAM_MODE", "MAP")

if RUN_ID is None:
    print(f"Usage: RUN_ID=<....> launch {SELF}")
    sys.exit(1)

RUN_DIR = RUN_VAR / RUN_ID

if not RUN_DIR.exists() or not RUN_DIR.is_dir():
    print(f"Error: {RUN_DIR} does not exist or is not a directory")
    sys.exit(1)

SLAM_DIR = RUN_DIR / "slam"
SLAM_DIR.mkdir(parents=False, exist_ok=True)

for RUN_BAG in (RUN_DIR / "bag").glob("*.mcap"):
    if RUN_BAG.is_file():
        break
else:
    print(f"Error: no bag files found in {RUN_DIR / 'bag'}")
    sys.exit(1)

ODOM_LIST_FILE = RUN_DIR / "odometry.list"
HEADING_OFFSET_FILE = SLAM_DIR / "HEADING_OFFSET.conf"
INITIAL_POSE_FILE = SLAM_DIR / "INITIAL_POSE.conf"
ODOM_HDG_FIX_FLAG = SLAM_DIR / "ODOM_HDG_FIX.flag"
MAP_COMPLETE_FLAG = SLAM_DIR / "MAP_COMPLETE.flag"


def load_config_float(file: Path, default: float = 0.0) -> float:
    if not file.exists() or not file.is_file():
        return default
    try:
        return float(file.read_text())
    except ValueError:
        return default


def load_config_json(file: Path, default: object) -> object:
    if not file.exists() or not file.is_file():
        return default
    try:
        return json.loads(file.read_text())
    except ValueError:
        return default


heading_offset = load_config_float(HEADING_OFFSET_FILE)
odom_direction = 0.0


for line in ODOM_LIST_FILE.open("r"):
    try:
        ts, rx, ty, rz, *_ = line.split(",")
        init_heading = radians(float(rz))
        break
    except:
        pass
else:
    init_heading = 0.0


if ODOM_HDG_FIX_FLAG.exists() and ODOM_LIST_FILE.exists():
    odom_direction = -init_heading

scan_player_params = dict(
    playback_speed=8.0,
    heading_offset=heading_offset,
    odom_direction=odom_direction,
    bag_path=str(RUN_BAG),
)

SLAM_CONFIG = HOME / "config" / "slam.yaml"
slam_config: dict = yaml.safe_load(SLAM_CONFIG.read_text())


def slam_config_extra(p: Path):
    slam_config.update(yaml.safe_load(p.read_text()))


match SLAM_MODE:
    case "MAP":
        scan_player_params["save_map"] = True
        slam_config_extra(HOME / "config" / "slam_mapping.yaml")
    case "TRJ":
        scan_player_params["save_trj"] = True
        slam_config_extra(HOME / "config" / "slam_localization.yaml")
    case _:
        print(f"Error: Invalid SLAM MODE: {SLAM_MODE}")
        sys.exit(1)


print("[INFO] Initial heading:", f"{degrees(init_heading):.2f}", "deg")
print("[INFO] Heading offset :", f"{degrees(heading_offset):.2f}", "deg")
print("[INFO] Odom direction :", f"{degrees(odom_direction):.2f}", "deg")

ScanPlayer = Node(
    package="lidar_toolbox",
    executable="scan_player",
    parameters=[scan_player_params],
    on_exit=[Shutdown()],
    cwd=str(SLAM_DIR),
)


def slam_launch_description(*nodes: Node):
    from launch.actions import (
        DeclareLaunchArgument,
        EmitEvent,
        LogInfo,
        RegisterEventHandler,
    )
    from launch.events import matches_action
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import LifecycleNode
    from launch_ros.event_handlers import OnStateTransition
    from launch_ros.events.lifecycle import ChangeState
    from lifecycle_msgs.msg import Transition

    ld = LaunchDescription(nodes)

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation/Gazebo clock",
    )

    slam_node = LifecycleNode(
        parameters=[
            {"use_lifecycle_manager": False},
            use_sim_time,
            slam_config,
        ],
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        namespace="",
        cwd=str(SLAM_DIR),
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="Activating slam_toolbox node."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
    )

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(slam_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld


def generate_launch_description():
    return slam_launch_description(
        ScanPlayer,
    )


def query_angle(initial: float, prompt: str) -> float:
    value = initial
    while True:
        try:
            disp = f"{value:.2f} rad | {degrees(value):.2f} deg"
            response = input(prompt.format(disp))
            if not response:
                break
            val, unit, *_ = response.split() + ["rad"]
            match unit.lower():
                case "deg" | "degree" | "degrees":
                    value += radians(float(val))
                case "rad" | "radian" | "radians":
                    value += float(val)
                case _:
                    raise ValueError
            value = (value + PI) % (2 * PI) - PI
        except ValueError:
            print("Invalid input:", response, file=sys.stderr)
            continue
        except EOFError:
            break
    return value


import signal


def raise_interrupt(signum, frame):
    print("\n")
    raise KeyboardInterrupt


def before_exit():
    signal.signal(signal.SIGINT, raise_interrupt)
    global heading_offset
    prev_heading_offset = heading_offset
    try:
        heading_offset = query_angle(heading_offset, "Update heading offset ({}): ")
    except KeyboardInterrupt:
        heading_offset = 0.0
    print(f"[INFO] Heading offset:", heading_offset)
    if heading_offset != 0.0:
        HEADING_OFFSET_FILE.write_text(str(heading_offset))
    else:
        HEADING_OFFSET_FILE.unlink(missing_ok=True)
    # Check if heading offset has been updated
    if heading_offset != prev_heading_offset:
        # Invalidate the MAP_COMPLETE flag
        MAP_COMPLETE_FLAG.unlink(missing_ok=True)


if SLAM_MODE == "MAP":
    atexit.register(before_exit)
