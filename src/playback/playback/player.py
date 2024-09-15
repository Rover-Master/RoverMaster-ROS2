#!/usr/bin/env python3
import cv2, rclpy
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rclpy.publisher import Publisher
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
from pathlib import Path
import time

bridge = CvBridge()


def header(ns: int) -> Header:
    return Header(stamp=Time(nanosec=ns))


# Message publishers
def img_pub_task(path: Path, ns: str, pub: Publisher):
    t = int(ns)

    def task(ns: int, stat: dict[str, int] | None = None):
        img = cv2.imread(str(path))
        if img is None:
            raise FileNotFoundError(f"Input Image not found: {path}")

        match (img.shape[2]):
            case 1:
                encoding = "mono8"
            case 3:
                encoding = "bgr8"
            case 4:
                encoding = "bgra8"
            case _:
                raise ValueError(f"Unsupported image format: {img.shape}")
        msg = bridge.cv2_to_imgmsg(img, encoding, header=header(ns))
        pub.publish(msg)
        # Append to statistics
        if stat is None:
            return
        if not pub.topic in stat:
            stat[pub.topic] = 0
        stat[pub.topic] += 1

    return t, task


# IMU publishers
def imu_pub_task(data: list[str], ns: str, pub: Publisher):
    t = int(ns)
    d = [float(x) for x in data]

    def task(ns: int, stat: dict[str, int] | None = None):
        msg = Imu(header=header(ns))
        msg.angular_velocity.x = d[0]
        msg.angular_velocity.y = d[1]
        msg.angular_velocity.z = d[2]
        msg.linear_acceleration.x = d[3]
        msg.linear_acceleration.y = d[4]
        msg.linear_acceleration.z = d[5]
        pub.publish(msg)
        # Append to statistics
        if stat is None:
            return
        if not pub.topic in stat:
            stat[pub.topic] = 0
        stat[pub.topic] += 1

    return t, task


# CSV to iterator loader
def load_csv(path: Path):
    with open(path, "r") as f:
        for line in f:
            if line.startswith("#"):
                continue
            if len(line.strip()) == 0:
                continue
            yield line.strip().split(",")


# Main
def main():
    rclpy.init()
    node = rclpy.create_node("player")
    # Parameter: path to the dataset
    node.declare_parameter("src", "img")
    node.declare_parameter("dst", "")
    dataset = Path(str(node.get_parameter("src").value))
    node.get_logger().info(f"Using Dataset: {dataset}")
    # Prepare CSV filles
    cam0_list = load_csv(dataset / "cam0" / "data.csv")
    cam0_dir = dataset / "cam0" / "data"
    cam1_list = load_csv(dataset / "cam1" / "data.csv")
    cam1_dir = dataset / "cam0" / "data"
    imu0_list = load_csv(dataset / "imu0" / "data.csv")
    # Prepare Publishers
    dst = str(node.get_parameter("dst").value)
    cam0_pub = node.create_publisher(Image, f"{dst}/cam0", 10)
    cam1_pub = node.create_publisher(Image, f"{dst}/cam1", 10)
    imu0_pub = node.create_publisher(Imu, f"{dst}/imu", 10)
    # Generate publish tasks
    tasks: list[tuple[int, function]] = [
        *[img_pub_task(cam0_dir / f, t, cam0_pub) for t, f in cam0_list],
        *[img_pub_task(cam1_dir / f, t, cam1_pub) for t, f in cam1_list],
        *[imu_pub_task(d, t, imu0_pub) for t, *d in imu0_list],
    ]
    tasks.sort(key=lambda t: t[0])
    if len(tasks) == 0:
        raise FileNotFoundError("No tasks found")
    # Align the clock
    rec_origin = tasks[0][0]
    sys_origin = time.time_ns()
    tasks = [(t - rec_origin, f) for t, f in tasks]
    # Playback all events
    stat = {}
    while len(tasks) > 0 and rclpy.ok():
        ns, task = tasks.pop(0)
        while sys_origin + ns > time.time_ns():
            rclpy.spin_once(node, timeout_sec=0)
        task(ns, stat)
        # Gather statistics
        if len(stat) > 0:
            node.get_logger().info(", ".join(f"{k}: {v:8d}" for k, v in stat.items()))
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
