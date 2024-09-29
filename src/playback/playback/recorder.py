#!/usr/bin/env python3
import cv2, rclpy, sys
from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path


# Test codecs according to preferences
def VideoWriter(path: str, size: tuple[int, int], fps: float):
    for cc in ["avc1", "mp4v", "MJPG", "XVID", "DIVX"]:
        codec = cv2.VideoWriter_fourcc(*cc)
        video = cv2.VideoWriter(path, codec, fps, size[::-1], isColor=True)
        if video.isOpened():
            return video, cc
        else:
            video.release()
    msg = f"No codec available for video {path} of size {size} at {fps} FPS"
    raise Exception(msg)

class Recorder(Node):
    bridge = CvBridge()
    video = None
    dst: Path

    first_frame = None
    first_ts: Time | None = None

    counter = 0

    def callback(self, msg: Image):
        ts = Time.from_msg(msg.header.stamp)
        # Video writer only works with color images.
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if frame.shape[2] == 1:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        self.get_logger().info(f"REC Frame: {frame.shape} @ {ts.nanoseconds / 1e9:.4f}s")
        cv2.imwrite(str(self.dst / f"{self.counter:06d}.png"), frame)
        self.counter += 1
        return
        # Initialize the video writer
        if self.first_frame is None or self.first_ts is None:
            self.first_frame = frame
            self.first_ts = ts
            return
        elif self.video is None:
            dt = ts.nanoseconds - self.first_ts.nanoseconds
            if dt <= 1e-3:
                dt = 1 / 30
            fps = round(1e9 / dt, 2)
            size = self.first_frame.shape[:2][::-1]
            # Log the information
            self.video, cc = VideoWriter(self.dst, size, fps)
            self.get_logger().info(f"Recording at {fps} FPS, size {size}, codec {cc}")
            # Push the first frame
            self.video.write(self.first_frame)
        else:
            # Write frame normally
            self.video.write(frame)

    def __init__(self):
        super().__init__("recorder")
        # Parameter: path to the dataset
        self.declare_parameter("src", "img")
        self.declare_parameter("dst", "recording.mp4")
        src = str(self.get_parameter("src").value)
        self.dst = Path(str(self.get_parameter("dst").value))
        self.dst.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Recording from topic {src}")
        self.sub = self.create_subscription(Image, src, self.callback, 10)
        self.get_logger().info(f"Saving to file {self.dst}")


# Main
def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print("Closing video writer", file=sys.stderr)
    if node.video is not None:
        node.video.release()
    node.destroy_node()
