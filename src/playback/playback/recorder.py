#!/usr/bin/env python3
import cv2, rclpy
from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path


class Recorder(Node):
    bridge = CvBridge()
    video = None
    dst: str

    first_frame = None
    first_ts: Time | None = None

    def callback(self, msg: Image):
        ts = Time.from_msg(msg.header.stamp)
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.get_logger().info(f"Received Image: {ts}")
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
            codec = cv2.VideoWriter_fourcc(*"mjpg")
            size = self.first_frame.shape[:2][::-1]
            # Log the information
            self.get_logger().info(f"Recording at {fps} FPS and size {size}")
            self.video = cv2.VideoWriter(self.dst, codec, fps, size)
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
        self.dst = str(self.get_parameter("dst").value)
        self.get_logger().info(f"Recording from topic {src}")
        self.sub = self.create_subscription(Image, src, self.callback, 10)
        self.get_logger().info(f"Saving to file {self.dst}")

    def destory_node(self):
        if self.video is not None:
            self.video.release()
        super().destory_node()


# Main
def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
