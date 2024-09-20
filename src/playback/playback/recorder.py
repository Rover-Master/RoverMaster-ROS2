#!/usr/bin/env python3
import cv2, rclpy, sys
from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# Test codecs according to preferences
for cc in ["avc1", "mp4v", "MJPG", "XVID", "DIVX"]:
    codec = cv2.VideoWriter_fourcc(*cc)
    v_test = cv2.VideoWriter("/tmp/test.mp4", codec, 30, (640, 480), isColor=True)
    success = v_test.isOpened()
    v_test.release()
    if success:
        break
    else:
        cc = None
        codec = None


class Recorder(Node):
    bridge = CvBridge()
    video = None
    dst: str

    first_frame = None
    first_ts: Time | None = None

    def callback(self, msg: Image):
        ts = Time.from_msg(msg.header.stamp)
        # Video writer only works with color images.
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if frame.shape[2] == 1:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        self.get_logger().info(f"Frame: {frame.shape} - {ts.nanoseconds / 1e9:.4f}s")
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
            self.get_logger().info(f"Recording at {fps} FPS, size {size}, codec {cc}")
            self.video = cv2.VideoWriter(self.dst, codec, fps, size, isColor=True)
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
