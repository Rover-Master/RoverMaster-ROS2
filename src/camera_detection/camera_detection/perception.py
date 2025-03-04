import numpy as np
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from pathlib import Path
from threading import Thread, Lock
import cv2

# Set up directories
CWD = Path(os.getcwd())
VAR = CWD / "var"
VAR.mkdir(parents=True, exist_ok=True)
ASSETS = CWD / "assets"
ASSETS.mkdir(parents=True, exist_ok=True)


class Perception(Node):
    lock: Lock
    frame: np.ndarray | None = None
    flag_term: bool = False

    def __init__(self):
        super().__init__("camera_detection_node")
        self.lock = Lock()
        self.br = CvBridge()
        self.frame_index = 0

        self.subscription = self.create_subscription(
            Image, "img", self.onCameraFrame, 10
        )

        self.velocity_publisher = self.create_publisher(
            Twist, "/rover/base/velocity/set", 10
        )

    def onCameraFrame(self, msg):
        frame = self.br.imgmsg_to_cv2(msg)
        with self.lock:
            self.frame = frame
            # self.save_frame(frame)


    def save_frame(self, frame: np.ndarray):
        filename = VAR / f"frame_{self.frame_index:04d}.jpg"
        self.frame_index += 1  # Increment the frame counter
        frame = cv2.resize(frame, None, fx=0.2, fy=0.2)
        cv2.imwrite(str(filename), frame)  # Save the frame as an image file
        return filename

def perception(node: Perception):
    from json import dumps
    from ultralytics import YOLO
    from ultralytics.engine.results import Results
    from .socket import SocketClient

    model = YOLO(ASSETS / "yolo11n.pt")
    socket = SocketClient("/tmp/omni-control.sock")

    if socket.check_socket():
        node.get_logger().info("Socket is connected successfully.")
    else:
        node.get_logger().warn("Failed to connect to the socket.")

    # Load the YOLO model
    while not node.flag_term:
        frame: np.ndarray | None = None
        while frame is None:
            with node.lock:
                frame = node.frame
                node.frame = None
        frame_id = node.save_frame(frame)
        # Process the frame with YOLO
        results: list[Results] = model(frame)  # Perform detection on the frame
        boxes = [frame_id.name]
        # Print object details
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                label = str(result.names[box.cls[0].item()])  # Object label
                # Bounding box coordinates
                x1, y1, x2, y2, *_ = map(float, box.xyxy[0])
                confidence = float(box.conf[0])  # Confidence score
                boxes.append([label, x1, y1, x2, y2, confidence])

        socket.send_all(dumps(boxes) + "\n")
        while True:
            line = socket.recv_line()
            if line is None:
                break


def main():
    global node
    # Initialize ROS 2
    rclpy.init()
    node = Perception()

    thread = Thread(target=perception, args=(node,), daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.flag_term = True
        thread.join()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
