import numpy as np
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path
from threading import Thread
import cv2

# Set up directories
CWD = Path(os.getcwd())
VAR = CWD / "var"
VAR.mkdir(parents=True, exist_ok=True)
ASSETS = CWD / "assets"
ASSETS.mkdir(parents=True, exist_ok=True)


class Perception(Node):
    frame: np.ndarray | None = None
    flag_term: bool = False

    def __init__(self):
        super().__init__("detection")
        self.br = CvBridge()
        self.img_sub = self.create_subscription(Image, "img", self.onFrame, 10)
        self.det_pub = self.create_publisher(String, "/detection", 10)

    def onFrame(self, msg):
        frame = self.br.imgmsg_to_cv2(msg)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self.frame = frame


def perception(node: Perception):
    from time import sleep
    from json import dumps
    from ultralytics import YOLO
    from ultralytics.engine.results import Results

    model = YOLO("/data/yolo11s-egg.pt")
    # Load the YOLO model
    prev_frame: np.ndarray | None = None
    while not node.flag_term:
        next_frame = node.frame
        if next_frame is None or (next_frame is prev_frame):
            sleep(0.01)
            continue

        cv2.imwrite(VAR / "frame.jpg", next_frame)
        frame = cv2.resize(next_frame, None, fx=0.2, fy=0.2)
        h, w, _ = frame.shape
        # Perform detection on the frame
        results: list[Results] = model(frame, conf=0.5)
        detections = []
        # Print object details
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                id = int(box.cls[0])
                # Bounding box coordinates
                x1, y1, x2, y2, *_ = map(float, box.xyxy[0])
                confidence = float(box.conf[0])  # Confidence score
                cx = (x1 + x2) / (2 * w)
                cy = (y1 + y2) / (2 * h)
                detections.append([id, cx, cy, confidence])

        msg = String()
        msg.data = dumps(detections)
        node.det_pub.publish(msg)


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
