#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import signal
import sys

node: Node
# Global variable to store the latest image
latest_img_msg: Image | None = None
latest_jpg: bytes | None = None
should_exit = False
bridge = CvBridge()

last_updated_img_msg: Image | None = None
def update_latest_jpg():
    global latest_img_msg, latest_jpg, last_updated_img_msg
    msg = latest_img_msg
    if msg is None or (last_updated_img_msg is msg):
        return latest_jpg
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # resize
        resized_image = cv2.resize(cv_image, None, fx=0.15, fy=0.15)
        img = resized_image
        success, jpg = cv2.imencode('.jpg', img)
        if success:
            latest_jpg = jpg.tobytes()
        last_updated_img_msg = msg
    except Exception as e:
        node.get_logger().error(f'Error processing image: {str(e)}')

class ImageHandler(BaseHTTPRequestHandler):
    def _set_cors_headers(self):
        if "Origin" in self.headers:
            self.send_header("Access-Control-Allow-Origin", self.headers["Origin"])
        else:
            self.send_header("Access-Control-Allow-Origin", "*")

        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type, Cache-Control')
    def do_OPTIONS(self):
        self.send_response(200)
        self._set_cors_headers()
        self.end_headers()
    
    def log_message(self, format, *args):
        pass
    
    def do_GET(self):
        try:
            if self.path.startswith('/image'):
                self.send_response(200)
                self._set_cors_headers()
                self.send_header('Content-Type', 'image/jpeg')
                
                # Generate fresh image data
                update_latest_jpg()
                if not latest_jpg:
                    raise Exception("No image available")
                
                self.send_header('Content-Length', str(len(latest_jpg)))
                self.end_headers()
                self.wfile.write(latest_jpg)
                
            else:  # HTML page
                self.send_response(200)
                self._set_cors_headers()
                self.send_header('Content-Type', 'text/html')
                self.end_headers()
                self.wfile.write(html)
                
        except Exception as e:
            self.send_response(500)
            self._set_cors_headers()
            self.end_headers()
            self.wfile.write(f"Error: {str(e)}".encode())

class ImageSubscriberNode(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/capture/camera_0/img',  
            self.image_callback,
            10
        )
        self.server_thread = None
        self.get_logger().info('Image subscriber node started')
        
        # Start the HTTP server in a separate thread
        self.start_http_server()
        
    def start_http_server(self, address="0.0.0.0", port=8085):
        self.server_thread = threading.Thread(
            target=self.launch_server,
            args=(address, port),
            daemon=True
        )
        self.server_thread.start()
        self.get_logger().info(f"HTTP server started on port {port}")
        
    def launch_server(self, address, port):
        server_address = (address, port)
        self.httpd = HTTPServer(server_address, ImageHandler)
        self.httpd.serve_forever()
        
    def image_callback(self, msg: Image):
        global latest_img_msg
        latest_img_msg = msg

def main(args=None):
    rclpy.init(args=args)    
    # Create the node
    global node
    node = ImageSubscriberNode()
    try:
        # Spin the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Clean up
    node.destroy_node()

if __name__ == '__main__':
    main()

html = """
<html>
    <head>
        <title>ROS2 Camera Stream</title>
        <style>
            body { font-family: Arial, sans-serif; text-align: center; }
            img { max-width: 100%; }
        </style>
    </head>
    <body>
        <h1>ROS2 Camera Stream</h1>
        <img id="stream" alt="Camera Stream" style="width: min(960px, 80vw)">
        <p>Image automatically refreshes every second</p>
        <script>
            const imgElement = document.getElementById("stream");
            function blobToDataURL(blob) {
                return new Promise((res) => {
                    const a = new FileReader();
                    a.onload = u => res(u.target.result)
                    a.readAsDataURL(blob);
                })
            }
            function delay(duration) {
                return new Promise(res => setTimeout(res, duration))
            }
            async function grabNextImage() {
                return fetch("/image.jpg")
                    .then(async res => {
                        res
                            .blob()
                            .then(blobToDataURL)
                            .then(url => imgElement.src = url)
                        await delay(33);
                        return grabNextImage();
                    })
            }
            grabNextImage()
        </script>
    </body>
</html>
""".encode()