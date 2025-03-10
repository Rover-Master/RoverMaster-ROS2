import rclpy, cv2, numpy as np
from rclpy.node import Node
from http.server import HTTPServer, BaseHTTPRequestHandler

def img2jpg(img: np.ndarray) -> bytes:
    # Convert numpy mat to JPG byte array in memeory
    ...

class MyHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        # check if the requested URL is valid
        if ...:
            self.send_response(404)
            self.end_headers()
            return
        # Convert latest frame to JPEG byte array
        # Normal response - send image
        self.send_response(200)
        self.send_header("Content-type", "image/jpeg")
        self.end_headers()
        print(self.wfile)
        self.wfile.write("<html><head><title>Title goes here.</title></head>")
        self.wfile.write("<body><p>This is a test.</p>")
        # If someone went to "http://something.somewhere.net/foo/bar/",
        # then s.path equals "/foo/bar/".
        self.wfile.write("<p>You accessed path: %s</p>" % self.path)
        self.wfile.write("</body></html>")
        self.wfile.close()

image: np.ndarray | None = None

def launch_server(address: str = "", port: int = 8081):
    httpd = HTTPServer((address, port), BaseHTTPRequestHandler)
    httpd.serve_forever()


def start_node():
    # Write to "image"
    ...

def main():
    from threading import Thread
    server_thread = Thread(launch_server, daemon=True)
    node_thread = Thread(start_node, daemon=True)
    server_thread.run()
    node_thread.run()
