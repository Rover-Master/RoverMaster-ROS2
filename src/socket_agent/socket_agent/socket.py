import socket, os, errno, time, sys


class DefaultLogger:
    def info(self, *msg: str):
        print("[INFO] ", *msg)

    def warn(self, *msg: str):
        print("[WARN] ", *msg)

    def error(self, *msg: str):
        print("[ERROR]", *msg, file=sys.stderr)


class SocketClient:
    """
    A simple Unix socket client that robustly connects to a Unix socket server.
    It automatically reconnects if the socket is not available or closed.
    """

    def __init__(self, path: str, logger=DefaultLogger()):
        self.path = path
        self.client = None
        self.logger = logger

    report_missing_socket = True

    def check_socket(self):
        if self.client is not None:
            return True
        # Set the path for the Unix socket
        if not os.path.exists(self.path):
            if self.report_missing_socket:
                self.logger.info(f"Waiting for socket unix:{self.path}")
                self.report_missing_socket = False
            return False
        self.report_missing_socket = True
        # Create the Unix socket server
        self.client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        # Set the socket to non-blocking mode
        self.client.setblocking(False)
        # Bind the socket to the path
        self.logger.info(f"Connecting to unix:{self.path}")
        try:
            self.client.connect(self.path)
        except socket.error as e:
            match e.args[0]:
                case errno.ECONNREFUSED:
                    self.logger.error(f"Connection refused to unix:{self.path}")
                case errno.ENOENT:
                    self.logger.error(f"Socket unix:{self.path} not found")
                case _:
                    self.logger.error(f"Failed to connect to unix:{self.path}", e)
            self.client.close()
            self.client = None
            return False
        self.logger.info(f"Connected to unix:{self.path}")
        return True

    def reset_socket(self):
        if self.client is None:
            return
        self.client.close()
        self.client = None
        self.logger.warn(f"Disconnected from unix:{self.path}")

    def send_all(self, msg: str):
        if not self.check_socket():
            return None
        try:
            self.client.sendall(msg.encode())
        except Exception as e:
            self.logger.error(f"Failed to send message: ${e}")
            self.reset_socket()

    line_buffer = b""

    def recv_line(self):
        """
        Read socket byte by byte until line feed is encountered, non-blocking.
        Returns None if no data is available.
        """
        if not self.check_socket():
            return None
        try:
            while True:
                data = self.client.recv(1)
                if not data:
                    return None
                if data == b"\n":
                    break
                self.line_buffer += data
            line = self.line_buffer.decode()
            self.line_buffer = b""
            return line
        except socket.error as e:
            err = e.args[0]
            if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                return None
            else:
                raise e
        except Exception:
            self.reset_socket()


# Example usage
if __name__ == "__main__":
    client = SocketClient("/tmp/ros-web-agent.sock")
    while True:
        client.send_all("Hello, World!\n")
        while True:
            line = client.recv_line()
            if line is None:
                break
            client.logger.info(f"Received: {line}")
        time.sleep(0.1)
