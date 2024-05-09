# Python & rclpy imports
import rclpy, json, os
from rclpy.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger as Logger
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict
from geometry_msgs.msg import Twist

# Project local imports
from .socket import SocketClient


class SocketAgent(Node):

    def __init__(self, socket: SocketClient):
        super().__init__("SocketAgent")
        self.socket = socket
        socket.logger = self.get_logger()
        self.timer = self.create_timer(0.01, self.recv)

    # Dict of all ROS2 topic publishers and subscribers
    publishers = {}
    subscribers = {}

    def subscribe(self, topic, type, qos=10):
        def callback(msg):
            try:
                self.socket.send_all(
                    f"{topic} {json.dumps(message_to_ordereddict(msg))}"
                )
            except Exception as e:
                self.get_logger().error(f"Error serializing message: {e}")

        self.subscribers[topic] = self.create_subscription(type, topic, callback, qos)

    def publish(self, topic, type, qos=10):
        publisher = self.create_publisher(type, topic, qos)

        def pub(message: dict | str | int | float | bool):
            try:
                msg = publisher.msg_type()
                if isinstance(message, dict):
                    set_message_fields(msg, message)
                else:
                    msg.data = message
                publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error publishing message: {e}")

        self.publishers[topic] = pub

    def recv(self):
        while rclpy.ok():
            message = self.socket.recv_line()
            if message is None:
                break
            # Parse the message in the format of "<topic> <message>"
            topic, message = str(message).split(" ", 1)
            # Publish the message to the topic
            if topic in self.publishers:
                try:
                    self.publishers[topic](json.loads(message))
                except Exception as e:
                    self.get_logger().error(f"Error deserializing message: {e}")
            else:
                self.get_logger().warn(f"Publishing to unknown topic {topic}")


def launch():
    rclpy.init()
    agent = SocketAgent(SocketClient("/tmp/ros-agent.sock"))
    agent.subscribe("vel/get", Twist)
    agent.publish("vel/set", Twist)
    agent.subscribe("platform/pos/get", Twist)
    agent.publish("platform/pos/set", Twist)
    agent.subscribe("imu/acc", Twist)
    agent.subscribe("imu/att", Twist)
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    launch()
