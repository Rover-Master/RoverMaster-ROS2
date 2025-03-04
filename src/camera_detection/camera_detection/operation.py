import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Operation(Node):

    def __init__(self):
        super().__init__("OmnibotOperation")
        self.velocity_publisher = self.create_publisher(
            Twist, "/rover/base/velocity/set", 10
        )

    def convert_and_publish(self, command):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        # Mapping command strings to Twist message fields
        if command == "Move Forward":
            twist_msg.linear.x = 0.2
        elif command == "Move Backward":
            twist_msg.linear.x = -0.2
        elif command == "Turn Left":
            twist_msg.angular.z = 0.2
        elif command == "Turn Right":
            twist_msg.angular.z = -0.2
        else:
            self.get_logger().warn(f"Unrecognized command: {command}")
            return
        
        # Publish the Twist message
        self.velocity_publisher.publish(twist_msg)
        self.get_logger().info(f"Publishing velocity command: {command}")


def main():
    from json import dumps, loads
    from .socket import SocketClient

    global node
    # Initialize ROS 2
    rclpy.init()
    node = Operation()
    socket = SocketClient("/tmp/omni-control.sock")
    try:
        while True:
            line = socket.recv_line()
            try:
                data = loads(line)
                node.get_logger().info(dumps(data))
                node.convert_and_publish(data)
            except:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
