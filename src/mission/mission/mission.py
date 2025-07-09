import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .util import attitude_from_quaternion

from .action import Action

class Misssion(Action.Hub, Node):
    frame: np.ndarray | None = None
    flag_term: bool = False

    def __init__(self):
        super().__init__("mission_node")
        self.velocity_pub = self.create_publisher(
            Twist, "/rover/base/velocity/set", 10
        )


    def motion(self, x: float, y: float, r: float):
        twist = Twist()
        twist.linear.x = float(x)
        twist.linear.y = float(y)
        twist.angular.z = float(r)
        return self.velocity_pub.publish(twist)

    @Action.action
    def move(self, vx: float, vy: float, vr: float, duration: float):
        yield
        deadline = time.time() + duration
        self.get_logger().info(f"move {vx, vy, vr} until {deadline}s")
        while time.time() < deadline:
            yield self.motion(vx, vy, vr)
        yield self.motion(0, 0, 0)

    @Action.action
    def combo(self):
        yield self.move(0.0, 0.0, 0.5, 7)
        yield self.move(-0.5, 0.0, 0.0, 6)
        yield self.move(+0.5, 0.0, 0.0, 5)


def main(args=None):
    rclpy.init(args=args)
    robot = Misssion()
    robot.combo()
    while rclpy.ok():
        rclpy.spin_once(robot, timeout_sec=0.01)
        robot.wait_action()
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# 1. Set vel to [+0.5, 0.0, 0.0], keep 5s
# 1. Set vel to [-0.5, 0.0, 0.0], keep 6s
# 1. Set vel to [ 0.0, 0.0, 0.5], keep 7s


