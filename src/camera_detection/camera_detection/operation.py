import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time
import math
from threading import Timer

class Operation(Node):

    override: bool = False

    def __init__(self):
        self.lastupdate = time.time()
        self.stop_timer = None
        super().__init__("OmnibotOperation")
        self.operation_override_publisher = self.create_publisher(
            Bool, "/rover/operation_override", 10
        )
        self.velocity_publisher = self.create_publisher(
            Twist, "/rover/base/velocity/set", 10
        )
        self.override_timer = self.create_timer(0.1, self.check_override)

    def check_override(self):
        if (time.time() - self.lastupdate >= 1.0) and (self.override == True):
            self.override = False
            msg = Bool()
            msg.data = self.override
            self.operation_override_publisher.publish(msg)
            self.get_logger().info("Override reset to False after 1 second")

    def move_forward_after_rotation(self, x_value):
        twist_msg = Twist()
        twist_msg.linear.x = 0.3 * x_value
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

        # Estimate travel time: assume 0.3 m/s
        travel_time = min(abs(x_value) / 0.3, 5.0)  # cap at 5s
        self.get_logger().info(f"Moving forward for {travel_time:.2f}s toward x={x_value}")

        # Schedule stop
        self.stop_timer = Timer(travel_time, self.stop_movement)
        self.stop_timer.start()

    def stop_movement(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info("Stopped at target.")

    
    def convert_and_publish(self, command):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        
        if "Controller:" in command:
            x_value = float(command.split("x:")[1].split()[0])
            y_value = float(command.split("y:")[1].split()[0])
            twist_msg.linear.x = y_value * (- 0.5)
            twist_msg.angular.z = x_value * (- 0.5)

            self.velocity_publisher.publish(twist_msg)
            self.get_logger().info(f"Publishing velocity command: {twist_msg}")

        elif "Button Click" in command or "Key Pressed" in command: 
            x_value = float(command.split("x:")[1].split()[0])
            y_value = float(command.split("y:")[1].split()[0])
            # Mapping command strings to Twist message fields
            if y_value == -1:
                twist_msg.linear.x = 0.2
            elif y_value == 1:
                twist_msg.linear.x = -0.2
            elif x_value == -1:
                twist_msg.angular.z = 0.2
            elif x_value == 1:
                twist_msg.angular.z = -0.2
            else:
                self.get_logger().warn(f"Unrecognized command: {command}")
                return
            
            # Publish the Twist message
            self.velocity_publisher.publish(twist_msg)
            self.get_logger().info(f"Publishing velocity command: {twist_msg}")

            
        elif "Target location designated" in command:
            try:
                x_value = float(command.split("x:")[1].split(",")[0].strip())
                y_value = float(command.split("y:")[1].split(")")[0].strip())

                # Clear previous timer if any
                if self.stop_timer:
                    self.stop_timer.cancel()

                # Step 1: Rotate first (based on Y)
                if abs(y_value) > 0.05:
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = -0.5 * y_value  # rotate direction
                    self.cmd_vel_publisher.publish(twist_msg)

                    # Estimate rotation time: assume 0.5 rad/s rotation speed
                    rotation_time = min(abs(y_value) * 1.5, 3.0)  # cap at 3s
                    self.get_logger().info(f"Rotating for {rotation_time:.2f}s toward y={y_value}")

                    # After rotation, move forward
                    self.stop_timer = Timer(rotation_time, self.move_forward_after_rotation, [x_value])
                    self.stop_timer.start()
                    return

                # If no rotation needed, go straight immediately
                if abs(x_value) > 0.05:
                    self.move_forward_after_rotation(x_value)

            except Exception as e:
                self.get_logger().warn(f"Failed to parse LIDAR target command: {command} | Error: {str(e)}")
        
def main():
    from json import dumps, loads
    from time import sleep
    from .socket import SocketClient

    global node
    # Initialize ROS 2
    rclpy.init()
    node = Operation()
    socket = SocketClient("/tmp/omni-control.sock")
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0)
            line = socket.recv_line()
            if line is None:
                sleep(0.01)
                continue
            try:
                node.override = True
                node.lastupdate = time.time()
                msg = Bool()
                msg.data = node.override
                node.operation_override_publisher.publish(msg)
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
