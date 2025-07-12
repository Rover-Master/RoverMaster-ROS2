from math import sqrt
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import time
from .lib.math import ang_diff, clamp, sign
from .lib.util import attitude_from_quaternion
from .lib.action import Action
from .driver import Motor, Driver, Expect
from contextlib import contextmanager


class Mission(Action.Hub, Node):
    frame: np.ndarray | None = None
    flag_term: bool = False

    def __init__(self):
        super().__init__("mission")
        self.vel_pub = self.create_publisher(Twist, "/rover/base/velocity/set", 10)
        self.det_sub = self.create_subscription(
            String, "/detection", self.onDetection, 10
        )
        self.imu_sub = self.create_subscription(Imu, "/rover/base/imu", self.onImu, 10)
        self.driver = Driver(baudrate=115200)
        self.get_logger().info(f"Driver Info: {self.driver.getInfo()}")
        self.m0 = Motor(self.driver, id=0, invert=1, scale=6.0, init_pos=0.0)
        self.m1 = Motor(self.driver, id=1, invert=0, scale=1.0, init_pos=0.0)
        self.m2 = Motor(self.driver, id=2, invert=1, scale=0.92, init_pos=0.0)

    @contextmanager
    def enable(self):
        while self.heading is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        with self.driver.enable(), self.m0.enable(), self.m1.enable(), self.m2.enable():
            self.m0.setPosition(0.5)
            Expect.wait(self.m0.move(1.0, speed=0.6))
            Expect.wait(self.m0.move(0.0, speed=0.6))
            # Home M0
            yield

    initial_hdg: float = 0.0
    # Range [-180, +180]
    heading: float | None = None

    def onImu(self, msg: Imu):
        _, _, heading = attitude_from_quaternion(msg.orientation)
        if self.heading is None:
            self.initial_hdg = heading
        self.heading = ang_diff(self.initial_hdg, heading)

    det_eggs: list[tuple[float, float]] = []
    det_bins: list[tuple[float, float]] = []

    def onDetection(self, msg: String):
        from json import loads

        try:
            data = loads(msg.data)
            self.det_eggs = [(x, y) for l, x, y, _ in data if l == 0]
            self.det_bins = [(x, y) for l, x, y, _ in data if l == 1]
            self.get_logger().info(f"detections: {self.det_eggs}")
        except Exception as e:
            self.get_logger().warn(f"invalid detection data: {e}")

    def motion(self, x: float, y: float, r: float):
        twist = Twist()
        twist.linear.x = float(x)
        twist.linear.y = float(y)
        twist.angular.z = float(r)
        self.get_logger().info(f"motion: vx={x:.2f}, vy={y:.2f}, vr={r:.2f}")
        return self.vel_pub.publish(twist)

    def halt(self):
        self.motion(0, 0, 0)

    @Action.action
    def delay(self, duration: float):
        deadline = time.time() + duration
        while time.time() < deadline:
            yield

    @Action.action
    def move(self, vx: float, vy: float, vr: float, duration: float):
        deadline = time.time() + duration
        while time.time() < deadline:
            yield self.motion(vx, vy, vr)
        yield self.motion(0, 0, 0)

    @Action.action
    def load(self, bin: int, end_pos: float = 0.0):
        # Align bin (m1) with m0:
        yield from self.m1.move(float(bin), speed=2.0)
        yield self.shake_effector()
        yield from self.m0.move(0.0, speed=0.4)
        yield from self.m0.move(0.2, speed=0.6)
        yield from self.m0.move(0.0, speed=0.6)
        yield from self.m0.move(0.2, speed=0.6)
        # Reset m0 position:
        yield from self.m0.move(end_pos, speed=0.3)

    @Action.action
    def dump(self, bin: int):
        # Align bin (m1) with m0:
        yield from self.m1.move(float(bin + 4.5), speed=2.0)
        # Dump egg (m2):
        yield from self.m2.move(+1.0, speed=2.0)
        yield from self.m2.move(+0.0, speed=2.0)

    @Action.action
    def turn_to(
        self, heading: float, err: float = 5.0, kp: float = -0.4
    ):
        """
        Heading is in degrees
        """
        while True:
            dr = ang_diff(self.heading, heading)
            if abs(dr) <= err:
                break
            vr = clamp(0.5, 1.0)(abs(dr)) * sign(dr) * kp
            self.get_logger().info(
                f"turn from {self.heading:.2f} to {heading:.2f}, dr={dr:.2f}, vr={vr:.2f}"
            )
            yield self.motion(0, 0, vr)
        yield self.halt()

    @Action.action
    def move_along(self, hdg: float, vel: float, duration: float, kp: float = None):
        deadline = time.time() + duration
        if kp is None:
            kp = -abs(vel / 2.0)
        while time.time() < deadline:
            dr = ang_diff(self.heading, hdg)
            vr = clamp(0.5, 1.0)(abs(dr)) * sign(dr) * kp
            yield self.motion(vel, 0, vr)
        yield self.halt()

    flag_success: bool = True

    @Action.action
    def approach_egg(
        self,
        camera_pos: float,
        tx: float = 0.5,
        ty: float = 0.5,
        *,
        threshold: float = 0.1,
        max_missed: int = 10,
        dt: float = 0.5,
    ):
        self.halt()
        yield from self.m0.move(camera_pos, speed=0.2)
        yield self.delay(0.5)
        det = last_det = self.det_eggs

        def wait_for_next_detection():
            nonlocal det, last_det
            while True:
                yield
                det = self.det_eggs
                if det is not last_det:
                    last_det = det
                    return

        missed_count = 0
        while True:
            yield from wait_for_next_detection()
            if len(det) == 0:
                missed_count += 1
                self.halt()
                if missed_count > max_missed:
                    self.flag_success = False
                    return
                else:
                    continue
            else:
                missed_count = 0
            delta = [(x - tx, y - ty) for x, y in det]
            dx, dy = min(delta, key=lambda d: d[0] ** 2 + d[1] ** 2)
            distance = sqrt(dx**2 + dy**2)
            self.get_logger().info(
                f"approach: dx={dx:.2f}, dy={dy:.2f} ({distance:.2f})"
            )
            if distance < threshold:
                self.halt()
                self.get_logger().info("reached the egg")
                return
            vx = clamp(0.5, 0.6)(abs(dy)) * sign(dy)
            vr = clamp(0.3, 0.4)(abs(dx)) * sign(dx)
            yield self.move(-vx, 0, -vr, dt)
            yield self.delay(1.0)


    @Action.action
    def shake_effector(self):
        yield from self.m0.move(1.0, speed=0.3)
        yield from self.m0.move(0.8, speed=0.6)
        yield from self.m0.move(1.0, speed=0.6)
        yield from self.m0.move(0.8, speed=0.6)
        yield from self.m0.move(1.0, speed=0.6)
        yield from self.m0.move(0.6, speed=0.4)

    @Action.action
    def collect_egg(self, n: int):
        self.flag_success = True
        yield self.approach_egg(0.3, tx=0.55, ty=0.7, threshold=0.20, dt=0.20)
        if not self.flag_success:
            yield self.move(-0.4, 0.0, 0.0, 1.0)
            return
        yield self.move(+0.3, 0, 0, 0.3)
        yield self.approach_egg(0.35, tx=0.65, ty=0.7, threshold=0.15, dt=0.10)
        if not self.flag_success:
            yield self.move(-0.4, 0.0, 0.0, 1.0)
            return
        yield self.move(+0.3, 0, 0, 0.3)
        yield self.approach_egg(0.4, tx=0.75, ty=0.6, threshold=0.10, dt=0.10)
        if not self.flag_success:
            yield self.move(-0.4, 0.0, 0.0, 1.0)
            return
        yield self.shake_effector()
        yield self.move(+0.4, 0.0, 0.0, 0.6)
        yield self.shake_effector()
        yield self.move(+0.4, 0.0, 0.0, 0.6)
        # yield self.shake_effector()
        # yield self.move(-0.4, 0.0, 0.0, 0.4)
        # yield self.shake_effector()
        # yield self.move(-0.4, 0.0, 0.0, 0.4)
        yield self.load(n, end_pos=0.3)
        yield self.move(-0.4, 0, 0, 2.0)

    @Action.action
    def combo(self):
        while True:
            yield from self.m0.move(0.3, speed=0.2)
            yield self.move_along(0.0, 0.6, 0.6)
            yield self.turn_to(+15.0)
            for _ in range(3):
                yield self.move_along(+15.0, 0.4, 0.4)
                yield self.delay(1.0)
                if len(self.det_eggs):
                    break
            else:
                for r in (30.0, 45.0, 15.0):
                    yield self.turn_to(r)
                    yield self.delay(1.0)
                    if len(self.det_eggs):
                        break
            for _ in range(3):
                yield self.collect_egg(0)
                if self.flag_success:
                    break
            yield self.move_along(-15.0, -0.6, 0.2)
            yield self.turn_to(-15.0)
            yield self.delay(1.0)

            yield from self.m0.move(0.3, speed=0.2)
            yield self.turn_to(-15.0)
            for _ in range(3):
                yield self.move_along(-15.0, 0.6, 0.4)
                yield self.delay(1.0)
                if len(self.det_eggs):
                    break
            else:
                for r in (-30.0, -45.0, -15.0):
                    yield self.turn_to(r)
                    yield self.delay(1.0)
                    if len(self.det_eggs):
                        break
            for _ in range(3):
                yield self.collect_egg(1)
                if self.flag_success:
                    break
            yield self.move_along(0.0, -0.6, 0.6)
            yield self.turn_to(0.0)
            yield self.move_along(0.0, 0.6, 3.0)
            yield self.move(-0.5, 0, 0, 2.5)
            yield self.turn_to(90.0)
            for n in range(2):
                yield self.dump(n)
            yield from self.m1.move(0.0, speed=2.0)
            yield self.turn_to(0.0)


@contextmanager
def expect_sigint():
    try:
        yield
    except KeyboardInterrupt:
        pass


def main(args=None):
    rclpy.init(args=args)
    robot = Mission()
    with robot.enable(), expect_sigint():
        robot.combo()
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            robot.wait_action()
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
