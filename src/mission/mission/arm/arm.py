import time, sys
from . import util


class DummySerial:
    is_open: False

    def write(*args, **kwargs):
        pass

    def recv(*args, **kwargs):
        pass


class Joint:
    ready = False
    moving = True
    pos = 0

    def __init__(
        self,
        name: str,
        max_speed: float = 60,
        range: tuple[float, float] | None = None,
        infinite: bool = False,
    ):
        self.name = name
        self.max_speed = max_speed
        self.range = range
        self.infinite = infinite


class Arm:
    tick = None
    J1, J2, J3, Z = JOINTS = [
        Joint("J1", 30, (-270.0, 270.0)),
        Joint("J2", 60, (-140.0, 140.0)),
        Joint("J3", 90, infinite=True),
        Joint("Z", 20, (0.0, 100.0)),
    ]

    def __init__(self, vid: int = None, pid: int = None, serial=None):
        if serial is not None:
            self.serial = serial
        else:
            self.serial = util.open_serial_port(util.find_serial_port(vid, pid))
        self.send("\nENABLE\n")

    def __del__(self):
        if self.serial is not None and self.serial.is_open:
            self.send("\nDISABLE\n")
            self.serial.close()

    def init(self):
        time.sleep(0.1)
        self.send(f"MOVE E=0\n")
        self.home(self.J3)
        self.home(self.J2)
        self.move(J2=135)
        self.home(self.J1)

    def send(self, msg: str):
        print(">>", msg.strip())
        self.serial.write(msg.encode())

    recv_buffer: str = ""

    def recv(self):
        msg = self.serial.read_all()
        if msg is not None:
            try:
                self.recv_buffer += str(msg, encoding="ASCII")
            except Exception as e:
                print(e, file=sys.stderr)
        if "\n" in self.recv_buffer:
            line, self.recv_buffer = self.recv_buffer.split("\n", 1)
            print("<<", line.strip())
            return line
        return None

    def handle_message(self, line: str | None):
        if line is None:
            return
        segs = [token for token in line.split(" ") if len(token)]
        if len(segs) <= 0:
            return
        cmd, args = segs[0], segs[1:] if len(segs) > 1 else None
        match (cmd):
            case "INFO" | "ERROR":
                # print(line)
                return
            case "READY":
                for joint_name in args:
                    for joint in self.JOINTS:
                        if joint.name == joint_name:
                            joint.ready = True
            case "DONE":
                for joint_name in args:
                    for joint in self.JOINTS:
                        if joint.name == joint_name:
                            joint.moving = False
            case "RANGE":
                for arg in args:
                    segs = arg.split("=")
                    if len(segs) != 2:
                        print(f"ERROR ill formed arg: {arg}")
                        continue
                    pass
            case "SYNC":
                for arg in args:
                    segs = arg.split("=")
                    if len(segs) != 2:
                        print(f"ERROR ill formed arg: {arg}")
                        continue
                    joint_name, value = segs
                    value = float(value)
                    for joint in self.JOINTS:
                        if joint.name == joint_name:
                            joint.pos = value
            case _:
                print(f"ERROR UNKNOWN MESSAGE: {line}")

    def ready(self, *joints: Joint):
        for joint in joints:
            if not joint.ready:
                return False
        return True

    def moving(self, *joints: Joint):
        for joint in joints:
            if joint.moving:
                return True
        return False

    def home(self, *joints: Joint):
        names = " ".join([j.name for j in joints])
        self.send(f"HOME {names}\n")
        while not self.ready(*joints):
            self.handle_message(self.recv())
            if self.tick is not None:
                self.tick()

    def speed(self, **kwargs: dict[str, float]):
        command = ["SPEED"]
        for j in self.JOINTS:
            if j.name in kwargs.keys():
                command.append(f"{j.name}={kwargs[j.name]}")
            else:
                command.append(f"{j.name}={j.max_speed}")
        self.send(" ".join(command) + "\n")

    def move(self, **kwargs: dict[str, float]):
        joints_to_move = []
        command = ["MOVE"]
        for j in self.JOINTS:
            if j.name in kwargs.keys():
                joints_to_move.append(j)
                pos = kwargs[j.name]
                command.append(f"{j.name}={pos}")
                j.moving = abs(float(j.pos) - float(pos)) > 1.0
        self.send(" ".join(command) + "\n")
        while self.moving(*joints_to_move):
            self.handle_message(self.recv())
            if self.tick is not None:
                self.tick()

    def plan(self, *options: list[list[float | None]]):
        optimal_angles = None
        optimal_duration = None
        for option in options:
            if len(option) < 3:
                continue
            duration = 0
            current_option: dict[str, float] = {}
            valid = True
            for joint, dst in [
                (self.J1, option[0]),
                (self.J2, option[1]),
                (self.J3, option[2]),
            ]:
                if dst is None:
                    current_option[joint.name] = joint.pos
                    continue
                if joint.range is not None and not joint.infinite:
                    # Validate range
                    if dst < joint.range[0] or dst > joint.range[1]:
                        valid = False
                        break
                if joint.infinite:
                    # Infinite joint, choose optimal direction
                    dst_forward = (dst - joint.pos) % 360.0
                    dst_backward = 360.0 - dst_forward
                    # Choose the shortest path
                    if dst_forward <= dst_backward:
                        dst = joint.pos + dst_forward
                    else:
                        dst = joint.pos - dst_backward
                # Calculate duration required to cover the distance
                joint_duration = abs(joint.pos - dst) / joint.max_speed
                duration = max(duration, joint_duration)
                current_option[joint.name] = dst
            if valid:
                if optimal_duration is None or duration < optimal_duration:
                    optimal_angles = current_option
                    optimal_duration = duration
        if optimal_angles is None or optimal_duration is None:
            return None, None, None
        def find_speed(j: Joint, pos: float, duration: float):
            if abs(duration) <= 1e-3:
                return j.max_speed
            dst = abs(j.pos - optimal_angles[j.name])
            return  dst / optimal_duration
        optimal_speeds = {
            j.name: find_speed(j, dst, optimal_duration)
            for j in self.JOINTS
            if j.name in optimal_angles
        }
        return optimal_angles, optimal_speeds, optimal_duration
