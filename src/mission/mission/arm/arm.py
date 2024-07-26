import time
from . import util


class Joint:
    name = None
    ready = False
    moving = True
    pos = 0

    def __init__(self, name: str = None):
        self.name = name


class Arm:
    J1, J2, J3, Z = JOINTS = [Joint("J1"), Joint("J2"), Joint("J3"), Joint("Z")]

    def __init__(self, vid: int = None, pid: int = None):
        self.serial = util.open_serial_port(util.find_serial_port(vid, pid))
        self.send("\nENABLE\n")

    def __del__(self):
        if self.serial is not None:
            self.send("\nDISABLE\n")
            self.serial.close()

    def init(self):
        time.sleep(0.1)
        self.home(self.J3)
        self.home(self.J2)
        self.move(J2=135)
        self.home(self.J1)

    def send(self, msg: str):
        print(">>", msg.strip())
        self.serial.write(bytes(msg, encoding="ASCII"))

    recv_buffer: str = ""

    def recv(self):
        msg = self.serial.read_all()
        if msg is not None:
            self.recv_buffer += str(msg, encoding="ASCII")
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
