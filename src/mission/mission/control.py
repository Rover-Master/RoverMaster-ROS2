import argparse, serial, time, sys, atexit

parser = argparse.ArgumentParser(description="Robot arm data collection tool")
parser.add_argument(
    "-p", "--port", help="Serial port of the arm controller", default="/dev/ttyACM0"
)
args = parser.parse_args()

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port=args.port,
    # port=2341,
    baudrate=115200,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS,
)

if not ser.isOpen():
    print(f"Cannot open serial port {args.port}, exiting...")
    sys.exit(1)


class Joint:
    name = None
    ready = False
    moving = True
    pos = 0

    def __init__(self, name: str = None):
        self.name = name


j1, j2, j3, z = JOINTS = [Joint("J1"), Joint("J2"), Joint("J3"), Joint("Z")]


def send(msg: str):
    print(">>", msg.strip())
    ser.write(bytes(msg, encoding="ASCII"))


recv_buffer: str = ""


def recv():
    global recv_buffer
    msg = ser.read_all()
    if msg is not None:
        recv_buffer += str(msg, encoding="ASCII")
    if "\n" in recv_buffer:
        line, recv_buffer = recv_buffer.split("\n", 1)
        print("<<", line.strip())
        return line
    return None


def handle_message(line: str | None):
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
                for joint in JOINTS:
                    if joint.name == joint_name:
                        joint.ready = True
        case "DONE":
            for joint_name in args:
                for joint in JOINTS:
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
                for joint in JOINTS:
                    if joint.name == joint_name:
                        joint.pos = value
        case _:
            print(f"ERROR UNKNOWN MESSAGE: {line}")


def ready(*joints: Joint):
    for joint in joints:
        if not joint.ready:
            return False
    return True

def moving(*joints: Joint):
    for joint in joints:
        if joint.moving:
            return True
    return False

def home(*joints: Joint):
    names = " ".join([j.name for j in joints])
    send(f"HOME {names}\n")
    while not ready(*joints):
        handle_message(recv())

def move(**kwargs: dict[str, float]):
    joints_to_move = []
    command = ["MOVE"]
    for j in JOINTS:
        if j.name in kwargs.keys():
            joints_to_move.append(j)
            pos = kwargs[j.name]
            command.append(f"{j.name}={pos}")
            j.moving = abs(float(j.pos) - float(pos)) > 1.0
    send(" ".join(command) + "\n")
    while moving(*joints_to_move):
        handle_message(recv())


def exit_handler():
    send("\nDISABLE\n")
    ser.close()

atexit.register(exit_handler)

def init():
    send("\nENABLE\n")
    time.sleep(0.1)
    home(j3)
    home(j2)
    move(J2=135)
    home(j1)
