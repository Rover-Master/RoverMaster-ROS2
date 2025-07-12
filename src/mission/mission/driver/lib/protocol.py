# ==============================================================================
# Author: Yuxuan Zhang (dev@z-yx.cc)
# License: TBD (UNLICENSED)
# ==============================================================================

from enum import Enum
from sys import stderr
from time import time as now, sleep

from .util import bytes_repr

METHOD_MASK = 0xF0


class Method(Enum):
    NOP = 0x00
    # HOST -> DEVICE
    GET = 0x10
    SET = 0x20
    # DEVICE -> HOST
    ACK = 0x30
    REJ = 0x40
    # DEVICE -> HOST, for asynchronous events
    SYN = 0x80
    # Special Log Method
    LOG = 0xF0


PROP_MASK = 0x0F


class Prop(Enum):
    NONE = None
    FW_INFO = 0x0
    SYS_ENA = 0x1
    MOT_ENA = 0x2
    MOT_CFG = 0x3
    MOT_MOV = 0x4
    MOT_HOME = 0x5
    MOT_STAT = 0x6
    LED_PROG = 0xA
    ODOM_SENSOR = 0xB
    COLOR_SENSOR = 0xC


@staticmethod
def encode(method: Method, prop: Prop, *data: bytes) -> bytes:
    payload = bytearray([0, method.value | prop.value, *b"".join(data)])
    # XOR all bytes in the payload with the first byte
    for byte in payload[1:]:
        payload[0] ^= byte
    return bytes(payload)


@staticmethod
def decode(frame: bytes) -> tuple[Method, Prop, bytes] | None:
    if len(frame) < 2:
        return None

    checksum = frame[0]

    # Verify checksum
    for byte in frame[1:]:
        checksum ^= byte

    if checksum != 0:
        print(f"Frame Dropped: bad checksum 0x{checksum:02x}", file=stderr)
        return None

    code = frame[1]
    try:
        method = Method(code & METHOD_MASK)
        prop = Prop(code & PROP_MASK)
    except:
        print(f"Bad frame header 0x{code:02x}", file=stderr)
        return None

    payload = frame[2:]

    return method, prop, bytes(payload)


class PacketChain:
    FLAG_TERM = object()  # Sentinel for termination

    method: Method
    prop: Prop
    payload: bytes

    # Lock-free linked list for chaining packets
    # next is read-only for consumers,
    # only one thread (thread) can write to this value once.
    next: "PacketChain | None" = None

    def __init__(
        self,
        method: Method,
        prop: Prop,
        payload: bytes = b"",
    ):
        self.method = method
        self.prop = prop
        self.payload = payload

    def __call__(self, timeout: float | None = None) -> "PacketChain.Iterator":
        if timeout is not None:
            ddl = now() + timeout
        else:
            ddl = None
        return self.Iterator(self, ddl)

    def __repr__(self):
        return f"{self.id} [{bytes_repr(self.payload)}]"

    def __str__(self):
        return repr(self)

    @property
    def id(self):
        return f"{self.method.name}::{self.prop.name}"

    class Iterator:
        initialized: bool = False

        def __init__(self, head: "PacketChain", ddl: float | None = None):
            self.head = head
            self.ddl = ddl

        def __iter__(self):
            return self

        def __next__(self):

            head = self.head
            if self.initialized:
                while head.next is None:
                    if self.ddl is not None and now() > self.ddl:
                        raise StopIteration
                    sleep(0.001)
                self.head = head.next
            else:
                self.initialized = True
            if self.head is PacketChain.FLAG_TERM:
                raise StopIteration
            return self.head
