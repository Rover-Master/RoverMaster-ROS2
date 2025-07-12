# ==============================================================================
# Author: Yuxuan Zhang (dev@z-yx.cc)
# License: TBD (UNLICENSED)
# ==============================================================================

from contextlib import contextmanager
from typing import TypedDict, Unpack
from sys import stderr
from dataclasses import dataclass

from .expect import Expect
from .stdint import struct, uint8, uint16, int64, uint64
from .driver import Driver
from .protocol import Method, Prop


class Config(TypedDict):
    invert: int  # forward = HIGH (false) or LOW (true)
    micro_steps: int  # 1, 2, 4, 8, 16, 32, 64, 128, 256
    stall_sensitivity: int  # 0-255, 0 = disabled
    rms_current: int  # mA

    @staticmethod
    def decode(data: bytes) -> "Motor.Config":
        id, invert, micro_steps, stall_sensitivity, rms_current = struct.unpack(
            data, uint8, uint8, uint8, uint8, uint16
        )
        return Config(
            invert=invert,
            micro_steps=micro_steps,
            stall_sensitivity=stall_sensitivity,
            rms_current=rms_current,
        )

    @staticmethod
    def encode(config: "Config") -> bytes:
        return struct.pack(
            uint8(config["invert"]),
            uint8(config["micro_steps"]),
            uint8(config["stall_sensitivity"]),
            uint16(config["rms_current"]),
        )


@dataclass
class MoveCommand:
    id: int
    target: int
    step_time: int

    @classmethod
    def decode(cls, data: bytes):
        return cls(*struct.unpack(data, uint8, int64, uint64))

    def encode(self):
        return struct.pack(
            uint8(self.id),
            int64(self.target),
            uint64(self.step_time),
        )


@dataclass
class Status:
    id: int
    diag_pin: int
    sg_result: int
    position: int

    @classmethod
    def decode(cls, data: bytes) -> "Status":
        return cls(*struct.unpack(data, uint8, uint8, uint16, int64))


class Motor:

    id: int

    def match_packet(self, buf: bytes) -> bool:
        id, _ = struct.unpack(buf, uint8, ...)
        return id == self.id

    config = Config(
        invert=0,
        micro_steps=32,
        stall_sensitivity=50,
        rms_current=1000,
    )

    def __init__(
        self,
        driver: Driver,
        id: int,
        scale: float = 1.0,
        init_pos: float | None = None,
        **config: Unpack[Config],
    ):
        self.id = id
        self.driver = driver
        self.scale = scale
        self.init_pos = init_pos
        self.configure(**config)

    def configure(self, **conf: Config):
        self.config.update(conf)
        self.driver(
            Method.SET,
            Prop.MOT_CFG,
            uint8(self.id),
            Config.encode(self.config),
            expect=(Method.ACK, Prop.MOT_CFG),
        )

    def _set_enable(self, enable: bool):
        """Set the motor enable state."""
        self.driver(
            Method.SET,
            Prop.MOT_ENA,
            uint8(self.id),
            uint8(1 if enable else 0),
            expect=(Method.ACK, Prop.MOT_ENA),
        )

    @contextmanager
    def enable(self):
        self._set_enable(True)
        if self.init_pos is not None:
            self.setPosition(self.init_pos)
        yield self
        self._set_enable(False)

    @property
    def steps_per_unit(self) -> float:
        # When scale == 1.0, 1 unit equals 1 revolution of the motor axle
        return self.scale * self.config["micro_steps"] * 200.0

    cached_position: float | None = None

    @property
    def position(self) -> float:
        if self.cached_position is None:
            return self.getPosition()
        else:
            return self.cached_position

    def getPosition(self) -> int:
        buffer = self.driver(
            Method.GET,
            Prop.MOT_MOV,
            uint8(self.id),
            expect=(
                Method.ACK,
                Prop.MOT_MOV,
                self.match_packet,
            ),
        )
        packet = MoveCommand.decode(buffer)
        self.cached_position = packet.target / self.steps_per_unit
        return self.cached_position

    def setPosition(self, position: float):
        self.cached_position = float(position)
        self.driver(
            Method.SET,
            Prop.MOT_MOV,
            MoveCommand(
                id=self.id,
                target=self.absolutePosition(position),
                step_time=0,  # Directly set position, no movement
            ).encode(),
            expect=(Method.ACK, Prop.MOT_MOV, self.match_packet),
        )

    def getStatus(self):
        buffer = self.driver(
            Method.GET,
            Prop.MOT_STAT,
            uint8(self.id),
            expect=(Method.ACK, Prop.MOT_STAT, self.match_packet),
        )
        return Status.decode(buffer)

    def absolutePosition(self, delta: float) -> int:
        return int(round(delta * self.steps_per_unit))

    def move(
        self,
        position: float,
        *,
        duration: float | None = None,
        speed: float | None = None,
    ):
        current_pos = self.absolutePosition(self.position)
        target_pos = self.absolutePosition(position)
        steps = abs(target_pos - current_pos)
        if steps == 0:
            return Expect.Fulfilled()
        if speed is not None and duration is not None:
            raise ValueError("Specify either duration or speed, not both.")
        if speed is not None:
            duration = abs((position - self.position) / speed)
        if duration is None:
            raise ValueError("Either duration or speed must be specified.")
        if duration <= 0:
            raise ValueError("Duration must be greater than 0")

        step_interval = int(round(1e6 * duration / steps))  # us

        def check_ack(data: bytes) -> bool:
            packet = MoveCommand.decode(data)
            if packet.id == self.id:
                if packet.target != target_pos:
                    raise RuntimeError(
                        f"Target position mismatch: expected {target_pos}, got {packet.target}"
                    )
                self.cached_position = packet.target / self.steps_per_unit
                return True
            else:
                return False

        self.cached_position = None
        expect = self.driver.expect(Method.ACK, Prop.MOT_MOV, check_ack)
        self.driver(
            Method.SET,
            Prop.MOT_MOV,
            uint8(self.id),
            uint64(target_pos),
            uint64(step_interval),
        )

        return expect
