# ==============================================================================
# Author: Yuxuan Zhang (dev@z-yx.cc)
# License: TBD (UNLICENSED)
# ==============================================================================

from enum import Enum
from .stdint import uint16
from .util import clamp


class Action:

    class Type(Enum):
        # Control flow
        HALT = 0
        JUMP = 1
        LOOP = 2  # TODO
        DELAY = 3
        # GPIO
        SIGNAL = 10
        STROBE_SYNC = 11
        # MEMS Control (SPI)
        SET_VOLT_X = 22
        SET_VOLT_Y = 23
        INC_VOLT_X = 24  # TODO
        INC_VOLT_Y = 25  # TODO
        # Serial Communication (Protocol)
        REPORT_BACK = 30

    def __init__(self, amplitude: float = 120.0, limit: float = 160.0):
        self.amplitude = clamp(amplitude, 0.0, 200.0) / 200.0
        self.limit = clamp(limit, 0.0, 200.0) / 200.0

    @staticmethod
    def compile(type: Type, value: bytes = b""):
        value = value.ljust(3, b"\0")
        return bytes([type.value]) + value

    @staticmethod
    def Halt():
        return Action.compile(Action.Type.HALT)

    @staticmethod
    def Jump(address: int):
        if not (0 <= address < 32768):
            raise ValueError(f"Address {address} out of range")
        # encode as uint16
        return Action.compile(Action.Type.JUMP, uint16(address))

    @staticmethod
    def Delay(us: int):
        if not (0 <= us < 65536):
            raise ValueError(f"Delay {us} out of range")
        # encode as uint16
        return Action.compile(Action.Type.DELAY, uint16(us))

    vx: bytes | None = None
    vy: bytes | None = None

    def __scale__(self, voltage: float) -> int:
        """
        Scale desired voltage from [-1.0, 1.0] to [0, 65535].
        Imposes the safety amplitude defined by user.
        """
        amplitude = clamp(self.amplitude, 0.0, 1.0)
        limit = clamp(self.limit, 0.0, 1.0)
        pct = voltage * amplitude / 2 + 0.5
        v_min = 0.5 - 0.5 * limit
        v_max = 0.5 + 0.5 * limit
        percent = clamp(pct, v_min, v_max)
        return uint16(round(65535 * percent))

    def SetVoltX(self, voltage: float):
        vx = self.__scale__(voltage)
        if self.vx == vx:
            return None
        self.vx = vx
        return Action.compile(Action.Type.SET_VOLT_X, vx)

    def SetVoltY(self, voltage: float):
        vy = self.__scale__(voltage)
        if self.vy == vy:
            return None
        self.vy = vy
        return Action.compile(Action.Type.SET_VOLT_Y, vy)
