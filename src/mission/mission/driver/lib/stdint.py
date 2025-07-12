# ==============================================================================
# Author: Yuxuan Zhang (dev@z-yx.cc)
# License: TBD (UNLICENSED)
# ==============================================================================

from struct import pack
from typing import Literal, Callable

from .util import bytes_repr


class struct:
    decode: Callable[[bytes], tuple[int | float, bytes]]

    @staticmethod
    def pack(*args: bytes):
        return b"".join(args)

    @staticmethod
    def unpack(data: bytes, *args: "struct"):
        raw_data = data
        results = []
        for arg in args:
            if arg is ...:
                results.append(data)
                break
            result, data = arg.decode(data)
            results.append(result)
        else:
            if len(data) != 0:
                info = [
                    "Data not fully consumed during unpacking",
                    "========================================",
                    f"Raw data: {bytes_repr(raw_data)}",
                    *(f"{t.__name__.ljust(6)}: {r}" for t, r in zip(args, results)),
                    f"Remaining data: {bytes_repr(data)}",
                ]
                raise ValueError("\n".join(info))
        return tuple(results)


class IntegerDecoder:
    def __init__(
        self, byte_size: int, signed, byteorder: Literal["little", "big"] = "little"
    ):
        self.byte_size = byte_size
        self.signed = signed
        self.byteorder = byteorder

    def __call__(self, data: bytes) -> tuple[int, bytes]:
        if len(data) < self.byte_size:
            raise ValueError("Data too short to decode uint8")
        value = int.from_bytes(
            data[: self.byte_size], self.byteorder, signed=self.signed
        )
        return value, data[self.byte_size :]


class uint8:
    def __new__(cls, value: int):
        return pack("<B", int(value) & 0xFF)

    decode = IntegerDecoder(1, signed=False)


class uint16:
    def __new__(cls, value: int):
        return pack("<H", int(value) & 0xFFFF)

    decode = IntegerDecoder(2, signed=False)


class uint32:
    def __new__(cls, value: int):
        return pack("<I", int(value) & 0xFFFFFFFF)

    decode = IntegerDecoder(4, signed=False)


class uint64:
    def __new__(cls, value: int):
        return pack("<Q", value & 0xFFFFFFFFFFFFFFFF)

    decode = IntegerDecoder(8, signed=False)


class int8:
    def __new__(cls, value: int):
        return pack("<b", int(value) & 0xFF)

    decode = IntegerDecoder(1, signed=True)


class int16:
    def __new__(cls, value: int):
        return pack("<h", int(value) & 0xFFFF)

    decode = IntegerDecoder(2, signed=True)


class int32:
    def __new__(cls, value: int):
        return pack("<i", int(value) & 0xFFFFFFFF)

    decode = IntegerDecoder(4, signed=True)


class int64:
    def __new__(cls, value: int):
        if not (-0x8000000000000000 <= value <= 0x7FFFFFFFFFFFFFFF):
            raise ValueError("Value out of range for int64")
        return pack("<q", value)

    decode = IntegerDecoder(8, signed=True)


class float8:
    def __new__(cls, value: float):
        if not (-128.0 <= value <= 127.0):
            raise ValueError("Value out of range for float8")
        return pack("<b", value)


class float16:
    def __new__(cls, value: float):
        if not (-32768.0 <= value <= 32767.0):
            raise ValueError("Value out of range for float16")
        return pack("<h", value)


class float32:
    def __new__(cls, value: float):
        if not (-3.4028235e38 <= value <= 3.4028235e38):
            raise ValueError("Value out of range for float32")
        return pack("<f", value)
