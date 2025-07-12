# ==============================================================================
# Author: Yuxuan Zhang (dev@z-yx.cc)
# License: TBD (UNLICENSED)
# ==============================================================================


class COBS:
    """
    COBS (Consistent Overhead Byte Stuffing) encoding and decoding.
    This class provides static methods to encode and decode data using COBS.
    """

    @staticmethod
    def encode(*data: bytes) -> bytes:
        result = bytearray(b"".join(data))
        if len(result) >= 255:
            raise ValueError(
                "COBS encoding does not support data longer than 255 bytes"
            )
        counter = 0
        for i in range(len(result) - 1, -1, -1):
            counter += 1
            if result[i] == 0:
                result[i] = counter
                counter = 0
        return bytes([0, counter + 1, *result, 0])

    @staticmethod
    def decode(input: bytes) -> tuple[bytes | None, bytes]:
        segments = input.lstrip(b"\0").split(b"\0", 1)
        if len(segments) < 2:
            return None, input if len(input) < 255 else b""
        cur, remainder = segments
        lst = bytearray(cur)
        counter = lst.pop(0) - 1
        for i, c in enumerate(lst):
            if counter == 0:
                counter = c
                lst[i] = 0
            counter -= 1
        if counter != 0:
            return None, remainder
        return bytes(lst), remainder
