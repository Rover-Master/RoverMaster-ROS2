# ==============================================================================
# Author: Yuxuan Zhang (dev@z-yx.cc)
# License: TBD (UNLICENSED)
# ==============================================================================

from threading import Lock, Thread
from sys import stderr
from contextlib import contextmanager
from serial import Serial
from serial.tools import list_ports
from typing import Callable

from .expect import Expect
from .cobs import COBS
from .protocol import Method, Prop, encode, decode, PacketChain
from .stdint import uint8
from .util import bytes_repr


def locate(vid, pid):
    ports = list_ports.comports()
    for port in ports:
        if port.vid == vid and port.pid == pid:
            return port.device
    return None


class Driver(Serial):
    def __init__(
        self,
        vid: int = 0x2341,
        pid: int = 0x0070,
        baudrate: int = 115200,
        verbose: int = 0,
    ):
        self.verbose = verbose
        port = locate(vid, pid)
        if self.verbose:
            print(f"Opening serial device {vid:04X}:{pid:04X} at {port}", file=stderr)
        if port is None:
            self.is_open = False
            raise IOError(f"Serial device {vid:04X}:{pid:04X} not found.")
        super().__init__(port, baudrate=baudrate, timeout=0.001)
        if not self.is_open:
            raise IOError(f"Failed to open serial port {self.portstr}")
        # Start the reader thread
        self.reader_thread = Thread(target=self.reader, daemon=True)
        self.reader_thread.start()

    def getInfo(self) -> str:
        info = self(Method.GET, Prop.FW_INFO, expect=(Method.ACK, Prop.FW_INFO))
        if info:
            return info.decode("utf-8", errors="replace")
        else:
            return "Unknown Device"

    @contextmanager
    def enable(self):
        if not self.is_open:
            raise IOError("Serial port is not open.")
        try:
            self(Method.SET, Prop.SYS_ENA, uint8(1))
            yield self
        finally:
            self(Method.SET, Prop.SYS_ENA, uint8(0))

    tx_lock = Lock()

    def __call__(
        self,
        method: Method,
        prop: Prop,
        *args: bytes,
        expect: (
            tuple[Method, Prop] | tuple[Method, Prop, Callable[[bytes], bool]] | None
        ) = None,
        retry_interval: float = 0.5,
    ) -> bytes:
        rx = self.rx
        while True:
            if self.verbose:
                _args = b"".join(args)
                print(
                    f"[DRV] << {method.name}::{prop.name} [{bytes_repr(_args)}] ({len(_args)} bytes)",
                    file=stderr,
                )
            packet = encode(method, prop, *args)
            self.write(COBS.encode(packet))
            self.flush()
            if expect is None:
                return b""
            if len(expect) == 2:
                m, p = expect
                check = lambda payload: True
            elif len(expect) == 3:
                m, p, check = expect
            else:
                raise ValueError(f"Invalid expect format: {expect}")
            for next in rx(retry_interval):
                rx = next
                if m == rx.method and p == rx.prop and check(rx.payload):
                    return rx.payload
                elif self.verbose >= 2:
                    print(
                        f"[DRV] >> Packet Mismatch: {rx.id} (expected {m.name}::{p.name})",
                        file=stderr,
                    )

    rx_buffer: bytes = b""

    def recv(self):
        data = self.read(size=128)
        if data and self.verbose >= 3:
            print(f"[DRV] >>", bytes_repr(data), file=stderr)
        self.rx_buffer += data
        frame, self.rx_buffer = COBS.decode(self.rx_buffer)
        if frame is None:
            return None
        if self.verbose >= 3:
            preview = frame[2:].decode("ascii", errors="replace")
            print(f'      >> "{preview}"', file=stderr)
        try:
            packet = decode(frame)
            if packet is None:
                print(f"Invalid frame: {bytes_repr(frame)}", file=stderr)
            return packet
        except ValueError as e:
            print(f"Decoding error: {e} for frame {bytes_repr(frame)}", file=stderr)
            return None

    rx = PacketChain(Method.NOP, Prop.NONE, b"")
    _sig_term = False

    def reader(self):
        while not self._sig_term:
            packet = self.recv()
            if packet is None:
                continue
            method, prop, payload = packet
            if method == Method.LOG:
                print(
                    f"[LOG] >> {payload.decode('ascii', errors='replace')}",
                    file=stderr,
                )
            else:
                rx = self.rx
                self.rx = PacketChain(method, prop, payload)
                rx.next = self.rx
                if self.verbose:
                    print("[DRV] >>", self.rx, file=stderr)
        # Signal termination to iterators
        self.rx.next = PacketChain.FLAG_TERM

    def close(self):
        self._sig_term = True
        self.reader_thread.join()
        super().close()

    def expect(
        self, method: Method, prop: Prop, payload: Callable[[bytes], bool] | None = None
    ):
        """Create an expectation for a specific packet."""
        return Expect(self.rx, method, prop, payload)
