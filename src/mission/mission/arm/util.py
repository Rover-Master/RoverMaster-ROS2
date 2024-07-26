import serial
import serial.tools.list_ports


def find_serial_port(vid: int, pid: int) -> str | None:
    vid, pid = int(vid), int(pid)
    for port in serial.tools.list_ports.comports():
        if port.vid == vid and port.pid == pid:
            return port.device
    return None


def open_serial_port(port: str | None, baudrate: int = 115200) -> serial.Serial:
    if port is None:
        raise RuntimeError("Serial port not found (port = None)")
    serial_port = serial.Serial(
        port=port,
        baudrate=baudrate,
        parity=serial.PARITY_ODD,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.SEVENBITS,
    )
    if not serial_port.isOpen():
        raise RuntimeError(f"Cannot open serial port {port}")
    return serial_port
