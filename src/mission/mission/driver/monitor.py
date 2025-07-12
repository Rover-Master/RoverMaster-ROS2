#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
from .lib.protocol import Method, Prop
from .lib.driver import Driver

parser = argparse.ArgumentParser(description="Serial Monitor")
parser.add_argument(
    "-B", "--baud", type=int, default=115200, help="Baud rate for serial communication"
)
parser.add_argument(
    "-v", "--verbose", action="store_true", help="Enable verbose output"
)
args = parser.parse_args()
baud = int(args.baud)
verb = bool(args.verbose)

driver = Driver(baudrate=baud, verbose=verb)

try:
    info = driver(Method.GET, Prop.FW_INFO, expect=(Method.ACK, Prop.FW_INFO))
    print(f"Device Info: {info.decode('utf-8') if info else 'Unknown'}")
    while True:
        data = driver.recv()
        if data:
            print(data)
except KeyboardInterrupt:
    pass
