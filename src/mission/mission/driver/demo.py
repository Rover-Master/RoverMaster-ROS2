#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
from sys import stdout, stderr

from .lib.expect import Expect
from .lib.driver import Driver
from .lib.motor import Motor
from .lib.util import debounce, loop_for

parser = argparse.ArgumentParser(description="Motor Demo")
parser.add_argument(
    "-B", "--baud", type=int, default=115200, help="Baud rate for serial communication"
)
parser.add_argument(
    "-v",
    "--verbose",
    action="count",
    default=0,
    help="Increase verbosity level",
)
parser.add_argument(
    "-p",
    "--plot",
    action="store_true",
    help="Enable plotting of stall guard results",
)
args = parser.parse_args()
baud = int(args.baud)
verbose = int(args.verbose)
should_plot = bool(args.plot)

driver = Driver(baudrate=baud, verbose=verbose)
print(f"Device Info: {driver.getInfo()}", file=stderr)
m0 = Motor(driver, id=0, invert=1, scale=6.0, init_pos=0.0)
m1 = Motor(driver, id=1, invert=0, scale=1.0, init_pos=0.0)
m2 = Motor(driver, id=2, invert=1, scale=0.92, init_pos=0.0)


@debounce(1.0 / 20.0)
def tick():
    if not should_plot:
        return
    status = [m.getStatus() for m in (m0, m1, m2)]
    diag = (f"m{i}.diag={s.diag_pin * 100 - 100}" for i, s in enumerate(status))
    sg_result = (f"m{i}.sg={s.sg_result}" for i, s in enumerate(status))
    print("$", *diag, *sg_result)
    stdout.flush()


try:
    with driver.enable(), m0.enable(), m1.enable(), m2.enable():
        while True:
            for slot in range(4):
                Expect.wait(m0.move(+1.0, speed=0.3), m1.move(slot, speed=1.0))
                for _ in loop_for(0.2):
                    tick()
                Expect.wait(m0.move(+0.0, speed=0.3))
                for _ in loop_for(0.2):
                    tick()
                Expect.wait(m0.move(+0.3, speed=0.3))
                for _ in loop_for(1.0):
                    tick()
            for slot in range(4):
                Expect.wait(m1.move(slot + 4.5, speed=1.0))
                Expect.wait(m2.move(1.0, speed=3.0))
                for _ in loop_for(0.2):
                    tick()
                Expect.wait(m2.move(0.0, speed=3.0))
                for _ in loop_for(0.2):
                    tick()
            m1.setPosition(m1.position % 6.0)
except KeyboardInterrupt:
    pass

driver.close()
