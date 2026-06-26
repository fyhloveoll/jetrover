#!/usr/bin/env python3
# encoding: utf-8
# Try to reset a HUNG control-board MCU from the host via the USB-serial control
# lines (DTR/RTS). These are hardware lines independent of the locked firmware,
# so IF the board's reset pin is wired to DTR/RTS, pulsing them resets the MCU
# WITHOUT a power cycle -- enabling a software watchdog auto-recovery.
# (This is NOT the USB driver re-enumeration we tried before; this drives the
#  actual control lines.)
import sys
import time
import serial

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
s = serial.Serial(port, 115200, timeout=1)
print('opened %s' % port)

# Pattern 1: pulse DTR low->high (Arduino-style reset)
s.dtr = False
s.rts = False
time.sleep(0.3)
s.dtr = True
time.sleep(0.3)
print('pulsed DTR low->high')

# Pattern 2: pulse RTS low->high
s.rts = False
time.sleep(0.3)
s.rts = True
time.sleep(0.3)
print('pulsed RTS low->high')

# Pattern 3: both low together then both high (some boards need both)
s.dtr = False
s.rts = False
time.sleep(0.5)
s.dtr = True
s.rts = True
time.sleep(0.3)
print('pulsed DTR+RTS together')

s.close()
print('done; closed port')
