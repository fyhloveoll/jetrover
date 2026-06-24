#!/usr/bin/env bash
# Soft-reset the control board USB (de-authorize + re-authorize) to try
# recovering a hung board WITHOUT a full reboot. Run when board is hung.
set -e
DEV=$(for d in /sys/bus/usb/devices/*/; do p=$(cat "$d/product" 2>/dev/null); [ "$p" = "USB Single Serial" ] && basename "$d" && break; done)
[ -z "$DEV" ] && { echo "control board USB device not found"; exit 1; }
echo "control board USB device = $DEV"
echo "de-authorize..."; echo 0 | sudo tee /sys/bus/usb/devices/$DEV/authorized >/dev/null
sleep 2
echo "re-authorize...";  echo 1 | sudo tee /sys/bus/usb/devices/$DEV/authorized >/dev/null
sleep 2
echo "ttyACM after reset:"; ls -l /dev/ttyACM* /dev/rrc 2>&1
echo "Note: ros_robot_controller must be restarted to reopen the serial port."
