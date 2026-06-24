#!/usr/bin/env bash
# Detect control-board hang via IMU liveness (CLI echo, which reliably receives /imu).
# On hang -> dump  sudo dmesg + USB state before any reboot. Passive, no auto-recovery.
source /home/ubuntu/jetrover_ws/jr_env.sh
LOGDIR=/home/ubuntu/jetrover_ws/hang_logs
mkdir -p "$LOGDIR"
fired=0
echo "$(date) imu_watchdog.sh up; logs -> $LOGDIR"
while true; do
  if timeout 4 ros2 topic echo /imu --once >/dev/null 2>&1; then
    [ "$fired" = 1 ] && echo "$(date) IMU back -> board recovered"
    fired=0
  else
    if [ "$fired" = 0 ]; then
      ts=$(date +%Y%m%d_%H%M%S); F="$LOGDIR/hang_$ts.txt"
      echo "$(date) IMU SILENT -> HANG; dumping $F"
      {
        echo "=== HANG @ $ts ==="; date; uptime; echo
        echo "##  sudo dmesg tail";  sudo dmesg | tail -80; echo
        echo "## lsusb"; lsusb; echo
        echo "## ttyACM"; ls -l /dev/ttyACM* /dev/rrc 2>&1; echo
        echo "## USB power/control"
        for d in /sys/bus/usb/devices/1-2 /sys/bus/usb/devices/1-2.1; do echo "-- $d"; cat $d/power/control 2>/dev/null; cat $d/product 2>/dev/null; done
      } > "$F" 2>&1
      echo "$(date) evidence saved: $F"
      fired=1
    fi
  fi
  sleep 3
done
