#!/bin/bash
# Sensor bridge for the MOSAIC pipelines: board on USB -> UDP 127.0.0.1:50500, vibrator commands on UDP 50601.
# Same as python3 robot_pc_sensor_logger.py; extra options are passed on (e.g. --port /dev/ttyUSB1).
here=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
exec python3 "$here/robot_pc_sensor_logger.py" "$@"
