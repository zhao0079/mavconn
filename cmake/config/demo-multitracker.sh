#!/bin/sh
killall watchdog
killall px_camera
killall px_mavlink_bridge_udp
killall px_mavlink_bridge_serial
killall px_multitracker

../bin/watchdog -n -u 1 -t 0 -s 50 -l 1 -p "../bin/px_core -h" -p "../bin/px_mavlink_bridge_udp" -p "../bin/px_mavlink_bridge_serial -p /dev/ttyUSB0" -p "i1:../bin/px_camera -t -e 1400" -p "../bin/px_multitracker -m artk_markerboard_large.cfg -c firefly_mv_9480305.cal --imu-fusion -f 0.3"

