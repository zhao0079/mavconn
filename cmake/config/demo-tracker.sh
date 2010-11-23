#!/bin/sh
killall camera
killall central
killall px_udplink
killall px_core
killall px_tracker
../bin/central -u -s &
../bin/px_udplink &
../bin/px_core &
../bin/camera &
../bin/px_tracker --camera-calibration firefly_mv_9480042.cal
