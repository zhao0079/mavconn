#!/bin/sh
killall px_udplink
killall px_asctecserial
killall qgroundstation

../../../groundcontrol/qgroundcontrol &
../bin/px_udplink &
../bin/px_asctecserial
