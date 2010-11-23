#!/bin/bash
/./home/root/pixhawk/software/embedded/build/bin/central -u -s &
/./home/root/pixhawk/software/embedded/build/bin/camInterface &
/./home/root/pixhawk/software/embedded/build/bin/mavserial --port=/dev/ttyS0 &
/./home/root/pixhawk/software/embedded/build/bin/multiTracker -w 3000 &
/./home/root/pixhawk/software/embedded/build/bin/waypointPlanning &
/./home/root/pixhawk/software/embedded/build/bin/udplink -r 192.168.1.121 &
