#!/bin/sh

killall px_house
killall px_rviz

roscore &
rosrun rviz rviz &

../bin/px_house &
../bin/px_rviz

