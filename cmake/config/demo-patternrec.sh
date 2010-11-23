#!/bin/sh
killall camera
killall central
killall px_udplink
killall px_core
killall patternrec
../bin/central -u -s &
../bin/px_udplink &
../bin/px_core &
../bin/camera &
../bin/patternrec -i ../media/face1.png -i ../media/face2.png -i ../media/face3.png -i ../media/face4.png -i ../media/face5.png
