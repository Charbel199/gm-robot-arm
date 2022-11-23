#!/bin/bash
cd src
unzip rosserial.zip
cd ..
docker build -t arduino-ros .
rm -rf ./catkin_ws/src/rosserial
