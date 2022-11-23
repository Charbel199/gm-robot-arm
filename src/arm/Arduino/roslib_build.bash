#!/bin/bash
source /opt/ros/melodic/setup.bash 
catkin_make 
catkin_make install
source /app/Arduino/install/setup.bash 
cd /usr/local/share/arduino/libraries 
rm -rf ros_lib 
rosrun rosserial_arduino make_libraries.py . 
