#!/bin/bash
git clone https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git
cd catkin_ws/src
git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git
cd ../..
docker build -t azure-kinect-ros .
rm -rf Azure-Kinect-Sensor-SDK
rm -rf ./catkin_ws/src/Azure_Kinect_ROS_Driver
