ARG UBUNTU_VERSION=18.04
# Use the official ubuntu:18.04 image as the parent image
FROM ubuntu:${UBUNTU_VERSION}

# Set the working directory to /app
WORKDIR /app

ARG DEBIAN_FRONTEND=noninteractive

ADD setup-azure.bash /app

RUN chmod +x ./setup-azure.bash
RUN ./setup-azure.bash
RUN  yes yes | DEBIAN_FRONTEND=dialog apt-get install -y k4a-tools



# 
# install ROS packages
#
FROM osrf/ros:melodic-desktop-full

SHELL ["/bin/bash", "-c"] 
WORKDIR /app
RUN mkdir catkin_ws
ADD /catkin_ws/src /app/catkin_ws/src
RUN cd /app/catkin_ws
RUN source /opt/ros/melodic/setup.bash

# install Azure Kinect SDK
ADD Azure-Kinect-Sensor-SDK /app/Azure-Kinect-Sensor-SDK
ADD setup-azure-SDK.bash /app

RUN chmod +x ./setup-azure-SDK.bash
RUN ./setup-azure-SDK.bash


