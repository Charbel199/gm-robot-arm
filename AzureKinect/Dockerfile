ARG UBUNTU_VERSION=18.04
# Use the official ubuntu:18.04 image as the parent image
FROM osrf/ros:melodic-desktop-full

# Set the working directory to /app
WORKDIR /app

ARG DEBIAN_FRONTEND=noninteractive

ADD setup-azure.bash /app

RUN chmod +x ./setup-azure.bash
RUN ./setup-azure.bash
ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'libk4a1.3 libk4a1.3/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | debconf-set-selections
RUN echo 'libk4a1.3 libk4a1.3/accept-eula boolean true' | debconf-set-selections
RUN echo 'libk4abt1.0 libk4abt1.0/accepted-eula-hash string 03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | debconf-set-selections
RUN echo 'libk4abt1.0 libk4abt1.0/accept-eula boolean true' | debconf-set-selections
RUN apt-get install -y --no-install-recommends k4a-tools=1.3.0 \
libk4a1.3-dev \
libk4abt1.0-dev 



# 
# install ROS packages
#

SHELL ["/bin/bash", "-c"] 
WORKDIR /app
RUN mkdir catkin_ws
ADD /catkin_ws/src /app/catkin_ws/src
RUN cd /app/catkin_ws

# install Azure Kinect SDK
ADD Azure-Kinect-Sensor-SDK /app/Azure-Kinect-Sensor-SDK
ADD setup-azure-SDK.bash /app

ARG DEBIAN_FRONTEND=noninteractive
RUN chmod +x ./setup-azure-SDK.bash
RUN ./setup-azure-SDK.bash

ADD catkin_make.bash /app
RUN ./catkin_make.bash

COPY Azure-Kinect-Sensor-SDK/scripts/99-k4a.rules /etc/udev/rules.d
# Dependencies for glvnd and X11.
RUN apt-get update \
  && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
  && rm -rf /var/lib/apt/lists/*# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
CMD source catkin_ws/devel/setup.bash && roslaunch azure_kinect_ros_driver driver.launch depth_enabled:=false




