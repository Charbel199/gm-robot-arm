#!/bin/bash
apt update
packages=(\
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu \
    file \
    dpkg-dev \
    qemu \
    binfmt-support \
    qemu-user-static \
    pkg-config \
    ninja-build \
    doxygen \
    clang \
    python3 \
    gcc \
    g++ \
    git \
    git-lfs \
    nasm \
    cmake \
    powershell \
    libgl1-mesa-dev \
    libsoundio-dev \
    libjpeg-dev \
    libvulkan-dev \
    libx11-dev \
    libxcursor-dev \
    libxinerama-dev \
    libxrandr-dev \
    libusb-1.0-0-dev \
    libssl-dev \
    libudev-dev \
    mesa-common-dev \
    libopencv-dev \
    uuid-dev )
apt-get install -y --no-install-recommends ${packages[@]}
cd Azure-Kinect-Sensor-SDK
mkdir build 
cd build
cmake .. -GNinja
ninja

