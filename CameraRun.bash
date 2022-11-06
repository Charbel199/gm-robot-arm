#!/bin/bash
docker run --privileged -e DISPLAY=$DISPLAY --net=host  -v /tmp/.X11-unix:/tmp/.X11-unix azure-kinect-ros 

