#!/bin/bash
docker run \
        -it \
        --rm \
        --network=host \
        -e DISPLAY=$DISPLAY \
        -v $HOME/.Xauthority:/home/developer/.Xauthority \
        --privileged\
        -v $HOME/Arduino:/home/developer/Arduino \
        arduino-ros\
        /bin/bash
