#!/bin/bash
xhost + 
docker run \
--name apr \
-it \
--privileged \
--network host \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v ${HOME}/.ssh:/root/.ssh \
-v /dev/bus/usb:/dev/bus/usb \
-v $(pwd)/catkin_ws/src:/catkin_ws/src \
-v $(pwd)/packages:/packages \
apr:latest \
bash
xhost -