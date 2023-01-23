#!/bin/bash

xhost +
# docker run --rm -it --name ros2 --network host --env="DISPLAY=host.docker.internal:0" -v /tmp/.X11-unix:/tmp/.X11-unix:rw pla10/ros2_humble:amd64 /bin/bash
docker run -it --name ros2 --privileged --network host --cap-add=NET_ADMIN --env="DISPLAY=:0" -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /dev/dri:/dev/dri pla10/ros2_humble:amd64 /bin/bash
docker run -it --name ros2 --privileged --network host --cap-add=NET_ADMIN -v /dev:/dev pla10/ros2_humble:amd64 /bin/bash

# ip route add 224.0.0.0/4 dev enp1s0
