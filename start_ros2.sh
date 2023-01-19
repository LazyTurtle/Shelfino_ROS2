#!/bin/bash
docker container start ros2
docker container exec -it ros2 /bin/bash -c "source /root/startup.sh; ros2 launch shelfino_node bringup_nav.launch.py robot_id:=shelfino2"