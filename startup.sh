#!/bin/bash
source "/opt/ros/humble/setup.bash"
source "/root/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=3

route add -net 224.0.0.0 netmask 240.0.0.0 dev enp1s0
ifconfig enp1s0 multicast