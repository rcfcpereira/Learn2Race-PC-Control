#!/bin/bash
# RViz2 in WSL showing Robot B's SLAM map (needs networkingMode=mirrored in %USERPROFILE%\.wslconfig).
# From Windows:  wsl -d Ubuntu-24.04 -- bash /mnt/c/<path to this repository>/rviz/rviz_b.sh
source /opt/ros/jazzy/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1  # WSLg GPU driver fails the map shader
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
exec rviz2 -d "$(dirname "$0")/slam_view.rviz"
