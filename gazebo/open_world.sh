#!/bin/bash
# Close any running Gazebo and open a world: open_world.sh TRACK   (run in WSL)
pkill -f "[g]z sim" ; sleep 2
source /opt/ros/jazzy/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
cd "$(dirname "$0")"
# worlds/ is generated for this clone's path (the floor texture is referenced by absolute path)
[ -f "worlds/$1.sdf" ] || python3 make_world.py "$1" || exit 1
setsid nohup gz sim -r "worlds/$1.sdf" > /tmp/gz_$1.log 2>&1 < /dev/null &
sleep 20   # WSL ends the session's children if the shell returns at once
