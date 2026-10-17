#!/bin/bash
# Figure appe_gazebo_track.png (appendix): render a world from a fixed camera, headless, in WSL:
#   CAMPOSE="x y z roll pitch yaw" overrides the dragster camera (default: behind the start line)
#   wsl -d Ubuntu-24.04 -- bash /mnt/c/.../docs/sim/gazebo/figure/run.sh portimao /mnt/c/.../out.png
source /opt/ros/jazzy/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
D=$(dirname "$0"); T=$1; OUT=$2
pkill -f "[g]z sim"; sleep 1
W=$(CAMPOSE="$CAMPOSE" python3 $D/shot.py $T) || exit 1
setsid nohup gz sim -s -r $W > /tmp/gzshot_$T.log 2>&1 < /dev/null &
sleep 8
python3 $D/grab.py $OUT; rc=$?
pkill -f "[g]z sim"; tail -5 /tmp/gzshot_$T.log; exit $rc
