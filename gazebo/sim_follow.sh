#!/bin/bash
# Closed loop in Gazebo: Prototype A's line follower (learn2race_camera_follower, unchanged) drives the simulated
# robot of make_sim_world.py round a circuit, and the run is recorded from an overhead camera. Run in WSL:
#
#   bash gazebo/sim_follow.sh TRACK SECONDS OUT_DIR [SPEED] [follower args...]
#
# Needs the Prototype A repository (L2R_A; default: Learn2Race-PrototypeA next to this repository, as in the
# Learn2Race main repository) and its follower built in a workspace (default ~/l2r_sim_ws, override with L2R_WS):
#   mkdir -p ~/l2r_sim_ws/src && ln -s $L2R_A/learn2race_ws/src/learn2race_camera_follower ~/l2r_sim_ws/src/
#   cd ~/l2r_sim_ws && colcon build --symlink-install
# Follower arguments of the track come from the "# robot.sh follower:" line of config/track_TRACK.yaml, as on the robot.
TRACK=$1; SECS=${2:-60}; OUT=$3; SPEED=${4:-0.3}; shift 4 2>/dev/null
HERE=$(cd "$(dirname "$0")" && pwd)
L2R_A=${L2R_A:-$(cd "$HERE/../.." && pwd)/Learn2Race-PrototypeA}
WS=${L2R_WS:-~/l2r_sim_ws}
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
# CycloneDDS: Fast DDS (the default) delivers nothing between processes under WSL here; domain 42 and localhost
# keep the simulation apart from the robots (domain 0)
export LIBGL_ALWAYS_SOFTWARE=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

pkill -f "[g]z sim"; pkill -f "[p]arameter_bridge"; pkill -f "[l]ine_follower_node"; sleep 2
python3 "$HERE/make_sim_world.py" "$TRACK" >/dev/null || exit 1
TRACK_CFG="$L2R_A/learn2race_ws/src/learn2race_bringup/config/track_$TRACK.yaml"
FARGS=$( [ -f "$TRACK_CFG" ] && sed -n 's/^# robot.sh follower: //p' "$TRACK_CFG" | tr '\n' ' ')
mkdir -p "$OUT"
echo "track $TRACK, $SECS s, speed $SPEED, follower: $FARGS $*" | tee "$OUT/args.txt"

setsid gz sim -s -r "$HERE/worlds/sim_$TRACK.sdf" > "$OUT/gz.log" 2>&1 < /dev/null &
sleep 6
setsid ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /image_raw@sensor_msgs/msg/Image[gz.msgs.Image > "$OUT/bridge.log" 2>&1 < /dev/null &
sleep 3
setsid ros2 launch learn2race_camera_follower camera_line_follower.launch.py start_camera:=false \
  cmd_vel_topic:=cmd_vel max_linear_speed:=$SPEED snapshot_dir:="$OUT/snaps" $FARGS "$@" > "$OUT/follower.log" 2>&1 < /dev/null &
python3 "$HERE/sim_record.py" "$TRACK" "$OUT/frames" "$SECS" "Prototype A line follower in Gazebo - $TRACK, max $SPEED m/s"
pkill -f "[l]ine_follower_node"; pkill -f "[p]arameter_bridge"; pkill -f "[g]z sim"
grep -E "fps|state=" "$OUT/follower.log" | tail -3
