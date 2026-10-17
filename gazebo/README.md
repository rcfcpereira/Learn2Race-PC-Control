# Gazebo replay of Prototype A races

Gazebo Harmonic worlds of the four events, built from the competition rules at real scale, with a model of Prototype A. `replay_race.py` moves the robot model along a recorded race.

The worlds were built to visualise recorded races. They are **not** used to develop or test the robot: there is no simulated camera or physics-driven drive.

## Files

| File | What it does |
|---|---|
| `make_world.py TRACK` | Writes `worlds/TRACK.sdf` for `monza`, `portimao`, `suzuka` or `dragster`. The floor is the circuit drawing of the 2026 TrackBotGP rules at real scale (20 mm line = 10 px, 2 mm per pixel; Portimão's drawn lap is 8.95 m against 8.959 m measured by rope). The Dragster floor is drawn by `make_dragster_texture.py`. The robot model uses Prototype A's measured dimensions, with its front at the start line |
| `make_dragster_texture.py` | Draws the Dragster floor from the Dragster rules (section IV, Figures 4–5): two 30 cm lanes with 20 mm border and centre stripes, 10 m from the start line to the finish stripe, stopping-zone lines 0.5 m and 1.0 m past it, 0.5 m of floor behind the start and a 2 m run-out (0.80 × 12.50 m). The Dragster world adds the start-light gantry of Figure 6 (about 70 × 30 cm, a four-LED light over each lane) over the start line, with Robot A in the left lane |
| `open_world.sh TRACK` | Closes any running Gazebo, generates the world if it is missing, and opens it (detached, so it survives WSL returning) |
| `replay_race.py RUN_DIR TRACK [SPEED]` | Replays a race from the follower's `rec.csv`: wheel speed with the IMU heading, gyro bias removed so the race closes on whole turns |
| `render/run.sh TRACK OUT.png` | Renders a world headless from a fixed camera and saves one image (track figures); `CAMPOSE="x y z roll pitch yaw"` moves the Dragster camera |
| `media/track_*.png` | Floor textures |
| `sample_runs/portimao_20261013-203303/` | A recorded Portimão race (138.8 s, three laps) to try the replay |

## Run (WSL, Ubuntu 24.04)

```bash
cd /mnt/c/<path to Learn2Race-PC-Control>
bash gazebo/open_world.sh portimao
source /opt/ros/jazzy/setup.bash
python3 gazebo/replay_race.py gazebo/sample_runs/portimao_20261013-203303 portimao 2   # twice real time
```

From Windows (Git Bash or PowerShell):

```bash
wsl -d Ubuntu-24.04 -- bash /mnt/c/<path to Learn2Race-PC-Control>/gazebo/open_world.sh portimao
```

In Git Bash, prefix the command with `MSYS_NO_PATHCONV=1` so the `/mnt/c` path is not rewritten.

Any run folder copied from the robot (`~/run_logs/<date-time>/`, recorded with `record_every_n`) can be replayed the same way.

## Limits

The replayed path is dead reckoning, not localisation. It keeps the shape of the race but drifts from the drawn line over a lap, because of the integrated gyro's scale error and wheel slip during line searches.

## Closed loop: Prototype A's line follower driving a simulated robot

`sim_follow.sh TRACK SECONDS OUT_DIR [SPEED] [follower args]` runs the robot's own `line_follower_node`
(package `learn2race_camera_follower` of Learn2Race-PrototypeA, unchanged) against a simulated robot, and films it.

- `make_sim_world.py` turns a track world into a driven one: Prototype A's wheels (64.9 mm on a 180.5 mm track),
  a frictionless castor, about 1.0 kg, a differential drive on `/cmd_vel` and `/odom`, and the line camera copied
  from the real one (IMX219, 320 × 240 px, 62.2°, lens 143 mm ahead of the axle and 65 mm up, 53° below
  horizontal) on `/image_raw`. A fixed camera films the GP circuits from above; on the Dragster a camera follows
  behind the robot.
- `ros_gz_bridge` connects `/image_raw`, `/cmd_vel` and `/odom`. The follower takes the track's arguments from
  `config/track_TRACK.yaml` of Prototype A, as on the robot. CycloneDDS, ROS domain 42 and localhost discovery
  keep the simulation apart from the robots (Fast DDS delivered nothing between processes under WSL).
- `sim_record.py` saves the frames with the model's true pose (`frames/pose.csv`); `encode_video.py` makes an MP4.

Set-up (WSL, once):

```bash
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-cv-bridge
export L2R_A=/mnt/c/<path>/Learn2Race-PrototypeA      # default: the sibling folder in the Learn2Race repository
mkdir -p ~/l2r_sim_ws/src && ln -s $L2R_A/learn2race_ws/src/learn2race_camera_follower ~/l2r_sim_ws/src/
cd ~/l2r_sim_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```

Runs (speeds as raced: 0.5 m/s on the GP circuits, 1.0 m/s on the Dragster):

```bash
bash gazebo/sim_follow.sh portimao 210 /tmp/sim_portimao 0.5
bash gazebo/sim_follow.sh dragster 40 /tmp/sim_dragster 1.0 race_mode:=dragster
python gazebo/encode_video.py //wsl.localhost/Ubuntu-24.04/tmp/sim_portimao/frames sim_portimao.mp4 17   # Windows
```

Results of the first runs (17 October 2026): Monza about six clean laps; Portimão about four laps, cutting inside
the tightest hairpin, two line losses; Suzuka lost at the crossing and the hairpins, as on the real track;
Dragster 10 m at 1.0 m/s in about 10.5 s, slowing to a stop at the finish stripe.

What the simulation leaves out: the drive is ideal (no motor lag), the floor is uniform (no glare, seams or track
light), the competition nodes (referee, state machine, pit stop, the Dragster stop) do not run, and the camera
renders at what the PC manages (about 30 Hz against the real 41.8 fps). It shows the detector and the steering in
closed loop on the real track geometry, not race performance.

## Replay video of a recorded race

`video_race.py RUN_DIR TRACK OUT_DIR [SPEED] [FPS]` renders a recorded race with the robot placed on the drawn
line by `track_route.py`: the recorded distance and timing of each lap (`rec.csv`, `referee.log`) are scaled
onto the drawn centreline, so speed changes, line searches and the pit stop keep their real timing. The trail is
coloured by lap. `sample_runs/portimao_20261013-203303/` has everything it needs.
