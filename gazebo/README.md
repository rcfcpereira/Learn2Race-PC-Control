# Gazebo replay of Prototype A races

Gazebo Harmonic worlds of the four events, built from the competition rules at real scale, with a model of Prototype A. `replay_race.py` moves the robot model along a recorded race.

The worlds were built to visualise recorded races. They are **not** used to develop or test the robot: there is no simulated camera or physics-driven drive.

## Files

| File | What it does |
|---|---|
| `make_world.py TRACK` | Writes `worlds/TRACK.sdf` for `monza`, `portimao`, `suzuka` or `dragster`. The floor is the circuit drawing of the 2026 TrackBotGP rules at real scale (20 mm line = 10 px, 2 mm per pixel; Portimão's drawn lap is 8.95 m against 8.959 m measured by rope). The Dragster floor comes from the Dragster rule dimensions. The robot model uses Prototype A's measured dimensions, with its front at the start line |
| `open_world.sh TRACK` | Closes any running Gazebo, generates the world if it is missing, and opens it (detached, so it survives WSL returning) |
| `replay_race.py RUN_DIR TRACK [SPEED]` | Replays a race from the follower's `rec.csv`: wheel speed with the IMU heading, gyro bias removed so the race closes on whole turns |
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
