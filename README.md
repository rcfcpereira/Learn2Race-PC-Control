# Learn2Race – PC control

The tools that run on the operator's PC for the two Learn2Race robots:
- driving Prototype B with a gamepad, without ROS on the PC;
- watching the SLAM map in RViz;
- replaying recorded races of Prototype A in Gazebo.

The robots run on their own. Nothing here is needed during a race.

| Folder | Tool | Runs on |
|---|---|---|
| [`teleop/`](teleop/) | `gamepad_udp.py`: Xbox-style pad → UDP → robot | Windows, plain Python |
| [`rviz/`](rviz/) | `rviz_b.sh` + `slam_view.rviz`: live map, scan and robot pose of Prototype B | WSL (Ubuntu 24.04) with ROS 2 Jazzy |
| [`gazebo/`](gazebo/README.md) | Track worlds and race replay for Prototype A | WSL with Gazebo Harmonic |
| [`docs/references.md`](docs/references.md) | References cited in the code headers as `[Rnn]` | |

Installation: [INSTALL.md](INSTALL.md).

## Gamepad driving (Prototype B)

1. On the robot, start the base and the UDP receiver:

   ```bash
   ~/robot.sh base            # or: ~/robot.sh slam
   python3 ~/Learn2Race-PrototypeB/tools/udp_teleop.py
   ```

2. On the PC, with the pad connected by USB cable or its receiver:

   ```bash
   python teleop/gamepad_udp.py 192.168.8.112      # the robot's address
   ```

| Control | Action |
|---|---|
| LB (hold) | Dead-man switch: the robot moves only while it is held |
| LB + RB | Faster (0.25 m/s instead of 0.15 m/s) |
| Left stick, up/down | Forward and back |
| Right stick, left/right | Turn (up to 0.8 rad/s; turn slowly when mapping) |
| Ctrl+C | Quit (the robot stops within 0.5 s) |

Commands are sent at 20 Hz. The stick-to-wheel latency is about 130 ms (median, measured from 120 fps video).

The pad is read through Windows XInput. Bluetooth pads that do not register as XInput devices will not work: connect them by cable.

## Live map in RViz (Prototype B)

With `~/robot.sh slam` running on the robot:

```bash
wsl -d Ubuntu-24.04 -- bash /mnt/c/<path to Learn2Race-PC-Control>/rviz/rviz_b.sh
```

## Race replay in Gazebo (Prototype A)

See [gazebo/README.md](gazebo/README.md). A recorded Portimão race is included for trying it.

## ROS on the PC (optional)

A PC with ROS 2 Jazzy can also use the launch files kept in the robot repositories:
- `teleop_pc.launch.py`: a gamepad on the PC publishing `teleop/cmd_vel`;
- `remote_pc_joy.launch.py`.

Use the same `ROS_DOMAIN_ID` as the robot (0). Run only one robot on the network at a time.
