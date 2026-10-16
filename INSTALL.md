# Installation guide – PC control

Tested on Windows 11 with WSL 2 (Ubuntu 24.04). A native Ubuntu 24.04 PC works the same way: skip the WSL sections.

## 1. Gamepad tool (Windows only)

Install [Python 3](https://www.python.org/downloads/) (3.10 or later). `gamepad_udp.py` uses only the standard library and Windows' XInput, so nothing else is needed.

```powershell
git clone https://github.com/rcfcpereira/Learn2Race-PC-Control.git
cd Learn2Race-PC-Control
python teleop\gamepad_udp.py 192.168.8.112
```

Connect the PC to the robots' network (the TL-WR802N router, `192.168.8.0/24`).

## 2. WSL with ROS 2 Jazzy (for RViz and Gazebo)

1. Install Ubuntu 24.04 in WSL, from an administrator PowerShell:

   ```powershell
   wsl --install -d Ubuntu-24.04
   ```

2. Let WSL share the PC's network, so DDS discovery reaches the robots. Create or edit `%USERPROFILE%\.wslconfig`:

   ```ini
   [wsl2]
   networkingMode=mirrored
   ```

   Then run `wsl --shutdown` and start Ubuntu again.

3. Allow ROS 2 (DDS) traffic from the robot network into WSL, from an administrator PowerShell:

   ```powershell
   New-NetFirewallHyperVRule -Name 'L2R-ROS-DDS' -DisplayName 'Learn2Race ROS 2 DDS from robot LAN' `
     -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' `
     -Protocol UDP -RemoteAddresses 192.168.8.0/24 -Action Allow
   ```

4. Inside Ubuntu, install ROS 2 Jazzy desktop (official guide: <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>), then the extra packages:

   ```bash
   sudo apt install -y ros-jazzy-desktop ros-jazzy-rmw-cyclonedds-cpp \
     ros-jazzy-ros-gz python3-opencv
   ```

   `ros-jazzy-ros-gz` brings Gazebo Harmonic (gz-sim 8) and its Python bindings (`gz.transport13`, `gz.msgs10`), which the replay uses.

5. Check that the robot is visible (with a robot running):

   ```bash
   source /opt/ros/jazzy/setup.bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
   ros2 topic list            # should list /scan, /odom, /map ...
   ```

## Notes

- **RViz or Gazebo shows a black window or crashes on start:** WSLg's GPU driver fails on some shaders. The scripts already set `LIBGL_ALWAYS_SOFTWARE=1`.
- **Gazebo closes as soon as the command returns:** WSL ends background processes when the calling shell exits. `open_world.sh` detaches Gazebo and waits 20 s.
- **Paths from Git Bash:** prefix `wsl` commands with `MSYS_NO_PATHCONV=1`, or Git Bash rewrites `/mnt/c/…`.
