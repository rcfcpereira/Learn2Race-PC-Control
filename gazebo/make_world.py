"""Gazebo (Harmonic) world of a TrackBotGP circuit with Prototype A, for replaying recorded races.

The floor is the circuit drawing of the 2026 rules (Figures 4-6) at real scale: the drawings are embedded
images where the 20 mm line is 10 px wide, i.e. 2 mm per pixel (Portimao's drawn lap 8.95 m against 8.959 m by rope). Robot A is built from its measured dimensions
(measured October 2026): envelope 180 x 206 x 158 mm, wheels 64.9 mm on a 180.5 mm track,
drive axle 37 mm from the back, a 8.94 mm castor ball 128 mm from the back, the camera board at the front.

    python gazebo/make_world.py TRACK      (monza | portimao | suzuka | dragster; writes worlds/TRACK.sdf)
"""
# References (full entries: docs/references.md at the repository root):
#   [R1] ISEP, TrackBotGP competition rules (2026 Robotics@ISEP Open): track figures
#   Gazebo Harmonic, SDFormat 1.9 specification
import os
import sys

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
MM_PX = 2.0                      # mm per drawing pixel (20 mm line = 10 px)
# the finish line on the main straight, in drawing pixels (x, y), and the start heading (up the drawing)
FINISH_PX = {"monza": (107, 840), "portimao": (109, 804), "suzuka": (116, 409),
             # dragster (texture from the rules dimensions): left lane centre, just behind the start line
             "dragster": (100, 5500)}            # the start line (400 mm from the near end)


def wsl_path(p):
    """Gazebo runs in WSL: a Windows drive path becomes /mnt/<drive>/... (unchanged on Linux)."""
    p = os.path.abspath(p)
    if len(p) > 1 and p[1] == ":":
        p = "/mnt/" + p[0].lower() + p[2:].replace(os.sep, "/")
    return p


def robot_model(pose=(0.0, 0.0, 0.0)):
    L, W, H = 0.180, 0.206, 0.158
    r, track = 0.06490 / 2, 0.1805
    back = -0.037                                    # base_link (drive axle) is 37 mm in front of the back
    return f"""
    <model name="prototype_a">
      <static>true</static>
      <pose>{pose[0]:.4f} {pose[1]:.4f} 0 0 0 {pose[2]:.4f}</pose>
      <link name="base_link">
        <visual name="body">
          <pose>{back + L / 2:.4f} 0 {r + 0.035:.4f} 0 0 0</pose>
          <geometry><box><size>{L - 0.03:.3f} {track - 0.03:.3f} 0.05</size></box></geometry>
          <material><diffuse>0.15 0.35 0.75 1</diffuse></material>
        </visual>
        <visual name="top_plate">
          <pose>{back + L / 2 - 0.015:.4f} 0 {H - 0.04:.4f} 0 0 0</pose>
          <geometry><box><size>0.14 0.15 0.004</size></box></geometry>
          <material><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
        <visual name="lidar">
          <pose>{back + 0.075:.4f} 0 {H - 0.02:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>0.019</radius><length>0.035</length></cylinder></geometry>
          <material><diffuse>0.05 0.05 0.05 1</diffuse></material>
        </visual>
        <visual name="camera_board">
          <pose>0.143 0 0.065 0 0.52 0</pose>
          <geometry><box><size>0.004 0.025 0.024</size></box></geometry>
          <material><diffuse>0.1 0.55 0.2 1</diffuse></material>
        </visual>
        <visual name="wheel_left">
          <pose>0 {track / 2:.4f} {r:.4f} 1.5708 0 0</pose>
          <geometry><cylinder><radius>{r:.4f}</radius><length>0.0255</length></cylinder></geometry>
          <material><diffuse>0.08 0.08 0.08 1</diffuse></material>
        </visual>
        <visual name="wheel_right">
          <pose>0 {-track / 2:.4f} {r:.4f} 1.5708 0 0</pose>
          <geometry><cylinder><radius>{r:.4f}</radius><length>0.0255</length></cylinder></geometry>
          <material><diffuse>0.08 0.08 0.08 1</diffuse></material>
        </visual>
        <visual name="castor">
          <pose>{back + 0.128:.4f} 0 0.0045 0 0 0</pose>
          <geometry><sphere><radius>0.0045</radius></sphere></geometry>
          <material><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
        <visual name="led">
          <pose>{back + 0.02:.4f} 0 {H - 0.035:.4f} 0 0 0</pose>
          <geometry><box><size>0.01 0.12 0.008</size></box></geometry>
          <material><diffuse>0.1 0.3 1 1</diffuse><emissive>0.1 0.3 1 1</emissive></material>
        </visual>
      </link>
    </model>"""


def world(track):
    tex = os.path.join(HERE, "media", f"track_{track}.png")
    h_px, w_px = cv2.imread(tex, cv2.IMREAD_GRAYSCALE).shape
    w_m, h_m = w_px * MM_PX / 1000, h_px * MM_PX / 1000
    # world frame: origin at the drawing's bottom-left corner, x right, y up the drawing
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{track}">
    <physics name="1ms" type="ignored"><max_step_size>0.01</max_step_size><real_time_factor>1</real_time_factor></physics>
    <!-- the physics system applies set_pose commands (the replay moves the static robot model) -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <scene><ambient>0.7 0.7 0.7 1</ambient><background>0.85 0.88 0.92 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.3 0.2 -1</direction>
    </light>
    <model name="track">
      <static>true</static>
      <pose>{w_m / 2:.4f} {h_m / 2:.4f} 0 0 0 0</pose>
      <link name="floor">
        <visual name="drawing">
          <geometry><plane><normal>0 0 1</normal><size>{w_m:.4f} {h_m:.4f}</size></plane></geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>file://{wsl_path(tex)}</albedo_map></metal></pbr>
          </material>
        </visual>
      </link>
    </model>{robot_model(start_pose(track, h_m))}
  </world>
</sdf>
""", (w_m, h_m)


def start_pose(track, h_m):
    fx, fy = FINISH_PX[track]
    # no part of the robot may cross the start line (rules II.6b): the camera board, the robot's front,
    # is 143 mm ahead of the drive axle, so the axle starts 0.15 m behind the line, facing up the drawing
    return fx * MM_PX / 1000, h_m - fy * MM_PX / 1000 - 0.15, 1.5708


if __name__ == "__main__":
    name = sys.argv[1]
    text, (w_m, h_m) = world(name)
    os.makedirs(os.path.join(HERE, "worlds"), exist_ok=True)
    out = os.path.join(HERE, "worlds", f"{name}.sdf")
    open(out, "w", encoding="utf-8", newline="\n").write(text)
    print(out, f"{w_m:.2f} x {h_m:.2f} m; start", start_pose(name, h_m))
