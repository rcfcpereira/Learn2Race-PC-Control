"""Replay a recorded race of Prototype A in Gazebo (Harmonic): run in WSL with the world open.

The path is wheel speed (odom_v) with the IMU heading (imu_wz
integrated, the constant gyro bias removed so the race closes on whole turns), from the follower's
rec.csv (every frame). It is placed on the circuit drawing at the finish line, facing up the main
straight (make_world.start_pose), and the model prototype_a is moved along it at real time (x SPEED).

    source /opt/ros/jazzy/setup.bash
    bash gazebo/open_world.sh portimao
    python3 gazebo/replay_race.py gazebo/sample_runs/portimao_20261013-203303 portimao [SPEED]
"""
# References (full entries: docs/references.md at the repository root):
#   [R44] Thrun et al., Probabilistic Robotics: dead reckoning
#   Gazebo Harmonic, gz-transport 13 / gz-msgs 10 Python bindings
import csv
import math
import os
import sys
import time

import cv2
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_world  # noqa: E402


def race_path(run_dir):
    rec = os.path.join(run_dir, "snaps", "rec.csv")
    if not os.path.exists(rec):
        rec = os.path.join(run_dir, "rec.csv")
    rows = [r for r in csv.DictReader(open(rec)) if r.get("imu_wz")]
    t = [float(r["t"]) for r in rows]
    v = [float(r["odom_v"]) for r in rows]
    wz = [float(r["imu_wz"]) for r in rows]
    # moving part only: from the first to the last sample with speed
    moving = [i for i, s in enumerate(v) if abs(s) > 0.01]
    a, b = moving[0], moving[-1] + 1
    t, v, wz = t[a:b], v[a:b], wz[a:b]
    th, x, y, out = 0.0, 0.0, 0.0, []
    total = sum(w * (t[i] - t[i - 1]) for i, w in enumerate(wz) if i)
    turns = round(total / (2 * math.pi))
    bias = (total - turns * 2 * math.pi) / (t[-1] - t[0])
    for i in range(len(t)):
        dt = t[i] - t[i - 1] if i else 0.0
        th += (wz[i] - bias) * dt
        x += v[i] * math.cos(th) * dt
        y += v[i] * math.sin(th) * dt
        out.append((t[i] - t[0], x, y, th))
    return out, turns, bias


def main():
    run_dir, track = sys.argv[1], sys.argv[2]
    speed = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    h_px, _ = cv2.imread(os.path.join(make_world.HERE, "media", f"track_{track}.png"), 0).shape
    sx, sy, syaw = make_world.start_pose(track, h_px * make_world.MM_PX / 1000)
    path, turns, bias = race_path(run_dir)
    print(f"{len(path)} poses, {path[-1][0]:.1f} s, {turns} turns, gyro bias removed {1000 * bias:+.2f} mrad/s")
    node = Node()
    service = f"/world/{track}/set_pose"
    c, s = math.cos(syaw), math.sin(syaw)
    t0 = time.monotonic()
    for k, (t, x, y, th) in enumerate(path):
        while time.monotonic() - t0 < t / speed:
            time.sleep(0.002)
        if k % 2:                      # ~20 updates a second are plenty
            continue
        p = Pose()
        p.name = "prototype_a"
        p.position.x, p.position.y, p.position.z = sx + c * x - s * y, sy + s * x + c * y, 0.0
        yaw = syaw + th
        p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        node.request(service, p, Pose, Boolean, 200)
    print("done")


if __name__ == "__main__":
    main()
