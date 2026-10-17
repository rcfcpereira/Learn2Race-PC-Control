"""Record a closed-loop run (sim_follow.sh): every overhead image with the robot's true pose, drawn as a trail.

    python3 sim_record.py TRACK OUT_DIR SECONDS [TITLE]

Writes OUT_DIR/frame_00000.png ... (10 Hz, the overhead camera's rate) and OUT_DIR/pose.csv (t, x, y, yaw, the
model pose from Gazebo, i.e. ground truth, not odometry).
"""
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_world  # noqa: E402

W, H, Z = 960, 540, 4.0


def main():
    track, out, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
    title = sys.argv[4] if len(sys.argv) > 4 else ""
    os.makedirs(out, exist_ok=True)
    tex = cv2.imread(os.path.join(make_world.HERE, "media", f"track_{track}.png"), cv2.IMREAD_GRAYSCALE)
    w, h = tex.shape[1] * make_world.MM_PX / 1000, tex.shape[0] * make_world.MM_PX / 1000
    cx, cy = w / 2, h / 2
    span = max(h * 1.12, w * 1.12 * W / H)
    f = (W / 2) / math.tan(math.atan(span / 2 / Z))

    def to_px(x, y):
        return int(round(W / 2 + f * (y - cy) / Z)), int(round(H / 2 + f * (x - cx) / Z))

    lock = threading.Lock()
    state = {"pose": None, "v": 0.0, "img": None, "n": 0}

    def on_pose(m):
        for p in m.pose:
            if p.name == "prototype_a":
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                with lock:
                    state["pose"] = (p.position.x, p.position.y, yaw)

    def on_odom(m):
        with lock:
            state["v"] = m.twist.linear.x

    def on_img(m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
        with lock:
            state["img"], state["n"] = cv2.cvtColor(a, cv2.COLOR_RGB2BGR), state["n"] + 1

    node = Node()
    node.subscribe(Pose_V, f"/world/{track}/dynamic_pose/info", on_pose)
    node.subscribe(Odometry, "/odom", on_odom)
    node.subscribe(Image, "/overhead", on_img)
    t0, n_seen, frame, trail, dist, last = time.time(), 0, 0, [], 0.0, None
    log = open(os.path.join(out, "pose.csv"), "w")
    log.write("t,x,y,yaw,v\n")
    while time.time() - t0 < secs:
        with lock:
            n, img, pose, v = state["n"], state["img"], state["pose"], state["v"]
        if n == n_seen or img is None or pose is None:
            time.sleep(0.005)
            continue
        n_seen = n
        t = time.time() - t0
        x, y, yaw = pose
        if last is not None:
            dist += math.hypot(x - last[0], y - last[1])
        last = (x, y)
        log.write(f"{t:.3f},{x:.4f},{y:.4f},{yaw:.4f},{v:.3f}\n")
        if track != "dragster":
            trail.append(to_px(x, y))
        else:                                         # the Dragster is filmed from behind the robot
            cam = Pose()
            cam.name = "chase_camera"
            cam.position.x, cam.position.y, cam.position.z = x - 0.55 * math.cos(yaw), y - 0.55 * math.sin(yaw), 0.40
            # roll 0, pitch 0.42 (down), yaw: the robot's heading
            cy_, sy_, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(0.21), math.sin(0.21)
            cam.orientation.w, cam.orientation.x = cy_ * cp, -sy_ * sp
            cam.orientation.y, cam.orientation.z = cy_ * sp, sy_ * cp
            node.request(f"/world/{track}/set_pose", cam, Pose, Boolean, 500)
        img = img.copy()
        if len(trail) > 1:
            cv2.polylines(img, [np.array(trail, np.int32)], False, (40, 120, 255), 2, cv2.LINE_AA)
        lines = [title, f"t = {t:6.1f} s    speed {v:4.2f} m/s    driven {dist:5.2f} m",
                 "camera image -> line_follower_node (Prototype A code, unchanged) -> /cmd_vel -> simulated drive"]
        for i, line in enumerate(lines):
            if line:
                yy, sc = 26 + 24 * i, (0.6 if i < 2 else 0.45)
                cv2.putText(img, line, (14, yy), cv2.FONT_HERSHEY_SIMPLEX, sc, (255, 255, 255), 4, cv2.LINE_AA)
                cv2.putText(img, line, (14, yy), cv2.FONT_HERSHEY_SIMPLEX, sc, (30, 30, 30), 1, cv2.LINE_AA)
        cv2.imwrite(os.path.join(out, f"frame_{frame:05d}.png"), img)
        frame += 1
    log.close()
    print(f"recorded {frame} frames over {secs:.0f} s, driven {dist:.2f} m")


if __name__ == "__main__":
    main()
