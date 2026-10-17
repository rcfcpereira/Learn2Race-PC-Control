"""Video of a recorded race replayed in Gazebo: a fixed camera straight above the track, the robot model moved
along the race route (track_route.race_route: the recorded distance and timing of each lap, from rec.csv and
referee.log, placed on the drawn line), one rendered image per video frame, with the route so far drawn as a
trail coloured by lap and the race time, lap and distance written on it.

Run in WSL (Gazebo Harmonic, gz-transport 13); frames go to OUT_DIR/frame_00000.png ..., then encode them:

    python3 gazebo/video_race.py gazebo/sample_runs/portimao_20261013-203303 portimao /tmp/frames 2 20
    (Windows) python gazebo/encode_video.py OUT_DIR video.mp4 FPS

The robot stays on the drawn line (the real axle cuts inside the bends); speed changes, line searches and the
pit stop keep their recorded timing. The video says so.
"""
# References (full entries: docs/references.md at the repository root):
#   [R44] Thrun et al., Probabilistic Robotics: dead reckoning
#   Gazebo Harmonic, gz-transport 13 / gz-msgs 10 Python bindings
import math
import os
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_world  # noqa: E402
import track_route  # noqa: E402

W, H, Z = 1280, 720, 4.0                  # image size (px), camera height above the floor (m)
TITLES = {"portimao": "Portimao", "monza": "Monza", "suzuka": "Suzuka", "dragster": "Dragster"}


class Camera:
    def __init__(self, node, topic):
        self.lock, self.count, self.img = threading.Lock(), 0, None
        node.subscribe(Image, topic, self._cb)

    def _cb(self, m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
        with self.lock:
            self.img, self.count = cv2.cvtColor(a, cv2.COLOR_RGB2BGR), self.count + 1

    def next_after(self, n, timeout=30.0):
        t = time.time()
        while time.time() - t < timeout:
            with self.lock:
                if self.count >= n:
                    return self.img.copy(), self.count
            time.sleep(0.005)
        raise RuntimeError("no camera image")


def main():
    run_dir, track, out = sys.argv[1], sys.argv[2], sys.argv[3]
    speed = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
    fps = float(sys.argv[5]) if len(sys.argv) > 5 else 20.0
    os.makedirs(out, exist_ok=True)

    text, (w, h) = make_world.world(track)
    cx, cy = w / 2, h / 2
    # straight down, image right = world +y (the long side of the drawing), image down = world +x
    span = max(h * 1.12, w * 1.12 * W / H)
    hfov = 2 * math.atan(span / 2 / Z)
    f = (W / 2) / math.tan(hfov / 2)
    cam = f"""
    <model name="video_camera"><static>true</static>
      <pose>{cx:.4f} {cy:.4f} {Z} 0 1.5708 3.14159</pose>
      <link name="l"><sensor name="cam" type="camera">
        <topic>video_cam</topic><update_rate>30</update_rate><always_on>1</always_on>
        <camera><horizontal_fov>{hfov:.5f}</horizontal_fov><image><width>{W}</width><height>{H}</height></image>
        <clip><near>0.05</near><far>20</far></clip></camera>
      </sensor></link></model>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  </world>"""
    text = text.replace("  </world>", cam)
    text = text.replace("<ambient>0.7 0.7 0.7 1</ambient>", "<ambient>1 1 1 1</ambient>")
    text = text.replace("<diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.3 0.2 -1</direction>",
                        "<diffuse>1 1 1 1</diffuse><direction>-0.15 0.1 -1</direction>")
    sdf = f"/tmp/video_{track}.sdf"
    open(sdf, "w").write(text)

    def to_px(x, y):                       # world floor point -> image pixel (pinhole, straight down)
        return int(round(W / 2 + f * (y - cy) / Z)), int(round(H / 2 + f * (x - cx) / Z))

    world, main, pit = track_route.race_route(run_dir, track, make_world.FINISH_PX[track])
    ev = track_route.referee_events(run_dir)
    t_pit, t_rel = ev["pit"] - ev["start"], ev["release"] - ev["start"]
    dist = [0.0]
    for a, b in zip(world, world[1:]):
        dist.append(dist[-1] + math.hypot(b[1] - a[1], b[2] - a[2]))

    env = dict(os.environ, LIBGL_ALWAYS_SOFTWARE="1")
    gz = subprocess.Popen(["gz", "sim", "-s", "-r", sdf], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                          env=env, start_new_session=True)
    try:
        node = Node()
        cam_in = Camera(node, "/video_cam")
        _, n = cam_in.next_after(1, timeout=60)
        service = f"/world/{track}/set_pose"
        trails, k, frame = {1: [], 2: [], 3: []}, 0, 0
        t_end = world[-1][0] + 1.0
        label = os.path.basename(os.path.normpath(run_dir)).replace("race_", "")
        while True:
            t = frame * speed / fps
            if t > t_end:
                break
            while k + 1 < len(world) and world[k + 1][0] <= t - 1.0:
                k += 1
                _, wx, wy, wa, wl = world[k]
                o = {1: -0.011, 2: 0.0, 3: 0.011}[wl]        # laps side by side: about 4 px apart
                trails[wl].append(to_px(wx - math.sin(wa) * o, wy + math.cos(wa) * o))
            tr, x, y, yaw, lap = world[k]
            if os.path.exists(os.path.join(out, f"frame_{frame:05d}.png")):    # resume an interrupted render
                frame += 1
                continue
            p = Pose()
            p.name = "prototype_a"
            p.position.x, p.position.y, p.position.z = x, y, 0.0
            p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            node.request(service, p, Pose, Boolean, 2000)
            img, n = cam_in.next_after(n + 2)              # a frame rendered after the move
            for n_lap, colour in ((1, (40, 120, 255)), (2, (60, 180, 60)), (3, (220, 90, 40))):
                if len(trails[n_lap]) > 1:
                    cv2.polylines(img, [np.array(trails[n_lap], np.int32)], False, colour, 2, cv2.LINE_AA)
            state = "PIT STOP" if t_pit <= tr <= t_rel else ("FINISHED" if tr >= world[-1][0] - 0.05 else "")
            lines = [f"Prototype A - {TITLES.get(track, track)} race {label} (Gazebo replay)",
                     f"t = {max(tr, 0):6.1f} s    lap {lap}/3    {dist[k]:5.2f} m    x{speed:g} speed    {state}",
                     "recorded distance and timing of each lap, placed on the drawn line; trail: lap 1 orange, 2 green, 3 blue"]
            for i, line in enumerate(lines):
                yy = 30 + 28 * i
                cv2.putText(img, line, (20, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.7 if i < 2 else 0.55, (255, 255, 255), 4, cv2.LINE_AA)
                cv2.putText(img, line, (20, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.7 if i < 2 else 0.55, (30, 30, 30), 1, cv2.LINE_AA)
            cv2.imwrite(os.path.join(out, f"frame_{frame:05d}.png"), img)
            if frame % 50 == 0:
                print(f"frame {frame}  t={t:.1f}/{t_end:.1f} s", flush=True)
            frame += 1
        print(f"done: {frame} frames")
    finally:
        os.killpg(gz.pid, 15)


if __name__ == "__main__":
    main()
