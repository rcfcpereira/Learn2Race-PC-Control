"""Write a copy of a track world with a fixed camera added (used by run.sh): shot.py TRACK -> prints the SDF path"""
import math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import make_world

track = sys.argv[1]
text, (w, h) = make_world.world(track)
cx, cy = w / 2, h / 2
dist, height = 1.45, 3.0
CAM = {"dragster": os.environ.get("CAMPOSE") or "0.40 -0.70 0.60 0 0.36 1.5708"}
W, H, FOV = (1800, 1200, 1.0) if track == "dragster" else (1500, 1800, 0.62)
if os.environ.get("CAMSIZE"):
    W, H, FOV = (float(v) for v in os.environ["CAMSIZE"].split())
    W, H = int(W), int(H)
pose = CAM.get(track, f"{cx:.3f} {cy - dist:.3f} {height:.3f} 0 {math.atan2(height, dist):.3f} 1.5708")
cam = f"""
    <model name="appe_camera"><static>true</static>
      <pose>{pose}</pose>
      <link name="l"><sensor name="cam" type="camera">
        <topic>appe_cam</topic><update_rate>5</update_rate><always_on>1</always_on>
        <camera><horizontal_fov>{FOV}</horizontal_fov><image><width>{W}</width><height>{H}</height></image>
        <clip><near>0.05</near><far>400</far></clip></camera>
      </sensor></link></model>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  </world>"""
text = text.replace("  </world>", cam)
# brighter, top-down light for a print figure
text = text.replace("<ambient>0.7 0.7 0.7 1</ambient>", "<ambient>1 1 1 1</ambient>")
text = text.replace("<diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.3 0.2 -1</direction>", "<diffuse>1 1 1 1</diffuse><direction>-0.15 0.1 -1</direction>")
text = text.replace("<background>0.85 0.88 0.92 1</background>", "<background>1 1 1 1</background>")
path = f"/tmp/appe_{track}.sdf"
open(path, "w").write(text)
print(path)
