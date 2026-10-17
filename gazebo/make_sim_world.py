"""Gazebo (Harmonic) world in which Prototype A's own software drives a simulated robot round a circuit.

The track floor is the same drawing as make_world.py (with a collision plane). The robot is a dynamic model
with Prototype A's measured geometry: two driven wheels of 64.9 mm on a 180.5 mm track, a frictionless castor
ball 91 mm ahead of the axle, about 1.0 kg; a differential-drive system takes /cmd_vel and publishes /odom.
The line camera copies the real one (measured October 2026): an IMX219 giving
320 x 240 px over a 62.2 deg field, lens 143 mm ahead of the axle and 65 mm above the floor, axis 53 deg below
the horizontal, published on /image_raw. An optional overhead camera (/overhead) films the run.

    python gazebo/make_sim_world.py TRACK    (writes worlds/sim_TRACK.sdf; run with sim_follow.sh)

Differences from the real robot, to keep in mind: the drive is ideal (no motor lag or wheel slip), the floor is
uniform (no glare, no seams), and there is no track light; the camera renders at what the PC can manage.
"""
# References (full entries: docs/references.md at the repository root):
#   Gazebo Harmonic: DiffDrive, Sensors (camera) systems; SDFormat 1.9
#   [R66] Sony IMX219 / Raspberry Pi Camera v2 (62.2 x 48.8 deg field)
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_world  # noqa: E402

R, TRACK_W = 0.06490 / 2, 0.1805          # wheel radius, wheel centres (m)
CAM_X, CAM_Z, CAM_PITCH = 0.143, 0.065, math.radians(53.0)
CAM_HFOV = math.radians(62.2)


def box_inertia(m, x, y, z):
    return (f"<inertial><mass>{m}</mass><inertia><ixx>{m * (y * y + z * z) / 12:.6g}</ixx>"
            f"<iyy>{m * (x * x + z * z) / 12:.6g}</iyy><izz>{m * (x * x + y * y) / 12:.6g}</izz>"
            "<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>")


def robot(pose, cam_rate=30):
    x, y, yaw = pose

    wheel = lambda side, sy: f"""
      <link name="{side}_wheel">
        <pose>0 {sy:.4f} {R:.5f} -1.5708 0 0</pose>
        <inertial><mass>0.05</mass><inertia><ixx>1.6e-5</ixx><iyy>1.6e-5</iyy><izz>2.6e-5</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="c"><geometry><cylinder><radius>{R:.5f}</radius><length>0.0255</length></cylinder></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><cylinder><radius>{R:.5f}</radius><length>0.0255</length></cylinder></geometry>
          <material><diffuse>0.08 0.08 0.08 1</diffuse></material></visual>
      </link>
      <joint name="{side}_wheel_joint" type="revolute">
        <parent>base_link</parent><child>{side}_wheel</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>"""
    return f"""
    <model name="prototype_a">
      <pose>{x:.4f} {y:.4f} 0.001 0 0 {yaw:.4f}</pose>
      <link name="base_link">
        <pose>0 0 0 0 0 0</pose>
        <inertial><pose>0.03 0 0.07 0 0 0</pose><mass>0.92</mass>
          <inertia><ixx>0.004</ixx><iyy>0.004</iyy><izz>0.005</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="castor"><pose>0.091 0 0.0045 0 0 0</pose>
          <geometry><sphere><radius>0.0045</radius></sphere></geometry>
          <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>
        <visual name="body"><pose>0.053 0 0.065 0 0 0</pose>
          <geometry><box><size>0.15 0.15 0.05</size></box></geometry>
          <material><diffuse>0.15 0.35 0.75 1</diffuse></material></visual>
        <visual name="top_plate"><pose>0.038 0 0.118 0 0 0</pose>
          <geometry><box><size>0.14 0.15 0.004</size></box></geometry>
          <material><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
        <visual name="lidar"><pose>0.038 0 0.138 0 0 0</pose>
          <geometry><cylinder><radius>0.019</radius><length>0.035</length></cylinder></geometry>
          <material><diffuse>0.05 0.05 0.05 1</diffuse></material></visual>
        <visual name="camera_board"><pose>{CAM_X} 0 {CAM_Z} 0 {CAM_PITCH:.4f} 0</pose>
          <geometry><box><size>0.004 0.025 0.024</size></box></geometry>
          <material><diffuse>0.1 0.55 0.2 1</diffuse></material></visual>
        <sensor name="line_camera" type="camera">
          <pose>{CAM_X + 0.004} 0 {CAM_Z} 0 {CAM_PITCH:.4f} 0</pose>
          <topic>/image_raw</topic><update_rate>{cam_rate}</update_rate><always_on>1</always_on>
          <camera><horizontal_fov>{CAM_HFOV:.4f}</horizontal_fov>
            <image><width>320</width><height>240</height><format>R8G8B8</format></image>
            <clip><near>0.02</near><far>3</far></clip></camera>
        </sensor>
      </link>{wheel("left", TRACK_W / 2)}{wheel("right", -TRACK_W / 2)}
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint>
        <wheel_separation>{TRACK_W}</wheel_separation><wheel_radius>{R:.5f}</wheel_radius>
        <topic>/cmd_vel</topic><odom_topic>/odom</odom_topic><frame_id>odom</frame_id>
        <child_frame_id>base_link</child_frame_id><odom_publish_frequency>20</odom_publish_frequency>
        <max_linear_acceleration>3</max_linear_acceleration><max_angular_acceleration>20</max_angular_acceleration>
      </plugin>
    </model>"""


def world(track, overhead=True):
    text, (w, h) = make_world.world(track)
    start = make_world.start_pose(track, h)
    # replace the static display model by the driven robot
    i = text.index('    <model name="prototype_a">')
    j = text.index("    </model>", i) + len("    </model>")
    text = text[:i] + robot(start).lstrip("\n") + text[j:]
    # the floor needs a collision surface now
    text = text.replace('<link name="floor">', f'''<link name="floor">
        <collision name="ground"><geometry><plane><normal>0 0 1</normal><size>{w + 2:.3f} {h + 2:.3f}</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>''')
    text = text.replace("<max_step_size>0.01</max_step_size>", "<max_step_size>0.002</max_step_size>")
    text = text.replace("<ambient>0.7 0.7 0.7 1</ambient>", "<ambient>1 1 1 1</ambient>")
    text = text.replace("<diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.3 0.2 -1</direction>",
                        "<diffuse>1 1 1 1</diffuse><direction>-0.15 0.1 -1</direction>")
    extra = '\n    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>'
    if overhead and track == "dragster":
        # a static camera that sim_record.py moves behind the robot every frame (a camera on the moving
        # model rendered nothing here)
        sx, sy, syaw = start
        extra += f"""
    <model name="chase_camera"><static>true</static>
      <pose>{sx - 0.55 * math.cos(syaw):.4f} {sy - 0.55 * math.sin(syaw):.4f} 0.40 0 0.42 {syaw:.4f}</pose>
      <link name="l"><sensor name="cam" type="camera">
        <topic>/overhead</topic><update_rate>10</update_rate><always_on>1</always_on>
        <camera><horizontal_fov>1.15</horizontal_fov><image><width>960</width><height>540</height></image>
        <clip><near>0.05</near><far>30</far></clip></camera>
      </sensor></link></model>"""
    if overhead and track != "dragster":
        cx, cy, z, W, H = w / 2, h / 2, 4.0, 960, 540
        span = max(h * 1.12, w * 1.12 * W / H)
        extra += f"""
    <model name="overhead_camera"><static>true</static>
      <pose>{cx:.4f} {cy:.4f} {z} 0 1.5708 3.14159</pose>
      <link name="l"><sensor name="cam" type="camera">
        <topic>/overhead</topic><update_rate>10</update_rate><always_on>1</always_on>
        <camera><horizontal_fov>{2 * math.atan(span / 2 / z):.5f}</horizontal_fov><image><width>{W}</width><height>{H}</height></image>
        <clip><near>0.05</near><far>20</far></clip></camera>
      </sensor></link></model>"""
    return text.replace("  </world>", extra + "\n  </world>")


if __name__ == "__main__":
    name = sys.argv[1]
    os.makedirs(os.path.join(make_world.HERE, "worlds"), exist_ok=True)
    out = os.path.join(make_world.HERE, "worlds", f"sim_{name}.sdf")
    open(out, "w", encoding="utf-8", newline="\n").write(world(name))
    print(out)
