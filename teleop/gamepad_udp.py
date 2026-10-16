"""Drive Prototype B from a Windows PC with an Xbox pad, without ROS on the PC (4.4 SLAM runs).
Reads the pad through XInput and sends the command at 20 Hz over UDP to udp_teleop.py on the robot,
which publishes teleop/cmd_vel. Hold LB to drive (deadman); RB with LB = faster.
Left stick up/down = forward/back, right stick left/right = turn. Ctrl+C to quit.
    python gamepad_udp.py [ROBOT_IP]"""
import ctypes
import json
import socket
import sys
import time

ROBOT = (sys.argv[1] if len(sys.argv) > 1 else "192.168.8.112", 5005)
V_SLOW, V_FAST, W_MAX = 0.15, 0.25, 0.8      # m/s, m/s, rad/s (turn slowly for scan matching)
LB, RB = 0x0100, 0x0200
DEAD = 0.12


class Pad(ctypes.Structure):
    _fields_ = [("buttons", ctypes.c_ushort), ("lt", ctypes.c_ubyte), ("rt", ctypes.c_ubyte),
                ("lx", ctypes.c_short), ("ly", ctypes.c_short), ("rx", ctypes.c_short), ("ry", ctypes.c_short)]


class State(ctypes.Structure):
    _fields_ = [("packet", ctypes.c_uint), ("pad", Pad)]


def axis(v):
    v = max(-1.0, v / 32767.0)
    return 0.0 if abs(v) < DEAD else (v - DEAD * (1 if v > 0 else -1)) / (1 - DEAD)


xinput = ctypes.windll.xinput1_4
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pad_id = next((i for i in range(4) if xinput.XInputGetState(i, ctypes.byref(State())) == 0), None)
if pad_id is None:
    sys.exit("No XInput gamepad found: connect it (USB or Bluetooth) and run again.")
print(f"Pad {pad_id} -> {ROBOT[0]}:{ROBOT[1]}. Hold LB to drive, RB for faster. Ctrl+C to quit.")
try:
    while True:
        s = State()
        v = w = 0.0
        if xinput.XInputGetState(pad_id, ctypes.byref(s)) == 0 and s.pad.buttons & LB:
            v = (V_FAST if s.pad.buttons & RB else V_SLOW) * axis(s.pad.ly)
            w = -W_MAX * axis(s.pad.rx)
        sock.sendto(json.dumps({"v": round(v, 3), "w": round(w, 3)}).encode(), ROBOT)
        print(f"\r  v {v:+.2f} m/s   w {w:+.2f} rad/s   ", end="", flush=True)
        time.sleep(0.05)
except KeyboardInterrupt:
    for _ in range(5):
        sock.sendto(b'{"v": 0, "w": 0}', ROBOT)
    print("\nstopped")
