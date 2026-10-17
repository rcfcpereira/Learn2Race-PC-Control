"""Save the first frame published on /appe_cam (the camera added by shot.py): grab.py OUT.png"""
import sys, time, numpy as np, cv2
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
got = []
def cb(m):
    if not got:
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        got.append(a.copy())
n = Node(); n.subscribe(Image, "/appe_cam", cb)
t = time.time()
while not got and time.time() - t < 120: time.sleep(0.2)
if not got: sys.exit("no image")
cv2.imwrite(sys.argv[1], cv2.cvtColor(got[0][:, :, :3], cv2.COLOR_RGB2BGR)); print("saved", got[0].shape)
