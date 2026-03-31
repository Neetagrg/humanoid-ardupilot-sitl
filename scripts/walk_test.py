#!/usr/bin/env python3
import time
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double

DT = 0.02
node = Node()
pubs = {
    'l_hip': node.advertise('/l_hip_pitch/cmd', Double),
    'r_hip': node.advertise('/r_hip_pitch/cmd', Double),
    'l_knee': node.advertise('/l_knee/cmd', Double),
    'r_knee': node.advertise('/r_knee/cmd', Double),
}
time.sleep(1.0)

def send(l_hip=0.0, r_hip=0.0, l_knee=0.0, r_knee=0.0):
    for name, val in [('l_hip',l_hip),('r_hip',r_hip),
                      ('l_knee',l_knee),('r_knee',r_knee)]:
        msg = Double()
        msg.data = float(val)
        pubs[name].publish(msg)

def hold(duration, **kwargs):
    t0 = time.time()
    while time.time() - t0 < duration:
        send(**kwargs)
        time.sleep(DT)

print("Standing 3s - click PLAY!")
hold(3.0)

print("Step 1: Lean backward first...")
hold(1.0, l_hip=-0.15, r_hip=-0.15)

print("Step 2: Lift left knee while leaning back...")
hold(1.5, l_hip=-0.2, r_hip=-0.1, l_knee=0.4, r_knee=0.0)

print("Step 3: Return to stand...")
hold(1.0)

print("Done!")
