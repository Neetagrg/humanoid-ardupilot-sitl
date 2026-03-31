#!/usr/bin/env python3
"""
ZMP Gait Controller - Phase 1 Penguin Walk
- Joints controlled via gz.transport directly
- ArduPilot EKF3 provides attitude feedback
- Penguin walk: hip pitch only, no knee lift
"""
import time, math, threading
import numpy as np
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from pymavlink import mavutil

# Parameters
DT = 0.02
STEP_DURATION = 1.0
SHIFT_DURATION = 0.5
HIP_STEP_ANGLE = 0.15   # rad forward step
HIP_SHIFT_ANGLE = 0.08  # rad lateral shift (hip roll approximation)
N_STEPS = 6

# Setup gz transport
node = Node()
pubs = {
    'l_hip': node.advertise('/l_hip_pitch/cmd', Double),
    'r_hip': node.advertise('/r_hip_pitch/cmd', Double),
    'l_knee': node.advertise('/l_knee/cmd', Double),
    'r_knee': node.advertise('/r_knee/cmd', Double),
}

def send(l_hip, r_hip, l_knee=0.0, r_knee=0.0):
    for name, val in [('l_hip',l_hip),('r_hip',r_hip),
                      ('l_knee',l_knee),('r_knee',r_knee)]:
        msg = Double()
        msg.data = float(val)
        pubs[name].publish(msg)

# Attitude from ArduPilot
pitch = 0.0
_lock = threading.Lock()

def mavlink_reader(mav):
    global pitch
    while True:
        msg = mav.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
        if msg:
            with _lock:
                pitch = msg.pitch

def hold(duration, l_hip=0.0, r_hip=0.0):
    t0 = time.time()
    while time.time() - t0 < duration:
        send(l_hip, r_hip)
        time.sleep(DT)

def main():
    print("Connecting to ArduPilot...")
    mav = mavutil.mavlink_connection('udp:0.0.0.0:14551')
    mav.wait_heartbeat()
    print("Connected!")
    threading.Thread(target=mavlink_reader, args=(mav,), daemon=True).start()

    print("Standing for 3s...")
    hold(3.0)

    print(f"Walking {N_STEPS} steps (penguin walk)...")
    for step in range(N_STEPS):
        with _lock: p = pitch
        print(f"\rStep {step+1}/{N_STEPS} pitch={math.degrees(p):+.1f}deg", end='', flush=True)

        if step % 2 == 0:
            # Left foot forward
            print(" LEFT", end='', flush=True)
            hold(SHIFT_DURATION, l_hip=0.05, r_hip=0.05)  # shift weight right
            hold(STEP_DURATION, l_hip=-HIP_STEP_ANGLE, r_hip=0.05)  # left foot forward
        else:
            # Right foot forward
            print(" RIGHT", end='', flush=True)
            hold(SHIFT_DURATION, l_hip=0.05, r_hip=0.05)  # shift weight left
            hold(STEP_DURATION, l_hip=0.05, r_hip=-HIP_STEP_ANGLE)  # right foot forward

    print("\nDone. Returning to stand...")
    hold(2.0)

if __name__ == '__main__':
    main()
