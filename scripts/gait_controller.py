#!/usr/bin/env python3
import time
import subprocess
from pymavlink import mavutil

MAVLINK_URL = 'udp:127.0.0.1:14551'
HIP_STAND   = -0.15
KNEE_STAND  =  0.35
KNEE_LIFT   =  1.20
HIP_STEP    =  0.20
HIP_SHIFT   =  0.12
SHIFT_TIME  = 1.5
STEP_TIME   = 2.0
RETURN_TIME = 1.0

def send_joint(topic, angle):
    subprocess.Popen(['gz','topic','-t',topic,'-m','gz.msgs.Double','-p',f'data: {angle:.4f}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def send_pose(lh, rh, lk, rk):
    send_joint('/l_hip_pitch/cmd', lh)
    send_joint('/r_hip_pitch/cmd', rh)
    send_joint('/l_knee/cmd', lk)
    send_joint('/r_knee/cmd', rk)
    send_joint('/l_ankle/cmd', 0.0)
    send_joint('/r_ankle/cmd', 0.0)

def get_bal(mav):
    msg = mav.recv_match(type='ATTITUDE', blocking=True, timeout=0.5)
    return msg.pitch if msg else 0.0

print("Connecting...")
mav = mavutil.mavlink_connection(MAVLINK_URL)
mav.wait_heartbeat()
print("Connected!")

phases = [
    ('STAND',       SHIFT_TIME,  lambda: send_pose(HIP_STAND, HIP_STAND, KNEE_STAND, KNEE_STAND)),
    ('SHIFT_RIGHT', SHIFT_TIME,  lambda: send_pose(HIP_STAND+HIP_SHIFT, HIP_STAND-HIP_SHIFT, KNEE_STAND, KNEE_STAND)),
    ('STEP_LEFT',   STEP_TIME,   lambda: send_pose(HIP_STAND+HIP_SHIFT+HIP_STEP, HIP_STAND-HIP_SHIFT, KNEE_LIFT, KNEE_STAND)),
    ('RETURN',      RETURN_TIME, lambda: send_pose(HIP_STAND, HIP_STAND, KNEE_STAND, KNEE_STAND)),
    ('STAND',       SHIFT_TIME,  lambda: send_pose(HIP_STAND, HIP_STAND, KNEE_STAND, KNEE_STAND)),
    ('SHIFT_LEFT',  SHIFT_TIME,  lambda: send_pose(HIP_STAND-HIP_SHIFT, HIP_STAND+HIP_SHIFT, KNEE_STAND, KNEE_STAND)),
    ('STEP_RIGHT',  STEP_TIME,   lambda: send_pose(HIP_STAND-HIP_SHIFT, HIP_STAND+HIP_SHIFT+HIP_STEP, KNEE_STAND, KNEE_LIFT)),
    ('RETURN',      RETURN_TIME, lambda: send_pose(HIP_STAND, HIP_STAND, KNEE_STAND, KNEE_STAND)),
]

phase = 0
phase_start = time.time()
while True:
    name, duration, action = phases[phase % len(phases)]
    elapsed = time.time() - phase_start
    if elapsed >= duration:
        phase += 1
        phase_start = time.time()
        continue
    bal = get_bal(mav)
    action()
    print(f"{name:12s} | bal:{bal:+.3f} | t:{elapsed:.1f}/{duration:.1f}")
    time.sleep(0.2)
