#!/usr/bin/env python3
"""Run this IMMEDIATELY after Gazebo starts, before anything else"""
import subprocess, time

def send(topic, val):
    subprocess.run(['gz','topic','-t',topic,'-m','gz.msgs.Double',
                   '-p',f'data: {val}'], capture_output=True)

print("Locking joints to standing pose...")
for _ in range(30):
    send('/l_hip_pitch/cmd', -0.10)
    send('/r_hip_pitch/cmd', -0.10)
    send('/l_knee/cmd',  0.30)
    send('/r_knee/cmd',  0.30)
    send('/l_ankle/cmd', 0.10)
    send('/r_ankle/cmd', 0.10)
    time.sleep(0.05)
print("Done — joints locked for 1.5s")
