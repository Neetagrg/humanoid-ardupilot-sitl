#!/usr/bin/env python3
"""
Balance controller for ArduBiped_Proto - stronger PID with ankle correction
"""
from pymavlink import mavutil
import time, math, subprocess

print("Connecting to ArduPilot on udp:14551...")
mav = mavutil.mavlink_connection('udp:0.0.0.0:14551')
mav.wait_heartbeat()
print("Connected!")

# Stronger gains
Kp, Ki, Kd = 2.0, 0.01, 0.5
integral, last_error, last_time = 0, 0, time.time()

# Standing pose offsets
HIP_STAND   = -0.09
KNEE_STAND  =  0.35
ANKLE_STAND =  0.0

def send_joint(topic, angle):
    subprocess.Popen(
        ['gz','topic','-t',topic,'-m','gz.msgs.Double','-p',f'data: {angle:.4f}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

print("Balance controller running...")
while True:
    msg = mav.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
    if msg:
        pitch = msg.pitch
        now = time.time()
        dt = max(now - last_time, 0.01)
        error = pitch
        integral = max(-0.5, min(0.5, integral + error * dt))
        derivative = (error - last_error) / dt
        correction = max(-0.4, min(0.4,
            Kp*error + Ki*integral + Kd*derivative))

        # Hip correction (primary)
        hip = HIP_STAND - correction
        # Ankle correction (opposite direction, smaller)
        ankle = ANKLE_STAND + correction * 0.5

        send_joint('/l_hip_pitch/cmd', hip)
        send_joint('/r_hip_pitch/cmd', hip)
        send_joint('/l_ankle/cmd', ankle)
        send_joint('/r_ankle/cmd', ankle)
        send_joint('/l_knee/cmd', KNEE_STAND)
        send_joint('/r_knee/cmd', KNEE_STAND)

        last_error, last_time = error, now
        print(f"Pitch:{math.degrees(pitch):+.1f}° Corr:{correction:+.4f} "
              f"Hip:{hip:+.3f} Ankle:{ankle:+.3f}")
    time.sleep(0.02)
