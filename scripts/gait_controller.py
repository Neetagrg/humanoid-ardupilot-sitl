#!/usr/bin/env python3
"""
Gait Controller v5 - penguin walk, no lateral shift
Uses alternating hip pitch only for forward motion
"""
from pymavlink import mavutil
import time, math, subprocess

def send_joint(topic, angle):
    subprocess.Popen(
        f'gz topic -t {topic} -m gz.msgs.Double -p "data: {angle}"',
        shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

print("Connecting to ArduPilot...")
mav = mavutil.mavlink_connection('udp:0.0.0.0:14551')
mav.wait_heartbeat()
print("Connected!")

print("Calibrating...")
samples = []
while len(samples) < 50:
    msg = mav.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
    if msg:
        samples.append(msg.pitch)
STANDING_PITCH = sum(samples) / len(samples)
print(f"Standing pitch: {math.degrees(STANDING_PITCH):+.1f}°")

Kp, Ki, Kd = 0.15, 0.001, 0.02
integral, last_error, last_time = 0, 0, time.time()

HIP_STAND = -0.09
KNEE_STAND = 0.35
KNEE_LIFT  = 0.55
HIP_STEP   = 0.15
DURATION   = 2.5

# Penguin gait - no lateral shift, alternate knee lift with hip pitch
gait_phases = [
    ( HIP_STAND,           HIP_STAND,           KNEE_STAND, KNEE_STAND, "STAND"),
    ( HIP_STAND-0.05,      HIP_STAND+0.05,      KNEE_STAND, KNEE_STAND, "SHIFT_R"),
    ( HIP_STAND+HIP_STEP,  HIP_STAND,           KNEE_LIFT,  KNEE_STAND, "STEP_L"),
    ( HIP_STAND,           HIP_STAND,           KNEE_STAND, KNEE_STAND, "STAND"),
    ( HIP_STAND+0.05,      HIP_STAND-0.05,      KNEE_STAND, KNEE_STAND, "SHIFT_L"),
    ( HIP_STAND,           HIP_STAND+HIP_STEP,  KNEE_STAND, KNEE_LIFT,  "STEP_R"),
]

print("Stabilizing 5 seconds...")
phase_start = time.time()
gait_step = 0
gait_active = False

while True:
    msg = mav.recv_match(type='ATTITUDE', blocking=True, timeout=0.1)
    now = time.time()
    if msg:
        pitch = msg.pitch
        dt = max(now - last_time, 0.01)
        error = pitch - STANDING_PITCH
        integral = max(-0.5, min(0.5, integral + error * dt))
        derivative = (error - last_error) / dt
        bal = max(-0.3, min(0.3, Kp*error + Ki*integral + Kd*derivative))
        last_error, last_time = error, now

        if not gait_active:
            lh = rh = HIP_STAND - bal
            lk = rk = KNEE_STAND
            label = "STABILIZE"
            if now - phase_start > 5.0:
                gait_active = True
                phase_start = now
                print("Gait starting!")
        else:
            if now - phase_start > DURATION:
                gait_step = (gait_step + 1) % len(gait_phases)
                phase_start = now
            lh, rh, lk, rk, label = gait_phases[gait_step]
            lh -= bal
            rh -= bal

        send_joint('/l_hip_pitch/cmd', lh)
        send_joint('/r_hip_pitch/cmd', rh)
        send_joint('/l_knee/cmd', lk)
        send_joint('/r_knee/cmd', rk)
        print(f"{label:12s} | bal:{bal:+.3f} | LH:{lh:+.2f} RH:{rh:+.2f} LK:{lk:+.2f} RK:{rk:+.2f}")
