#!/usr/bin/env python3
import time, math, subprocess
from pymavlink import mavutil

MAVLINK_URL = 'udp:127.0.0.1:14551'
HIP_STAND  = -0.09
KNEE_STAND =  0.35
KNEE_LIFT  =  0.38
HIP_STEP   =  0.20
DURATION   =  6.0

Kp, Ki, Kd = 0.40, 0.002, 0.08
integral, last_error, last_time = 0, 0, time.time()

def send(topic, val):
    subprocess.Popen(['gz','topic','-t',topic,'-m','gz.msgs.Double',
                     '-p',f'data: {val:.4f}'],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def send_pose(lh, rh, lk, rk):
    send('/l_hip_pitch/cmd', lh)
    send('/r_hip_pitch/cmd', rh)
    send('/l_knee/cmd', lk)
    send('/r_knee/cmd', rk)
    send('/l_ankle/cmd', 0.0)
    send('/r_ankle/cmd', 0.0)

def balance(pitch, dt):
    global integral, last_error, last_time
    error = pitch
    integral = max(-0.3, min(0.3, integral + error * dt))
    derivative = (error - last_error) / dt
    last_error = error
    return max(-0.3, min(0.3, Kp*error + Ki*integral + Kd*derivative))

print("Connecting to ArduPilot...")
mav = mavutil.mavlink_connection('udp:0.0.0.0:14551')
mav.wait_heartbeat()
print("Connected!")

# Kill startup.sh now that we're connected
subprocess.run(['pkill', '-f', 'startup.sh'], capture_output=True)
print("startup.sh stopped")
# Immediately hold standing pose to prevent fall
print("Holding pose...")
for _ in range(20):
    send_pose(HIP_STAND, HIP_STAND, KNEE_STAND, KNEE_STAND)
    time.sleep(0.05)

# Calibrate standing pitch
print("Calibrating...")
samples = []
for _ in range(20):
    msg = mav.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
    if msg:
        samples.append(msg.pitch)
    time.sleep(0.05)
STANDING_PITCH = sum(samples) / len(samples) if samples else 0.0
print(f"Standing pitch: {math.degrees(STANDING_PITCH):+.1f}°")

gait_phases = [
    (HIP_STAND,          HIP_STAND,          KNEE_STAND, KNEE_STAND, "STAND"),
    (HIP_STAND-0.05,     HIP_STAND+0.05,     KNEE_STAND, KNEE_STAND, "SHIFT_R"),
    (HIP_STAND+HIP_STEP, HIP_STAND,          KNEE_LIFT,  KNEE_STAND, "STEP_L"),
    (HIP_STAND,          HIP_STAND,          KNEE_STAND, KNEE_STAND, "STAND"),
    (HIP_STAND+0.05,     HIP_STAND-0.05,     KNEE_STAND, KNEE_STAND, "SHIFT_L"),
    (HIP_STAND,          HIP_STAND+HIP_STEP, KNEE_STAND, KNEE_LIFT,  "STEP_R"),
]

print("Stabilizing 3 seconds...")
t_start = time.time()
while time.time() - t_start < 3.0:
    msg = mav.recv_match(type='ATTITUDE', blocking=True, timeout=0.5)
    if msg:
        now = time.time()
        dt = max(now - last_time, 0.01)
        bal = balance(msg.pitch - STANDING_PITCH, dt)
        last_time = now
        lh = HIP_STAND - bal
        rh = HIP_STAND - bal
        send_pose(lh, rh, KNEE_STAND, KNEE_STAND)
        print(f"\rSTABILIZE    | bal:{bal:+.3f} | LH:{lh:+.2f} RH:{rh:+.2f}", end='', flush=True)
    time.sleep(0.05)

print("\nGait starting!")
# Reset balance integral to prevent carry-over from stabilize phase
integral = 0
last_error = 0
phase = 0
phase_start = time.time()

while True:
    msg = mav.recv_match(type='ATTITUDE', blocking=True, timeout=0.5)
    if not msg:
        continue

    now = time.time()
    dt = max(now - last_time, 0.01)
    bal = balance(msg.pitch - STANDING_PITCH, dt)
    last_time = now

    lh0, rh0, lk, rk, name = gait_phases[phase % len(gait_phases)]
    elapsed = time.time() - phase_start

    if elapsed >= DURATION:
        phase += 1
        phase_start = time.time()
        continue

    lh = lh0 - bal
    rh = rh0 - bal
    send_pose(lh, rh, lk, rk)

    print(f"\r{name:<12} | bal:{bal:+.3f} | LH:{lh:+.2f} RH:{rh:+.2f} LK:{lk:+.2f} RK:{rk:+.2f}", end='', flush=True)
    time.sleep(0.05)
