#!/usr/bin/env python3
"""
ZMP Preview Gait Controller for ArduHumanoid SITL
Architecture: ZMP math → MAVLink servo override → ArduPilot → ArduPilotPlugin → Gazebo joints
Based on: scaron.info/robotics/prototyping-a-walking-pattern-generator.html
"""
import time, math, threading
import numpy as np
from pymavlink import mavutil
import sys
sys.path.insert(0, '/home/neetamis/humanoid-ardupilot-sitl/scripts')
from preview_control    import LIPMPreviewController, ZMPReferenceGenerator
from foot               import SwingFootTrajectory, FootstepPlanner
from inverse_kinematics import LegIKSolver

# ── Robot parameters ──────────────────────────────────────────────────────────
DT           = 0.02
COM_HEIGHT   = 0.38
PREVIEW_HZ   = 40
STEP_LENGTH  = 0.05
FOOT_SEP     = 0.12
STEP_DUR     = 0.8
DS_DUR       = 0.2
SWING_H      = 0.05
N_STEPS      = 6
G            = 9.81
OMEGA        = math.sqrt(G / COM_HEIGHT)

# ── DCM gains (from tuning blog) ──────────────────────────────────────────────
DCM_KP = 2.0
DCM_KI = 0.1

# ── ArduPilot PWM conversion (from SDF) ──────────────────────────────────────
# angle = (pwm - 1500) / 500 * multiplier + offset
# → pwm = (angle - offset) / multiplier * 500 + 1500
HIP_MULT  = 1.571;  HIP_OFF  = -0.5
KNEE_MULT = 2.618;  KNEE_OFF = -0.5
ANKLE_MULT = 1.047; ANKLE_OFF = 0.0

def angle_to_pwm(angle_rad, multiplier, offset):
    pwm = (angle_rad - offset) / multiplier * 500 + 1500
    return int(np.clip(pwm, 1000, 2000))

# ── State ─────────────────────────────────────────────────────────────────────
pitch = 0.0; roll = 0.0; com_vx = 0.0; com_vy = 0.0
_lock = threading.Lock()

def mavlink_reader(mav):
    global pitch, roll
    while True:
        msg = mav.recv_match(type=['ATTITUDE','LOCAL_POSITION_NED'],
                             blocking=True, timeout=1.0)
        if msg and msg.get_type() == 'ATTITUDE':
            with _lock:
                pitch = msg.pitch
                roll  = msg.roll

def send_joints(mav, lhr, lhp, lk, la, rhr, rhp, rk, ra):
    """Send joint angles as MAVLink servo overrides to ArduPilot."""
    pwms = [
        angle_to_pwm(lhp, HIP_MULT,  HIP_OFF),   # CH1 l_hip_pitch
        angle_to_pwm(rhp, HIP_MULT,  HIP_OFF),   # CH2 r_hip_pitch
        angle_to_pwm(lk,  KNEE_MULT, KNEE_OFF),  # CH3 l_knee
        angle_to_pwm(rk,  KNEE_MULT, KNEE_OFF),  # CH4 r_knee
        angle_to_pwm(la,  ANKLE_MULT, ANKLE_OFF), # CH5 l_ankle
        angle_to_pwm(ra,  ANKLE_MULT, ANKLE_OFF), # CH6 r_ankle
        1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500  # CH7-16
    ]
    mav.mav.rc_channels_override_send(
        mav.target_system,
        mav.target_component,
        *pwms[:16]
    )

def main():
    print("ArduHumanoid ZMP Controller")
    print("Architecture: ZMP → MAVLink → ArduPilot → ArduPilotPlugin → Gazebo")
    print()

    mav = mavutil.mavlink_connection('udp:0.0.0.0:14551')
    mav.wait_heartbeat()
    print(f"Connected! System {mav.target_system}")

    threading.Thread(target=mavlink_reader, args=(mav,), daemon=True).start()

    # Setup controllers
    lipm_x = LIPMPreviewController(DT, COM_HEIGHT, PREVIEW_HZ)
    lipm_y = LIPMPreviewController(DT, COM_HEIGHT, PREVIEW_HZ)
    zmp_gen = ZMPReferenceGenerator(DT, STEP_DUR, DS_DUR, STEP_LENGTH, FOOT_SEP)
    planner = FootstepPlanner(STEP_LENGTH, FOOT_SEP, N_STEPS)
    ik_l    = LegIKSolver('left')
    ik_r    = LegIKSolver('right')

    # Generate ZMP reference
    zmp_x_ref, zmp_y_ref = zmp_gen.generate(N_STEPS)
    footsteps = planner.plan()
    total = len(zmp_x_ref)
    pad   = PREVIEW_HZ
    zmp_x_pad = np.concatenate([zmp_x_ref, np.full(pad, zmp_x_ref[-1])])
    zmp_y_pad = np.concatenate([zmp_y_ref, np.full(pad, zmp_y_ref[-1])])

    # Standing IK
    j_l = ik_l.solve(COM_HEIGHT, 0.0, 0.0)
    j_r = ik_r.solve(COM_HEIGHT, 0.0, 0.0)
    print(f"Standing pose:")
    print(f"  hip_pitch={math.degrees(j_l['hip_pitch']):.1f}° "
          f"knee={math.degrees(j_l['knee']):.1f}° "
          f"ankle={math.degrees(j_l['ankle']):.1f}°")

    # Wait for EKF3
    print("Waiting for EKF3...")
    while True:
        msg = mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=5.0)
        if msg and (msg.flags & 0x1F) == 0x1F:
            print("EKF3 ready!")
            break
        time.sleep(0.1)

    # Hold standing pose 3s
    print("Holding stand 3s...")
    t0 = time.time()
    while time.time()-t0 < 3.0:
        send_joints(mav,
                    j_l['hip_roll'], j_l['hip_pitch'], j_l['knee'], j_l['ankle'],
                    j_r['hip_roll'], j_r['hip_pitch'], j_r['knee'], j_r['ankle'])
        time.sleep(DT)

    # Walking loop
    print(f"Walking {N_STEPS} steps...")
    Nss = int(STEP_DUR/DT)
    Nds = int(DS_DUR/DT)
    step_starts = []
    idx = 0
    for _ in range(N_STEPS):
        idx += Nds; step_starts.append(idx); idx += Nss

    step_ptr   = 0
    swing_traj = None
    swing_time = 0.0
    swing_side = None
    stance_x   = {'left': 0.0, 'right': 0.0}
    swing_x    = {'left': 0.0, 'right': 0.0}

    # DCM integrator
    dcm_int = 0.0

    for k in range(total):
        loop_start = time.time()

        com_x = lipm_x.step(zmp_x_pad[k:k+PREVIEW_HZ])
        com_y = lipm_y.step(zmp_y_pad[k:k+PREVIEW_HZ])

        # DCM-based balance correction
        with _lock: p = pitch
        dcm   = lipm_x.dcm
        dcm_target = zmp_x_pad[k]
        dcm_err = dcm - dcm_target
        dcm_int = np.clip(dcm_int + dcm_err * DT, -0.2, 0.2)
        bal = np.clip(DCM_KP * dcm_err + DCM_KI * dcm_int, -0.3, 0.3)

        # Footstep trigger
        if step_ptr < len(step_starts) and k >= step_starts[step_ptr]:
            fs = footsteps[step_ptr]
            swing_side = fs[2]
            swing_traj = SwingFootTrajectory(
                swing_x[swing_side], fs[0], fs[1], STEP_DUR, SWING_H)
            swing_time = 0.0
            step_ptr  += 1

        hr_l = -com_y * 2.0
        hr_r =  com_y * 2.0

        if swing_traj and swing_side:
            sx, sy, sz = swing_traj.at(swing_time)
            swing_time += DT
            if swing_side == 'left':
                j_l = ik_l.solve(COM_HEIGHT - bal*0.1, sx-com_x, sz, hr_l)
                j_r = ik_r.solve(COM_HEIGHT + bal*0.1, stance_x['right']-com_x, 0.0, hr_r)
                swing_x['left'] = sx
            else:
                j_r = ik_r.solve(COM_HEIGHT - bal*0.1, sx-com_x, sz, hr_r)
                j_l = ik_l.solve(COM_HEIGHT + bal*0.1, stance_x['left']-com_x, 0.0, hr_l)
                swing_x['right'] = sx
        else:
            j_l = ik_l.solve(COM_HEIGHT, stance_x['left']-com_x,  0.0, hr_l)
            j_r = ik_r.solve(COM_HEIGHT, stance_x['right']-com_x, 0.0, hr_r)

        send_joints(mav,
                    j_l['hip_roll'], j_l['hip_pitch'], j_l['knee'], j_l['ankle'],
                    j_r['hip_roll'], j_r['hip_pitch'], j_r['knee'], j_r['ankle'])

        if k % 25 == 0:
            print(f"\rk={k}/{total} com_x={com_x:+.3f} dcm={dcm:+.3f} "
                  f"pitch={math.degrees(p):+.1f}° bal={bal:+.3f} "
                  f"step={step_ptr}/{N_STEPS}", end='', flush=True)

        elapsed = time.time() - loop_start
        if elapsed < DT:
            time.sleep(DT - elapsed)

    print("\nDone. Returning to stand...")
    j_l = ik_l.solve(COM_HEIGHT, 0.0, 0.0)
    j_r = ik_r.solve(COM_HEIGHT, 0.0, 0.0)
    for _ in range(50):
        send_joints(mav,
                    j_l['hip_roll'], j_l['hip_pitch'], j_l['knee'], j_l['ankle'],
                    j_r['hip_roll'], j_r['hip_pitch'], j_r['knee'], j_r['ankle'])
        time.sleep(DT)
    print("Complete!")

if __name__ == '__main__':
    main()
