#!/usr/bin/env python3
import time, math, subprocess, threading
import numpy as np
from pymavlink import mavutil
import sys
sys.path.insert(0, '/home/neetamis/humanoid-ardupilot-sitl/scripts')
from preview_control    import LIPMPreviewController, ZMPReferenceGenerator
from foot               import SwingFootTrajectory, FootstepPlanner
from inverse_kinematics import LegIKSolver

DT=0.02; COM_HEIGHT=0.38; PREVIEW_HORIZON=20
STEP_LENGTH=0.12; FOOT_SEP=0.08
STEP_DURATION=0.6; DS_DURATION=0.1
SWING_HEIGHT=0.08; N_STEPS=8
Kp_bal=0.10; Ki_bal=0.001; Kd_bal=0.01

pitch=0.0; roll=0.0; _lock=threading.Lock()

def mavlink_reader(mav):
    global pitch, roll
    while True:
        msg = mav.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
        if msg:
            with _lock:
                pitch=msg.pitch; roll=msg.roll

def send_joint(topic, angle):
    subprocess.Popen(['gz','topic','-t',topic,'-m','gz.msgs.Double',
                      '-p',f'data: {angle:.4f}'],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def send_all(lhr,lhp,lk,la,rhr,rhp,rk,ra):
    send_joint('/l_hip_roll/cmd',lhr)
    send_joint('/l_hip_pitch/cmd',lhp)
    send_joint('/l_knee/cmd',lk)
    send_joint('/l_ankle/cmd',la)
    send_joint('/r_hip_roll/cmd',rhr)
    send_joint('/r_hip_pitch/cmd',rhp)
    send_joint('/r_knee/cmd',rk)
    send_joint('/r_ankle/cmd',ra)

def main():
    print("ZMP Preview Gait Controller starting...")
    mav = mavutil.mavlink_connection('udp:0.0.0.0:14551')
    mav.wait_heartbeat()
    print("Connected!")
    subprocess.run(['pkill','-f','startup.sh'], capture_output=True)
    threading.Thread(target=mavlink_reader, args=(mav,), daemon=True).start()

    lipm_x = LIPMPreviewController(DT, COM_HEIGHT, PREVIEW_HORIZON)
    lipm_y = LIPMPreviewController(DT, COM_HEIGHT, PREVIEW_HORIZON)
    zmp_gen = ZMPReferenceGenerator(DT, STEP_DURATION, DS_DURATION,
                                    STEP_LENGTH, FOOT_SEP)
    planner = FootstepPlanner(STEP_LENGTH, FOOT_SEP, N_STEPS)
    ik_l = LegIKSolver('left')
    ik_r = LegIKSolver('right')

    zmp_x_ref, zmp_y_ref = zmp_gen.generate(N_STEPS)
    footsteps = planner.plan()
    total = len(zmp_x_ref)
    pad = PREVIEW_HORIZON
    zmp_x_pad = np.concatenate([zmp_x_ref, np.full(pad, zmp_x_ref[-1])])
    zmp_y_pad = np.concatenate([zmp_y_ref, np.full(pad, zmp_y_ref[-1])])

    bal_int=0.0; bal_last=0.0; last_t=time.time()

    j_l = ik_l.solve(COM_HEIGHT, 0.0, 0.0)
    j_r = ik_r.solve(COM_HEIGHT, 0.0, 0.0)
    print("Holding stand 3s...")
    t0=time.time()
    while time.time()-t0 < 3.0:
        send_all(j_l['hip_roll'],j_l['hip_pitch'],j_l['knee'],j_l['ankle'],
                 j_r['hip_roll'],j_r['hip_pitch'],j_r['knee'],j_r['ankle'])
        time.sleep(DT)

    print(f"Walking {N_STEPS} steps...")
    Nss=int(STEP_DURATION/DT); Nds=int(DS_DURATION/DT)
    step_starts=[]
    idx=0
    for _ in range(N_STEPS):
        idx+=Nds; step_starts.append(idx); idx+=Nss

    step_ptr=0; swing_traj=None; swing_time=0.0; swing_side=None
    stance_x={'left':0.0,'right':0.0}
    swing_x={'left':0.0,'right':0.0}

    for k in range(total):
        loop_start=time.time()
        com_x=lipm_x.step(zmp_x_pad[k:k+PREVIEW_HORIZON])
        com_y=lipm_y.step(zmp_y_pad[k:k+PREVIEW_HORIZON])

        if step_ptr < len(step_starts) and k >= step_starts[step_ptr]:
            fs=footsteps[step_ptr]
            swing_side=fs[2]
            swing_traj=SwingFootTrajectory(
                swing_x[swing_side], fs[0], fs[1], STEP_DURATION, SWING_HEIGHT)
            swing_time=0.0; step_ptr+=1

        now=time.time()
        with _lock: p=pitch
        dt_b=max(now-last_t, DT); last_t=now
        bal_int=np.clip(bal_int+p*dt_b, -0.3, 0.3)
        bal_d=(p-bal_last)/dt_b; bal_last=p
        bal=np.clip(Kp_bal*p+Ki_bal*bal_int+Kd_bal*bal_d, -0.25, 0.25)

        hr_l=-com_y*2.0+(-bal*0.3)
        hr_r= com_y*2.0+( bal*0.3)

        if swing_traj and swing_side:
            sx,sy,sz=swing_traj.at(swing_time)
            swing_time+=DT
            if swing_side=='left':
                j_l=ik_l.solve(COM_HEIGHT, sx-com_x, sz, hr_l)
                j_r=ik_r.solve(COM_HEIGHT, stance_x['right']-com_x, 0.0, hr_r)
                swing_x['left']=sx
            else:
                j_r=ik_r.solve(COM_HEIGHT, sx-com_x, sz, hr_r)
                j_l=ik_l.solve(COM_HEIGHT, stance_x['left']-com_x, 0.0, hr_l)
                swing_x['right']=sx
        else:
            j_l=ik_l.solve(COM_HEIGHT, stance_x['left']-com_x, 0.0, hr_l)
            j_r=ik_r.solve(COM_HEIGHT, stance_x['right']-com_x, 0.0, hr_r)

        send_all(j_l['hip_roll'],j_l['hip_pitch'],j_l['knee'],j_l['ankle'],
                 j_r['hip_roll'],j_r['hip_pitch'],j_r['knee'],j_r['ankle'])

        if k%25==0:
            print(f"\rk={k}/{total} com_x={com_x:+.3f} com_y={com_y:+.3f} "
                  f"pitch={math.degrees(p):+.1f}deg bal={bal:+.3f} "
                  f"step={step_ptr}/{N_STEPS}", end='', flush=True)

        elapsed=time.time()-loop_start
        if elapsed < DT: time.sleep(DT-elapsed)

    print("\nDone. Returning to stand...")
    j_l=ik_l.solve(COM_HEIGHT,0.0,0.0)
    j_r=ik_r.solve(COM_HEIGHT,0.0,0.0)
    for _ in range(50):
        send_all(j_l['hip_roll'],j_l['hip_pitch'],j_l['knee'],j_l['ankle'],
                 j_r['hip_roll'],j_r['hip_pitch'],j_r['knee'],j_r['ankle'])
        time.sleep(DT)

if __name__=='__main__':
    main()
