-- humanoid.lua
-- Bipedal walking gait controller for ArduPilot Rover SITL
local THIGH_LEN=200.0; local SHIN_LEN=200.0; local LEG_LEN=400.0
local CH_L_HIP=0; local CH_R_HIP=1; local CH_L_KNEE=2
local CH_R_KNEE=3; local CH_L_ANKLE=4; local CH_R_ANKLE=5
local PWM_NEUTRAL=1500; local PWM_MIN=1000; local PWM_MAX=2000
local PWM_SCALE=500.0/(math.pi*0.5); local PWM_TIMEOUT=100
local STEP_HEIGHT=40.0; local STEP_LENGTH=55.0
local PHASE_DUR_MS=600; local LOOP_PERIOD_MS=20
local ZMP_ANKLE_LOAD=0.12; local ZMP_ANKLE_UNLOAD=-0.10
local S_STAND=0; local S_SHIFT_RIGHT=1; local S_SWING_LEFT=2
local S_PLANT_LEFT=3; local S_SHIFT_LEFT=4; local S_SWING_RIGHT=5
local S_PLANT_RIGHT=6; local gait_state=S_STAND; local phase_start_ms=0
local function lerp(a,b,t) if t<=0 then return a end; if t>=1 then return b end; return a+(b-a)*t end
local function smoothstep(t) t=math.max(0,math.min(1,t)); return t*t*(3-2*t) end
local function angle_to_pwm(a) return math.floor(math.max(PWM_MIN,math.min(PWM_MAX,PWM_NEUTRAL+a*PWM_SCALE))+0.5) end
local function set_joints(lh,lk,la,rh,rk,ra)
  SRV_Channels:set_output_pwm_chan_timeout(CH_L_HIP,angle_to_pwm(lh),PWM_TIMEOUT)
  SRV_Channels:set_output_pwm_chan_timeout(CH_L_KNEE,angle_to_pwm(lk),PWM_TIMEOUT)
  SRV_Channels:set_output_pwm_chan_timeout(CH_L_ANKLE,angle_to_pwm(la),PWM_TIMEOUT)
  SRV_Channels:set_output_pwm_chan_timeout(CH_R_HIP,angle_to_pwm(rh),PWM_TIMEOUT)
  SRV_Channels:set_output_pwm_chan_timeout(CH_R_KNEE,angle_to_pwm(rk),PWM_TIMEOUT)
  SRV_Channels:set_output_pwm_chan_timeout(CH_R_ANKLE,angle_to_pwm(ra),PWM_TIMEOUT)
end
local function solve_ik(fx,fz)
  local L1=THIGH_LEN; local L2=SHIN_LEN
  local d2=fx*fx+fz*fz; local d=math.sqrt(d2); local mr=L1+L2-2
  if d>mr then local s=mr/d; fx=fx*s; fz=fz*s; d=mr; d2=d*d end
  local ck=math.max(-1,math.min(1,(d2-L1*L1-L2*L2)/(2*L1*L2)))
  local knee=math.acos(ck)
  local alpha=math.atan(fx,fz)
  local cb=math.max(-1,math.min(1,(d2+L1*L1-L2*L2)/(2*d*L1)))
  local hip=alpha-math.acos(cb)
  return hip,knee
end
local function stand_pose()
  local hip,knee=solve_ik(0,LEG_LEN); local ankle=-hip
  set_joints(hip,knee,ankle,hip,knee,ankle)
end
local function update()
  if not arming:is_armed() then stand_pose(); gait_state=S_STAND; phase_start_ms=millis():tofloat(); return update,LOOP_PERIOD_MS end
  local thr=rc:get_pwm(3); local walking=(thr~=nil)and(thr>1550)
  if not walking then stand_pose(); gait_state=S_STAND; phase_start_ms=millis():tofloat(); return update,LOOP_PERIOD_MS end
  local now=millis():tofloat(); local elapsed=now-phase_start_ms
  if gait_state==S_STAND then gait_state=S_SHIFT_RIGHT; phase_start_ms=now; elapsed=0 end
  local t_raw=elapsed/(PHASE_DUR_MS*1.0); local t=smoothstep(t_raw)
  if gait_state==S_SHIFT_RIGHT then
    local hip,knee=solve_ik(0,LEG_LEN)
    set_joints(hip,knee,lerp(0,ZMP_ANKLE_UNLOAD,t),hip,knee,lerp(0,ZMP_ANKLE_LOAD,t))
    if t_raw>=1 then gait_state=S_SWING_LEFT; phase_start_ms=now end
  elseif gait_state==S_SWING_LEFT then
    local rh,rk=solve_ik(0,LEG_LEN); local sw_z=LEG_LEN-STEP_HEIGHT*math.sin(t_raw*math.pi)
    local lh,lk=solve_ik(lerp(0,STEP_LENGTH,t),sw_z)
    set_joints(lh,lk,-lh,rh,rk,ZMP_ANKLE_LOAD)
    if t_raw>=1 then gait_state=S_PLANT_LEFT; phase_start_ms=now end
  elseif gait_state==S_PLANT_LEFT then
    local lh,lk=solve_ik(STEP_LENGTH,LEG_LEN); local rh,rk=solve_ik(0,LEG_LEN)
    set_joints(lh,lk,lerp(ZMP_ANKLE_UNLOAD,0,t),rh,rk,lerp(ZMP_ANKLE_LOAD,0,t))
    if t_raw>=1 then gait_state=S_SHIFT_LEFT; phase_start_ms=now end
  elseif gait_state==S_SHIFT_LEFT then
    local hip,knee=solve_ik(0,LEG_LEN)
    set_joints(hip,knee,lerp(0,ZMP_ANKLE_LOAD,t),hip,knee,lerp(0,ZMP_ANKLE_UNLOAD,t))
    if t_raw>=1 then gait_state=S_SWING_RIGHT; phase_start_ms=now end
  elseif gait_state==S_SWING_RIGHT then
    local lh,lk=solve_ik(0,LEG_LEN); local sw_z=LEG_LEN-STEP_HEIGHT*math.sin(t_raw*math.pi)
    local rh,rk=solve_ik(lerp(0,STEP_LENGTH,t),sw_z)
    set_joints(lh,lk,ZMP_ANKLE_LOAD,rh,rk,-rh)
    if t_raw>=1 then gait_state=S_PLANT_RIGHT; phase_start_ms=now end
  elseif gait_state==S_PLANT_RIGHT then
    local rh,rk=solve_ik(STEP_LENGTH,LEG_LEN); local lh,lk=solve_ik(0,LEG_LEN)
    set_joints(lh,lk,lerp(ZMP_ANKLE_LOAD,0,t),rh,rk,lerp(ZMP_ANKLE_UNLOAD,0,t))
    if t_raw>=1 then gait_state=S_SHIFT_RIGHT; phase_start_ms=now end
  end
  return update,LOOP_PERIOD_MS
end
gcs:send_text(6,"humanoid.lua loaded")
phase_start_ms=millis():tofloat()
return update,500
