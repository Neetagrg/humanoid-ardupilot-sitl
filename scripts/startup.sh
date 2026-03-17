#!/bin/bash
echo "Holding standing pose - matches gait controller values"
while true; do
  gz topic -t /l_hip_pitch/cmd -m gz.msgs.Double -p "data: -0.09" 2>/dev/null
  gz topic -t /r_hip_pitch/cmd -m gz.msgs.Double -p "data: -0.09" 2>/dev/null
  gz topic -t /l_knee/cmd -m gz.msgs.Double -p "data: 0.35" 2>/dev/null
  gz topic -t /r_knee/cmd -m gz.msgs.Double -p "data: 0.35" 2>/dev/null
  gz topic -t /l_ankle/cmd -m gz.msgs.Double -p "data: 0.0" 2>/dev/null
  gz topic -t /r_ankle/cmd -m gz.msgs.Double -p "data: 0.0" 2>/dev/null
  sleep 0.05
done
