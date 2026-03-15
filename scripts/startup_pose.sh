#!/bin/bash
sleep 1
gz topic -t /l_hip_pitch/cmd -m gz.msgs.Double -p "data: -0.2"
gz topic -t /r_hip_pitch/cmd -m gz.msgs.Double -p "data: -0.2"
gz topic -t /l_knee/cmd -m gz.msgs.Double -p "data: 0.4"
gz topic -t /r_knee/cmd -m gz.msgs.Double -p "data: 0.4"
sleep 0.5
gz topic -t /l_hip_pitch/cmd -m gz.msgs.Double -p "data: -0.2"
gz topic -t /r_hip_pitch/cmd -m gz.msgs.Double -p "data: -0.2"
gz topic -t /l_knee/cmd -m gz.msgs.Double -p "data: 0.4"
gz topic -t /r_knee/cmd -m gz.msgs.Double -p "data: 0.4"
