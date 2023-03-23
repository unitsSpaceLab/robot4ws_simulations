#!/bin/bash

# @author: Matteo Caruso
# @email: matteo.caruso@phd.units.it
# @email: matteo.caruso1993@gmail.com

source ~/catkin_ws/devel/setup.bash

rostopic pub /cmd_vel_motors robot4ws_msgs/Dynamixel_parameters1 "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
One_Primary: 0.00264871
One_Secondary: 0.0
Two_Primary: 0.00264871
Two_Secondary: 0.0
Three_Primary: 0.00213614
Three_Secondary: 0.0
Four_Primary: 0.00213614
Four_Secondary: 0.0
Five_Secondary: 0.0
Five_Primary: -0.164637
Six_Primary: 0.164637
Six_Secondary: 0.0
Seven_Primary: -0.205645
Seven_Secondary: 0.0
Eight_Primary: 0.205645
Eight_Secondary: 0.0" >/dev/null &


sleep 2


rostopic pub /cmd_vel_motors robot4ws_msgs/Dynamixel_parameters1 "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
One_Primary: 2.64871
One_Secondary: 0.0
Two_Primary: 2.64871
Two_Secondary: 0.0
Three_Primary: 2.13614
Three_Secondary: 0.0
Four_Primary: 2.13614
Four_Secondary: 0.0
Five_Secondary: 0.0
Five_Primary: -0.164637
Six_Primary: 0.164637
Six_Secondary: 0.0
Seven_Primary: -0.205645
Seven_Secondary: 0.0
Eight_Primary: 0.205645
Eight_Secondary: 0.0" >/dev/null




							