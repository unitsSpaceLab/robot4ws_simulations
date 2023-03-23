#!/bin/bash


# @author: Matteo Caruso
# @email: matteo.caruso@phd.units.it
# @email: matteo.caruso1993@gmail.com

source ~/catkin_ws/devel/setup.bash


rostopic pub /cmd_vel_motors robot4ws_msgs/Dynamixel_parameters1 "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
One_Primary: 1.0
One_Secondary: 0.0
Two_Primary: 1.0
Two_Secondary: 0.0
Three_Primary: 1.0
Three_Secondary: 0.0
Four_Primary: 1.0
Four_Secondary: 0.0
Five_Secondary: 0.0
Five_Primary: 0.0
Six_Primary: 0.0
Six_Secondary: 0.0
Seven_Primary: 0.0
Seven_Secondary: 0.0
Eight_Primary: 0.0
Eight_Secondary: 0.0"
