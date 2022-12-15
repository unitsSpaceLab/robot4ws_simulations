# robot4ws_simulations

# Installation
## Ubuntu
Install Ubuntu 18.04 LTS

## Install main packets
Download .deb from !(https://code.visualstudio.com/)[here]
sudo apt install git

## ROS
Follow !(http://wiki.ros.org/melodic/Installation/Ubuntu)

After installation go to ROS tutorial, as per the last step of the guide.

Follow Tutorial 1

If robot4ws_simulations exists as a folder in the repository, then copy the folder in catkin_ws. If not, clone this repo directly in catkin_ws.

Clone robot4ws_msgs in catkin_ws/src, then run catkin_make in catkin_ws/

Run source devel/setup.bash in catkin_ws/ to activate the workspace

Clone robot4ws_description in catkin_ws/src, then run catkin_make in catkin_ws/

Run source devel/setup.bash in catkin_ws/ to activate the workspace

Run roslaunch robot4ws_simulations archimede_gazebo_basic.launch
(NON FUNZIONA)