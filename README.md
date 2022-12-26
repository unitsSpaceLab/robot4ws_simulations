# robot4ws_simulations

# Installation
## Ubuntu
Install Ubuntu 18.04 LTS

## Install main packets
Download .deb from !(https://code.visualstudio.com/)[here]
```sudo apt install git```

## ROS
Follow [install guide](http://wiki.ros.org/melodic/Installation/Ubuntu)

After installation go to ROS tutorial, as per the last step of the guide.

Follow Tutorial 1

### Installation of the robot environment and model
If ```robot4ws_simulations``` exists as a folder in the repository, then copy the folder in ```catkin_ws```. If not, clone this repo directly in ```catkin_ws```.

Clone ```robot4ws_msgs``` in ```catkin_ws/src```, then run ```catkin_make``` in ```catkin_ws/```

Clone ```robot4ws_description``` in ```catkin_ws/src```
If using ROS kinetic ```git checkout kinetic```
If using ROS melodic ```git checkout melodic```
Then run catkin_make in ```catkin_ws/```

### Test
Everytime we open a Terminal we should do the following:
* Run ```source devel/setup.bash``` in ```catkin_ws/``` to activate the workspace
* Run ```roslaunch robot4ws_simulations archimede_gazebo_basic.launch include_plugins:=true```

**Note**: if an API error arises, run the following
```nano ~/.ignition/fuel/config.yaml```
Change the url from ```api.ignitionfuel.org``` to ```api.ignitionrobotics.org```


## Dependecies

* gazebo_realsense_plugin: [repository](https://github.com/pal-robotics/realsense_gazebo_plugin)
* robot4ws_description: [repository](aaa)
* robot4ws_kinematics: [repository](aaaa)
* robot4ws_teleop_keyboard: [repository](aaaa)
* robot4ws_teleop_cotroller: [repository](aaaa)
* hector_gazebo_plugins: [repository](http://wiki.ros.org/action/fullsearch/hector_gazebo_plugins?action=fullsearch&context=180&value=linkto%3A%22hector_gazebo_plugins%22)
``` sudo apt-get install ros-melodic-hector-gazebo-plugins ```