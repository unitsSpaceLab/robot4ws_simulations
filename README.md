# robot4ws_simulations
This package is used to load and run the simulation in Gazebo of the Archimede rover, with (not yet developed) or without the differential.



# 1. Installation
## 1.1 Ubuntu
Install Ubuntu 18.04 LTS

## 1.2 Install main packets
Download .deb from [here](https://code.visualstudio.com/).
```sudo apt install git```

## 1.3 ROS
Follow the [ROS install guide](http://wiki.ros.org/melodic/Installation/Ubuntu).

After installation go to ROS tutorial, as per the last step of the guide.

Follow Tutorial 1

## 1.4 MATLAB
Install matlab from mathworks

## 1.5 Simulation environment
### 1.5.1 Installation of the robot environment and model
If ```robot4ws_simulations``` exists as a folder in the repository, then copy the folder in ```catkin_ws```. If not, clone this repo directly in ```catkin_ws```.

Clone ```robot4ws_msgs``` in ```catkin_ws/src```, then run ```catkin_make``` in ```catkin_ws/```

Clone ```robot4ws_description``` in ```catkin_ws/src```
If using ROS kinetic ```git checkout kinetic```
If using ROS melodic ```git checkout melodic```
Then run ```catkin_make``` in ```catkin_ws/```

**Note**: if an API error arises, run the following
```nano ~/.ignition/fuel/config.yaml```
Change the url from ```api.ignitionfuel.org``` to ```api.ignitionrobotics.org```

### 1.5.2 Dependecies
* gazebo_realsense_plugin: [repository](https://github.com/pal-robotics/realsense_gazebo_plugin)
* robot4ws_description: [repository](https://github.com/matteocaruso1993/robot4ws_description)
* robot4ws_kinematics: [repository](https://github.com/matteocaruso1993/rover4ws-kinematics)
* robot4ws_teleop_keyboard: [repository](https://github.com/matteocaruso1993/rover4ws_teleop_keyboard)
* robot4ws_teleop_controller: [repository](https://github.com/matteocaruso1993/robot4ws-teleop-joystick)
* hector_gazebo_plugins: [repository](http://wiki.ros.org/action/fullsearch/hector_gazebo_plugins?action=fullsearch&context=180&value=linkto%3A%22hector_gazebo_plugins%22)
``` sudo apt-get install ros-melodic-hector-gazebo-plugins ```



# 2. Run environment
## 2.1 Run simulation environment
Everytime we open a Terminal we should do the following:
* Run ```source devel/setup.bash``` in ```catkin_ws/``` to activate the workspace
* Run ```roslaunch robot4ws_simulations archimede_gazebo_basic.launch include_plugins:=true```


## 2.2 Simulation environment reset
To reset the simulation run ```rosservice call /gazebo/reset_simulation "{}"```

## 2.3 Extract simulation data from gazebo
The plugin ```/src/archimede_get_body_pose.cpp``` (header: ```/include/archimede_get_body_pose.h```) writes on the HDD, at each timestep, the position and velocity data of the bodies specified in ```robot4ws_description```. Data is a CSV file.
This can be found usually in ```src/robot4ws_simulations/data/link_data```, unless otherwise specified in ```robot4ws_description```.



# 3. Robot control
## 3.1 Robot control variables
The robot is controlled using ROS messages. Some examples are in ```/src/robot4ws_simulations/```.

In the simulation environment ```*_Secondary``` are not used. In the physical environment these are used as additional parameters for some types of motor control techniques.

**Hub motors**
>1 - Rear right wheel
>2 - Front right wheel
>3 - Rear left wheel
>4 - Front left wheel

**Steer motors**
>5 - Rear right wheel
>6 - Front right wheel
>7 - Rear left wheel
>8 - Front left wheel

**Naming scheme example**
Hub motor for rear right wheel:
```One_Primary```

## 3.2 Robot control via ROS in terminal
In ```/src/robot4ws_simulations/``` there are some examples of messages to control the robot.

## 3.3 Robot control via ROS in MATLAB
Run the rover companion app ```/RoverMatlab/main.mlapp```.
At startup, no ROS node is detected. This is normal; press OK when prompted. When controlling hardware, the IP should be specified.

When simulating:
* Specify the namespace, in this case ```Archimede```. When using the hardware robot, this is left blank.
* Check the ```Simulation``` checkbox.

Use the ```Teleop``` and ```Experimental testing``` tabs to control the robot.

### To add a subscriber
In the app ```properties``` add the variable for the subscriber you want to subscribe to, e.g. ```gazebo_link_states_subscriber```.

In ```initialize(app)```, in the simulation mode part of the ```if```, add the subscription function call:
```app.gazebo_link_states_subscriber = rossubscriber("/gazebo/link_states");```












