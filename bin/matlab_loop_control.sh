#!/bin/bash

# Parse arguments
while getops "c:" flag; do
    case $flag in
        c) controller=$OPTARG;;
    esac
done

if [[-z $controller ]]
then
    controller=0
fi

# Launch the simulation environment
source ~/catkin_ws/devel/setup.bash
roslaunch robot4ws_simulations archimede_gazebo_basic.launch &

sleep 10

# Activate python environment
source ~/p38Env/bin/activate

# Run joystick controller if requested
if [$controller = 1 ]
then
    # Run the kinematics node
    roslaunch robot4ws_kinematics kinematics.launch &
    sleep 2
    roslaunch robot4ws-teleop-joystick controller.launch
else
    roslaunch robot4ws_kinematics kinematics.launch
fi

# Kill
killall -9 gzserver gzclient