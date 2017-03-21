# 4-wheeled-ros-robot
A 4 wheeled robot for SLAM and navigation with ROS and Gazebo

Video: https://youtu.be/NZH0TxBaLyU

## Setup
You clone this package into your catkin-workspace/src folder and then build it with catkin_make.
I am using ROS Kinetic with the gazebo and navigation stack under Ubuntu 16.04. For older ROS versions you probably have to use qt4 for the GUI plugin.

Further, you need to prepare and source your .bashrc similar to this:
```
#ROS:
source /opt/ros/kinetic/setup.bash
source ~/catkin-workspace/devel/setup.bash
#Gazebo:
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin-workspace/src/qbot/gazebo/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/kinetic/lib
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/catkin-workspace/src/qbot/gazebo/world
```

## Start

You can start the Gazebo simulation with:
```
roslaunch ~/catkin-workspace/src/qbot/launch/gazebo.launch
```
And the navigation and SLAM packages with
```
roslaunch ~/catkin-workspace/src/qbot/launch/navigation.launch
```
Then, you can send navigation commands via
```
rqt --standalone qbot_4wheeled_robot
```
or with rviz.

## Note
Don't expect too much from the simulation because the skid-steering is inaccurately simulated. However this could be fixed by tuning physics parameters in the simulation or by simulating the robot motion on a higher abstraction level.