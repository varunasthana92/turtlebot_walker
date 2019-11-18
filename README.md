# ROS package: turtlebot_walker
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
---

## Overview
This ROS package "turtlebot_walker" outlines the usage of ros functionalities with the gazebo simulator. A custom world has been setup in gazebo to provide a confined space with various obstacles. A turtlebot is then spawned into this environment with a laser scnaner on it. The scan data is used to check if the way ahead of turtlebot is clear of any obstacle or not. If no nearby obstacle is found, then the turtlebot moves forward, or else it stops and rotates untill the way is clear.

<p align="center">
<img src="https://github.com/varunasthana92/turtlebot_walker/blob/master/additional_files/bot_behaviour1.png">
</p>

Turtlebot was approaching the shperical object on the left. It was detected by the sensor and course correction is done. 
<p align="center">
<img src="https://github.com/varunasthana92/turtlebot_walker/blob/master/additional_files/bot_behaviour2.png">
</p>

### Dependencies
```
ROS Kinetic
Ubuntu 16.04 LTS
Gazebo
```

### Assumptions
To run the package-
1) Above mentioned dependancies are available and running in the user's system.
2) catkin_ws workspace is properly setup.
(If your workspace is named something else or is at other path, use that name and/or path in the below commands)
3) turtlebot package is installed. If not, then run the below command to install it.
```
$ sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```
4) Default forder for Gazebo's World Files is known to user. (World files are located in a versioned system directory, for example /usr/share/gazebo-7 on Ubuntu. Fodler name may vary with the version of Gazebo)
### How to build the program
Clone the package in the catkin_ws/src directory
```
$ cd catkin_ws/src
$ git clone https://github.com/varunasthana92/turtlebot_walker.git
$ cd ..
$ catkin_make
```
Copy the file world/obstacle.world, provided in the repository, into the gazebo's default folder for World Files (as higlighed in the Assumptions section). You may require admin access to copy the file. An example of the command to copy the file is provided below, but it may change based on the final destination folder. (You will be asked to enter password after $ sudo command)

```
$ cd ~/catkin_ws/src/turtlebot_walker/world
$ sudo cp obstacle.world /usr/share/gazebo-7/worlds
$ exit
```

### How to run cpplint and cppcheck
Use the below commands to run cppcheck and cpplint.
```
$ cd ~/catkin_ws/src/turtlebot_walker
$ cppcheck $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./results" )
$ cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./results" )
```
### How to run and interact with program
In a new terminal
```
$ roslaunch turtlebot_walker turtlebot_walker.launch
```
Gazebo will open with the custom world and turtlebot simulation.

#### Data recording by rosbag
The launch file also has a tag to initiate data recording of all the topics (except for /camera* topics) in a bag file "BagData". By default this functionality is kept "ON", which will save a new .bag file (or replace any existing file with same name) in /results directory, each time the launch file is executed. User has the option to disable the recording of data by passing an argument set to 0 as below while running the launch file.

```
$ roslaunch turtlebot_walker turtlebot_walker.launch record_data:=0
```
#### Inspecting and playing back the rosbag file
Terminate all instances of the program, gazebo and ros. Use the below set of commands to inspect the previously created BagData.bag file and playback the same.

In a new terminal
```
$ cd ~/catkin_ws/src/turtlebot_walker/results
$ rosbag info BagData.bag
```
Output will be something similar to as shown below. You may notice that the date and the time stamps are not as per the system clock, this is because of gazebo.
```
path:        BagData.bag
version:     2.0
duration:    17.9s
start:       Dec 31 1969 21:45:20.60 (9920.60)
end:         Dec 31 1969 21:45:38.52 (9938.52)
size:        99.1 MB
messages:    123782
compression: none [130/130 chunks]
types:       bond/Status                           [eacc84bf5d65b6777d4c50f463dfb9c8]
             dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
             dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
             gazebo_msgs/LinkStates                [48c080191eb15c41858319b4d8a609c2]
             gazebo_msgs/ModelStates               [48c080191eb15c41858319b4d8a609c2]
             geometry_msgs/Twist                   [9f195f881246fdfa2798d1d3eebca84a]
             nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
             rosgraph_msgs/Clock                   [a9c97c1d230cfc112e270351a944ee47]
             rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
             sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/JointState                [3066dcd76a6cfaef579bd0f34173e9fd]
             sensor_msgs/LaserScan                 [90c7ef2dc6895d81024acba2ac42f369]
             std_msgs/String                       [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage                    [94810edda583a504dfda3829e70d7eec]
topics:      /clock                                            17923 msgs    : rosgraph_msgs/Clock 
             /cmd_vel_mux/active                                   1 msg     : std_msgs/String                      
             /cmd_vel_mux/input/navi                             180 msgs    : geometry_msgs/Twist                  
             /cmd_vel_mux/parameter_descriptions                   1 msg     : dynamic_reconfigure/ConfigDescription
             /cmd_vel_mux/parameter_updates                        1 msg     : dynamic_reconfigure/Config           
             /depthimage_to_laserscan/parameter_descriptions       1 msg     : dynamic_reconfigure/ConfigDescription
             /depthimage_to_laserscan/parameter_updates            1 msg     : dynamic_reconfigure/Config           
             /gazebo/link_states                               17836 msgs    : gazebo_msgs/LinkStates               
             /gazebo/model_states                              17847 msgs    : gazebo_msgs/ModelStates              
             /gazebo/parameter_descriptions                        1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo/parameter_updates                             1 msg     : dynamic_reconfigure/Config           
             /gazebo_gui/parameter_descriptions                    1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo_gui/parameter_updates                         1 msg     : dynamic_reconfigure/Config           
             /joint_states                                     16938 msgs    : sensor_msgs/JointState               
             /laserscan_nodelet_manager/bond                      66 msgs    : bond/Status                           (2 connections)
             /mobile_base/commands/velocity                      168 msgs    : geometry_msgs/Twist                  
             /mobile_base/sensors/imu_data                     16972 msgs    : sensor_msgs/Imu                      
             /mobile_base_nodelet_manager/bond                   132 msgs    : bond/Status                           (3 connections)
             /odom                                             16984 msgs    : nav_msgs/Odometry                    
             /rosout                                              27 msgs    : rosgraph_msgs/Log                     (8 connections)
             /rosout_agg                                          12 msgs    : rosgraph_msgs/Log                    
             /scan                                               149 msgs    : sensor_msgs/LaserScan                
             /tf                                               18538 msgs    : tf2_msgs/TFMessage                    (2 connections)
             /tf_static                                            1 msg     : tf2_msgs/TFMessage
```
Keep this terminal open, and we will come back to it.

In a new terminal
```
$ roscore
```
To play this bag file, use the below command.
Use the same terminal in which we ran the rosbag info command, as we need to be in the directory where the file is available.
```
$ rosbag play BagData.bag
```
Once the playback finishes, you may ternimate all instances of ros.
