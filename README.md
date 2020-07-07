# Nubot_Pumbaa_Gazebo_Plugin
This is a Gazebo simulation plugin of Nubot_Pumbaa tracked vehicle.  
Generated by skywalker1941 from NuBot team.  

## Features Description
1.Use ROS topics to control the velocity & direction & flippers position.  
2.The robot tracks motion mechanism is based on the brilliant work of Martin Pecka.  

>M. Pecka, K. Zimmermann, and T. Svoboda, “Fast simulation of vehicles with non-deformable tracks,” Sep. 2017, pp. 6414–6419, doi: 10.1109/IROS.2017.8206546.

3.The flippers are controlled by directly setting the position to the model, which needs to be improved.  

## Operating Environment
Ubuntu 18.04  
ROS melodic  
Gazebo 9.0.0  
Python 3.6  

## Additional Configurations
opende head files (details are described later)  
threadpool libraries (maybe needed)  

## Files Descriptions

>/gazebo_deps_opende

This is a folder copy from the source code of gazebo(gazebo/deps/opende). It contains some head files needed by the plugin.  

>/Reference_File

This is a folder contains the original code of Martin Peica's work.  

>/Reference_File/threadpool-0_2_5-src.zip

This is a library pakage that may be needed during compilation. Just install it to the system.  

>/gazebo_description

The modle of robot & obstacles in the environment, including ".sdf" & ".world" file.  

>/Pumbaa_Msg

The self defined ROS message for Pumbaa control.  

>/Pumbaa_Plugin

The core code of Nubot_Pumbaa_Gazebo Plugin.  

>/Pumbaa_Control

The control algorithm of Nubot_Pumbaa in Python3.  

## Compilation Note

use  
```c++
mkdir src
cd src/
git pull https://github.com/skywalker1941/Nubot_Pummba_Gazebo.git
cd ..
catkin_make
```
to compile the project.  

## Run the Project
After compilation, run:  
```c++
roscore
source devel/setup.bash
cd src/
roslaunch Nubot_Pumbaa_test.launch
```

## Parameter Adjustment
You can adjust most robot & environment parameters by modifying the ".sdf" & ".world" files.  
At the ***end*** of:  

>/gazebo_description/models/Nubot_Pumbaa/model.sdf

you can change mechanical parameters of robot, including friction, flippers PID and so on.  

## Control the Robot
Control the velocity & angular by sending messages through rostopic.  
```c++
rostopic pub -1 /Nubot_Pumbaa/nubotcontrol/pumbaacmd nubot_pumbaa_msg/PumbaaCmd
```
'nubot_pumbaa_msg/PumbaaCmd' is a self defined message type.  
"vel_linear" means the linear velocity of the robot, "vel_angular" means the angular velocity.  
"front_left" means the rotate position of the front_left flipper, the rest are similar.  

## TODO
1.Find out the problem of current robot motion simulating mechanism, make it more real.  
2.Add self designed flipper position controller.  
3.Add RGBD camera.  
4.Connect to nubot_joy_stick.  
