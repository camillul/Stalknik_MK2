
#Stalknik

## Table of Contents


1. [Introduction]
2. [Installation]
3. [Setup]
4. [Launcher]
5. [ROS architecture]
6. [Test]

### Introduction

Stalknik is a drone which aim to follow a car.
This car in our case will be chasing a simple remote car but it could work of course with higher scales.
This project is held by 3 developpers: 

CAMILLE Ulrich
RODRIGUES Nicolas
GASPARD Clement

This drone will be developped under ROS2 Galactic

### Installation

-Documentation is at https://docs.ros.org
Follow ROS Galactic installation instruction

-(used for cv_bridge package) install Boost library 
https://sourceforge.net/projects/boost/
please install the version 14.2


### Setup

You must use x86 Native Tools Command Prompt for VS 2019 as administrator, then :

cd <path to repository>\Stalknik_MK2\ros_stalknik && call <path to ros>\ros2_galactic\local_setup.bat && colcon build --merge-install || call install\setup.bat

This warning should appear :
"[rti_connext_dds_cmake_module][warning] RTI Connext DDS environment script not found (\resource\scripts\rtisetenv_x64Win64VS2017.bat). RTI Connext DDS will not be available at runtime, unless you already configured PATH manually."

### Launcher

In order to use/call the software :
- move into launch forlder
- ros2 launch All_launcher.py

if you want to use with a simulation:
-ros2 launch sim_launcher.py


### ROS architecture

iteration 1 :

![My Image](https://github.com/camillul/Stalknik_MK2/blob/main/PFE_image/rosgraph10022022.png)

iteration 2 :

![My Image](https://github.com/camillul/Stalknik_MK2/blob/main/PFE_image/rosgraph22022022.png)

iteration 3 :

![My Image](https://github.com/camillul/Stalknik_MK2/blob/main/PFE_image/rosgraph27022022.png)

### Test

To test

launch_test <path to test file>

