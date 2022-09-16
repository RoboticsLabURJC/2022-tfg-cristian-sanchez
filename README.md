[![linting: pylint](https://img.shields.io/badge/linting-pylint-yellowgreen)](https://github.com/PyCQA/pylint)

<h1 align="center">RADIO FREQUENCY DETECTOR - DRONE</h1>

<p align="center">
<img src="https://github.com/RoboticsLabURJC/2022-tfg-cristian-sanchez/blob/main/res/real_drone.jpeg" alt="Real drone" width="600"/>
</p>

## Introduction

This repository contains my bachelor's degree final project, which final goal is to implement a real drone system that can locate and navigate throught a RF signal.

However, before doing that, the first steps are to simulate properly simple tasks, using **Gazebo 11** and **ROS Noetic for Ubuntu 20.04 LTS**.

The objective is to  solve simple problems, in order to scale into more complex behaviors. All these developments are contained [here](https://roboticslaburjc.github.io/2022-tfg-cristian-sanchez/), which is a bitacora where I post week by week all progress.

## Installation

### Step 1: Install ROS Noetic and Gazebo 11.

- [ROS Noetic Desktop Full](http://wiki.ros.org/noetic/Installation/Ubuntu)
- Gazebo 11

At this point this commands should run properly:

`roscore &`
`rosrun gazebo_ros gazebo`

### Step 2: Install JdeRobot drone infrastructure

```
sudo apt update && sudo apt upgrade -y
sudo apt install ros-noetic-jderobot-drones
```

### Step 3: PX4

1. Download and install Geographiclib
```
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm ./install_geographiclib_datasets.sh  # optional
```

2. Install common dependencies
```
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake \
    build-essential genromfs ninja-build exiftool \
    python3-pip python3-dev python-is-python3 -y
```

3. Get PX4 source (v1.11.3)
```
mkdir ~/repos && cd repos
git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b v1.11.3
```

4. Run PX4 installation script
```
cd ~/repos/PX4-Autopilot/Tools/setup/
bash ubuntu.sh --no-nuttx --no-sim-tools
```

5. Install gstreamer
```
sudo apt install libgstreamer1.0-dev
sudo apt install gstreamer1.0-plugins-bad
```

6. Build PX4
```
cd ~/repos/PX4-Autopilot
DONT_RUN=1 make px4_sitl gazebo
```

7. Export environment variables
```
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/repos/PX4-Autopilot/build/px4_sitl_default/build_gazebo' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/repos/PX4-Autopilot/Tools/sitl_gazebo/models' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/repos/PX4-Autopilot/build/px4_sitl_default/build_gazebo' >> ~/.bashrc    
echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/repos/PX4-Autopilot' >> ~/.bashrc
echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/repos/PX4-Autopilot/Tools/sitl_gazebo' >> ~/.bashrc
    
source ~/.bashrc
```

8. (OPTIONAL) Try PX4 
```
roslaunch px4 mavros_posix_sitl.launch
pxh> commander arm # when launching finishes
```

### Step 4: Install workspace

1. Clone and set up workspace

```
git clone https://github.com/RoboticsLabURJC/2022-tfg-cristian-sanchez.git
```

2. Environment variables

```
echo 'export ROS_WORKSPACE=~/2022-tfg-cristian-sanchez' >> ~/.bashrc
source ~/.bashrc
```

3. Compile

```
roscd
catkin_make
```

4. Change default iris model to our custom one

```
roscd irisdrone_gazebo/models/custom_iris
mv iris.sdf ~/repos/PX4-Autopilot/Tools/sitl_gazebo/models/iris
```

5. (OPTIONAL) Try it! At this point, you should be able to run the code inside this repository.

```
roslaunch teleop px4_rc.launch
```

