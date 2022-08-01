---
title: "Week 1-2. Getting started"
last_modified_at: 2022-08-01T19:43:00
categories:
  - Blog
tags:
  - ROS Noetic
  - Kanban
  - Gazebo
  - openCV
---

## Week 1
The first week, I dedicated my time to look into some information inside [Antonio's TFM](https://github.com/RoboticsLabURJC/2020-tfm-antonio-triguero) about drones and the infrastructure behind. Also, I set up my TFG github repository to ensure the traceability of the project.

Once that was done, I installed all the software needs to start coding and make sure that it was working properly:

1. ROS Noetic for Ubuntu 20.04 LTS
2. MavROS (Mavlink)
3. JdeRobot-drones

<p align="center">
<img src="/2022-tfg-cristian-sanchez/images/px4_drone_gz.png" alt="drone" width="500"/>
</p>

## Week 2
The next week, I started developing a simple teleoperator for Iris drone. I tried  some libraries (tkinter, pyqt5, ...etc) but openCV was the best option, because it was simple and easy to work with it.

The goal, was to be able to command velocities using sliders, so the first idea was to move in the XYZ axes.

On the other hand, I was introduce into kanban method, similar to scrum, wich objective is to manage and organice the tasks of the project. 
