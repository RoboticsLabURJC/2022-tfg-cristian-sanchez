---
title: "Week 5-6. Center to center app"
last_modified_at: 2022-08-01T19:43:00
categories:
  - Blog
tags:
  - ROS Noetic
  - Kanban
  - Gazebo
  - openCV
---

After a time, I've continued with the project, developing a new app. In this case, I wanted the drone to travel 2 m in the XY axes directions, moving inside a grid of 2x2 m cells.

## Week 5

At the beginning, I needed to adapt the previous teleoperation mini-project to be able to command that types of orders. So I've simplified the GUI and tuned de necessary values, resulting this:

<p align="center">
<img src="/2022-tfg-cristian-sanchez/images/c2c_gui.png" alt="c2c GUI" width="500"/>
</p>

Also, and for scalability purposes, I've made a new launcher with a simple empty world.

## Week 6

Next part was targeted on developing a debugging system. Here I needed to decided which way to take:

- Gazebo simulator
- Rviz

Gazebo seemed to be simpler, but rviz was again, more scalable, so I chose rviz. 

The way the debug app was done was by using markers, one for the cells of the grid, and the other for the trajectory of the drone. The idea behind it, was to visualice visited and unvisited regions, to make, in the future, a heatmap of the intensity of the radio frequency signal.

Here is a little demonstration of how it works:

<p align="center">
<img src="/2022-tfg-cristian-sanchez/images/debugging_path.gif" alt="debug demo" width="500"/>
</p>