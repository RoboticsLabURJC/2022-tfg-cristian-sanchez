---
title: "Week 3-4. Improving teleoperation"
last_modified_at: 2022-08-01T19:43:00
categories:
  - Blog
tags:
  - ROS Noetic
  - Kanban
  - Gazebo
  - openCV
---

Continuing with the project, this weeks aimed to improve the radio control and complete the documentation of the proccess.

## Week 3

Once the teleoperation functionality was working properly, we planned to improve it, adding a video screen (editing the SDF model file and including the pluging) that is responsible for showing a front view inside the drone, and also added some buttons for the drone to perform some simple tasks:

1. Takeoff
2. Land
3. Turn specific angles/move specific distance
4. Stop

Also, I modified the slider GUI to make it easy to use. Instead of moving around the XYZ axes, I developed a system relative to the Iris drone, as seen in the video below:

<iframe width="560" height="315" src="https://www.youtube.com/embed/_XKJYMi-2PE" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Week 4

Finally, this week goal was to end this teleoperation mini-project.

I implemented a new method in charge of landing, via services, using a custom msg to communicate between the radio_control and the PX4 controller. Here is a little summary:

`uint8 cmd`

- IDLE = 0
- TAKEOFF = 1
- LAND = 2
- POSITION = 3
- VELOCITY = 4

I also included all the bitacora inside github pages posts, and I reorganiced the code to stay clean and ready to use (adding some usefull comments inside).
