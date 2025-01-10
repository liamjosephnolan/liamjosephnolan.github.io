---
layout: page
title: Turtlebot 4 Simulation
description: Simulating Aruco Marker Detection with the Turtlebot4 Gazebo Simulation
hide_description: true
date: 10 Jan 2025
---
Apologies to my legions of fans for my extended absence from this blog (Exams are keeping me busy)

I am working on Aruco Marker tracking using the Turtlebot 4 but the physical hardware is such a headache and I need to learn Gazebo a bit better for my thesis. Because of these factors I am going to be doing this in simulation. 

Clearpath already created a really nice simulation world for the turtlebot, might as well use it. It can be launched with 


```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
```

You have to make sure to hit the play button in the bottom right but the TB functions more or less similar to the actual system. This also launchs slam and Rviz 

I can bring up the camera feed with

```bash
ros2 run rqt_image_view rqt_image_view
```

It gives a live feed of the camera. I can't actuall get the robot to move yet but the camera feed is live (I can place objects in front of it and it updates)

Topics seem to function as normal which is nice. I have previously written an aruco marker detection node called "turt_camera_test_node" which detects aruco markers. I have modified it to subscribe to the the normal oakd camera topic (Rather than the robot name space) and it appears to be working. I need to figure out how to add an aruco marker in the the simulation to fully verify though. 


