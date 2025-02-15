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
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true localization:=true
```

You have to make sure to hit the play button in the bottom right but the TB functions more or less similar to the actual system. This also launchs slam and Rviz 

I can bring up the camera feed with

```bash
ros2 run rqt_image_view rqt_image_view
```

Topics seem to function as normal which is nice. I have previously written an aruco marker detection node called "turt_camera_test_node" which detects aruco markers. I have modified it to subscribe to the the normal oakd camera topic (Rather than the robot name space) and it appears to be working. I need to figure out how to add an aruco marker in the the simulation to fully verify though. 

It turns out adding an aruco marker into the model was a massive pain and I have it working but still don't fully understand it.

I struggled alot with getting textures to work in the Gazebo simulation. Most of the documentation I found was for Gazebo Classic but as far as I can tell I am using Gazebo Igniton (AKA Gazebo Sim)

This models use dae files to mesh the textures. The easiest was to place an aruco marker in the simulation was to yank the marker0 folder from this GH repo: https://github.com/mikaelarguedas/gazebo_models

The entire marker0 folder can be placed in the models folder of the turtlebot4_igniton_bringup but we need to update the model path with 

```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/your/models
```


The warehouse.sdf file needed to be modified to include this model like this

```XML
    <include>
      <uri>model://marker0</uri>
      <name>aruco_marker</name>
      <pose>0 0 1 0 0 0</pose> <!-- Adjust the pose as needed -->
    </include>

```

Now when in our ros2_ws directory we can run the follwowing and we should see the aruco marker in the word 

```bash
colcon build --packages-select turtlebot4_simulator 
source ./install/setup.bash
ign gazebo src/turtlebot4_simulator/turtlebot4_ignition_bringup/worlds/warehouse.sdf 
```


But now when I try to run the turtlebot simulation the aruco marker wont show. I can manually load it as a mesh though which is fine for now. 

When I run

```bash
ros2 run turtle_factory_py turt_camera_test_node
```

I am not detecting markers which is obviously bad.










































Report Details:

-Methods
-Building block of what the program was built on 
-Theory behind the marker
-Detection algorithm works
-Test case
-Results
-Video
-









