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


Also ya gotta make sure your localization is set to true if you want to drive around (it is a bit buggy)


```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true localization:=true
```

It turns out adding an aruco marker into the model is a bit of a pain. Ok first you need to create a model.sdf file that looks like this:

```sdf
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="aruco_marker">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>0.2 0.2</size>  <!-- You can adjust size as needed -->
          </plane>
        </geometry>
        <material>
          <!-- Specify the texture using a proper image path -->
          <texture>
            <image>model://aruco_marker/materials/textures/marker.png</image> <!-- Point to your texture -->
          </texture>
        </material>
      </visual>
    </link>
  </model>
</sdf>

```

and it needs to be in ~/ros2_wssrc/turtlebot4_simulator/models/aruco_marker/model.sdf 

you also need to create a model.config file that looks like

```sdf
<?xml version="1.0" ?>
<model>
  <name>aruco_marker</name>
  <version>1.0</version>
  <!-- Ensure model.sdf is referenced correctly -->
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>YourEmail@example.com</email>
  </author>
  <description>A simple Aruco marker model</description>
</model>

```

and this needs to be in ~/ros2_ws/src/turtlebot4_simulator/models/aruco_marker/model.config 


You also need to add a marker.png file to //aruco_marker/materials/textures/marker.png

after doing this you can do the following to build and open your world
```bash
colcon build --packages-select turtlebot4_simulator 
source ./install/setup.bash
ign gazebo src/turtlebot4_simulator/turtlebot4_ignition_bringup/worlds/warehouse.sdf 
```

And now you will have a blank black square that is definitely NOT an aruco marker (Kill me)

Not sure why I am having this issue. I thought it might be an issue with the image but I swapped in a picture of a frog and it didn't fix it :( 

Ok Apprently I need to use a script like in this tutorial: https://classic.gazebosim.org/tutorials?tut=color_model

Will explore this later

For now I am going to set up a better Ros2 workspace for testing purposes. My Ros2_ws is a mess and its getting to be a pain to deal with









































Report Details:

-Methods
-Building block of what the program was built on 
-Theory behind the marker
-Detection algorithm works
-Test case
-Results
-Video
-









