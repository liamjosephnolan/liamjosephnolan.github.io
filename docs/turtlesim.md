---
layout: page
title: ROS2 TurtleSim Edge detection
description: >
  Edge detect in turtle sim
hide_description: true
---

Here you can find a brief walkthrough on creating and building an edge detection ROS node for TurtleSim

## Creating a package

First we will create our package and node. While in the ros2_ws we can enter the following into the terminal:

~~~js
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name edge_detect edge_detection
~~~

Let's break this down:

~~~js

* ros2 pkg create: This is the base command to create a new ROS 2 package.

* --build-type ament_cmake: This option specifies the build system to be used for the package. In this case, ament_cmake is selected, which is a build system based on CMake that is commonly used in ROS 2.

* --license Apache-2.0: This option specifies the license for the package. In this case, the Apache License 2.0 is chosen. This license grants permissions for users to use, modify, and distribute the software under certain conditions.

* --node-name edge_detect: This option specifies the name of the main node for the package. In this case, the node is named "edge_detect".

* edge_detection: This is the name of the package being created. In ROS 2, a package is a directory that contains ROS-related files, such as source code, configuration files, and dependencies.

~~~

So, when you run this command, it creates a new ROS 2 package named "edge_detection" with the specified build system, license, and main node name. We now should have a folder in the ros2_ws/src directory called edge_detection

![alt text](image-1.png)