---
layout: project
title: 'ROS 2 Turtlesim Edge Detection'
caption: Custom ROS 2 Node for turtle sim that implements edge detection
date: '25-04-2024'
image: 
  path: /assets/img/projects/edge_detect.png
  srcset: 
    1920w: /assets/img/projects/edge_detect.png
    960w:  /assets/img/projects/edge_detect.png
    480w:  /assets/img/projects/edge_detect.png
links:
  - title: Link
    url: https://github.com/liamjosephnolan
sitemap: false
---
This custom C++ node for ROS 2 will spawn a simulated turtlebot in the center of the screen with a random orientation. It will then drive forward until an edge is detected where it will turn 90 degrees, print "Edge Detected" to the terminal and continue driving. A custom Bash script can install the package.

You can read more about the technical aspects on my [Dev Blog]{:.heading.flip-title}


[Dev Blog]: /docs/turtlesim
