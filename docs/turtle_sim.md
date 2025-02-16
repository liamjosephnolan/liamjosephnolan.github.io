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

I am not detecting markers which is obviously bad. I can clearly see the marker on the camera feed but maybe there is a problem with my camera detection node? 

I think next steps would be testing the aruco detection algorithm? Im sure its a simple bug. Could also be a lighting issue

Update: I found a solution. I changed the marker ID to 42 and changed the world to one with better lighting and it detected the marker. 

So now next steps are to actually start the docking procedure which is very exciting!!!

```
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("Image Viewer Node Started. Listening to /oakd/rgb/preview/image_raw")

    def image_callback(self, msg):

        marker_size = 0.1 # meters
        focal_length = 500 # Pixels

        try:
            # Convert ROS Image message to OpenCV image
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
            parameters = cv2.aruco.DetectorParameters()

# Create the ArUco detector
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
# Detect the markers
            corners, ids, rejected = detector.detectMarkers(gray)
# Print the detected markers
            print("Detected markers_debug_state:", ids)
            if ids is not None:
                
                for marker_corners in corners:
                    # marker_corners has shape (1, 4, 2)
                    corners_2d = marker_corners[0]  # Extract the (4, 2) array

                    # Upper-left corner
                    upper_left = tuple(corners_2d[0])

                    # Upper-right corner
                    upper_right = tuple(corners_2d[1])
                pixel_distance = np.sqrt((upper_right[1] - upper_left[0])**2 + (upper_right[0]- upper_left[1])**2)
                distance_m = (marker_size * focal_length) / pixel_distance
                print(f"Estimated distance to the marker: {distance_m} meters")

                cv2.aruco.drawDetectedMarkers(image, corners, ids)
                cv2.imshow('Robot Camera', image)
            else:
                cv2.imshow('Robot Camera', image)
            if cv2.waitKey(1) == ord('q'):
                rclpy.shutdown()  # Gracefully shut down the node if 'q' is pressed
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

```













































Report Details:

-Methods
-Building block of what the program was built on 
-Theory behind the marker
-Detection algorithm works
-Test case
-Results
-Video
-









