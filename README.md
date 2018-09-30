# indoor-mobile-robot
Codes for an autonomous indoor robot that can traverse static known environments using a Microsoft Kinect v2. 

<h2> Description of files </h2>

1. func_for_scan.py:

This is a script that contains a function to obtain the expected scan measurements from the map image. 

2. localization.py:

If started with this script, the robot uses particle filters on the depth data from Microsoft Kinect to estimate its location in the map. It also provides directives to an underlying microcontroller to control the movement of the robot as necessitated by the algorithm.

3. mapping.launch

ROS launch file that launches the RTABMap's RGBD mapping using Kinect. This is a handheld mapping technique, and is used in this project to prepare an initial map of the surroundings. 

4. pathplanning.py

Implements the A* path planning algorihtm to calculate the shortest path between two points on the map. 

5. reducemap.py

Implements a simple resize function to resize a map as per the required resolution. 
