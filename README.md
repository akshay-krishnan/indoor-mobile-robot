# indoor-mobile-robot
Codes for an autonomous indoor robot that can traverse static known environments using a Microsoft Kinect v2. 

<h2> Description of files </h2>

1. func_for_scan.py
This is a script that contains a function to obtain the expected scan measurements from the map image. 

2. localization.py
If started with this script, the robot uses particle filters on the depth data from Microsoft Kinect to estimate its location in the map. 

