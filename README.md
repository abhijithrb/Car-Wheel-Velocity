# Car-Wheel-Velocity
Detects and tracks the wheels of car passing in front of a camera and finds their velocity in terms on pixels/frame
- I have used Hough Circles for detecting the wheels.
- I have used a centroid based tracking alogorithm where every new detection's center is compared with the centeres of existing tracks to update and assign new tracks

Implemented using OpenCV and C++.
Download/clone the repository to your local system. Make sure the input video file is in the same directory as main.cpp. OR change the path to the input video file at line 28 of main.cpp.
