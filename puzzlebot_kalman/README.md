### Puzzlebot_kalman

Contains the main codes for the Extended Kalman Filter algorithm implementation in Puzzlebot.

The final version of the code, and the one successfully implemented in simulation is in the file kalman_filter.py.

The other codes are previous versions which could be run as simoultaneous ROS nodes, but ideally Kalman algorithm is in only one node.

The launch file rviz_set_up.lunch contains all necessary files to run the simulation and loads Rviz coniguration for visualization.
This file runs:
- Rviz node: for complete visualization of simulation
- aruco_detector_cam.py: for ArUco detection and observation model sensing
- kalman_filter.py: for Extended Kalman Filter algorithm computation
- reference_odometry.py: for parallel visualization of Gazebo and Rviz
