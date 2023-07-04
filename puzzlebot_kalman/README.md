## Puzzlebot_kalman

Contains the main codes for the Extended Kalman Filter algorithm implementation in Puzzlebot.

The nodes and topics are connected in simulation this way:

![imagen](https://github.com/RoboticaInteligente8voTecCEM2023/advanced_navigation_mobile_robots/assets/67598380/05618060-fbbc-4e00-94cc-45d5eb10e431)

The final version of the code, and the one successfully implemented in simulation is in the file kalman_filter.py.

The other codes are previous versions which could be run as simoultaneous ROS nodes, but ideally Kalman algorithm is in only one node.

The launch file rviz_set_up.lunch contains all necessary files to run the simulation and loads Rviz coniguration for visualization.
This file runs:
- Rviz node: for complete visualization of simulation
- aruco_detector_cam.py: for ArUco detection and observation model sensing
- kalman_filter.py: for Extended Kalman Filter algorithm computation
- reference_odometry.py: for parallel visualization of Gazebo and Rviz

### ArUco Detection

ArUco detection node refers to the sensing part of the Kalman Filter. Within this robot's context, observation model consists on obtaining the distance of robot to a reference object, and differential angle between the object and the robot. The reference object is and ArUco. The node then, is in charge of effectively detecting ArUco markers and getting these observation model values.

The essence of detection relies on openCV built-in functions from cv2.aruco library. Function *detectMarkers* is the one for detecting all markers, ids and rejected candidates in image. You can get transformation between camera and each ArUco detected using *cv2.solvePnP*, the most important thing to look out while using this method is to have a precise camera matrix and camera coeffs. In simulation is easy because the information should be around somewhere, for example in Gazebo there is a topic *camera/camera_info*. For real-life robot implementations, there are several ways of obtaining these matrices, the one we used for this robot is Charuco calibration. Find more info about this in the /Aruco section at the beginning of the repository.

For the Filter to work, we need to know iteratively when the robot has seen a marker and calculate the observation model variables. When there is no marker detected, the Filter should also know. With the methods mentioned above, we can get these information:

![imagen](https://github.com/RoboticaInteligente8voTecCEM2023/advanced_navigation_mobile_robots/assets/67598380/bfd7c7c7-5bb6-4d0a-bc4c-fa7280af098b)

So the code does:
1. Detect markers
2. If none detected, publish euclidean distance of -1.0
3. Else, for each marker, get its rotation and translation, calculate euclidean distance to marker and differential angle, and save both numbers in a dictionary with ArUco id as key
4. Find closest marker: the one with smaller euclidean distance
5. If distance is smaller than 4 meters, publish ArUco coordinates (same refence frame as robot), euclidean distance, and differential angle.

### Extended Kalman Filter

For the development of the EKF, find all information in our training-partner's repository: https://github.com/ManchesterRoboticsLtd/TE3003B_Integration_of_Robotics_and_Intelligent_Systems.git.

Overview of the algorithm is as shown:

![imagen](https://github.com/RoboticaInteligente8voTecCEM2023/advanced_navigation_mobile_robots/assets/67598380/e26a6ffc-87e9-4abd-b66f-fad11d411ce0)

<sub>From Manchester Robotics presentations, 2023</sub>

Setps 1 to 4 refers to prediction phase of the Filter while the rest are about correction phase based on the observation model and robot sensing.

For the particular case of *Puzzlebot*, follow this logic:

![imagen](https://github.com/RoboticaInteligente8voTecCEM2023/advanced_navigation_mobile_robots/assets/67598380/dd731342-0f2c-4fe6-b8ef-3a30b5a59996)

For getting a Prediction, we follow the first two parts of the logic.

First we need both the robot state with and without noise. We get the state without noise by performing state transition model using data from input control vector. For the state with noise, we use the robot encoders for same model operations. This way get iterative data of the robot state and can calculate a covariance matrix.

The next step is Kalman Filter prediction which means we get a best estimation of the location of the robot, where the centroid of the confidence-elipsoid is the best estimation based on the gaussian distribution probabilistic function. We get this estimation by calculating the state covariance matrix, obtained from the Jacobian of the state transition model, the previous covariance matrix, and the non-deterministic noise matrix which maps the encoders' noise. The robot, by being a differential drive robot, has much more noise when rotating than when moving forward orbackwards.

The magic comes in this next phase, the correction phase. This phase occurs only if there is a landmark detected. Correction works by comparing state prediction of the robot against a landmark (a mark in our map that we know with precision where it's located) by using an observation model. In this case the model consists of euclidean distance and differential angle between the robot and the landmark. This is why we use ArUco landmarks, as these are easy to use and read with a camera. It is really important that sensing data for observation model should be completely independent from robot state. This because the model uses the state prediction to get observation model values, and the way it calculates the Kalman Gain for correction is by comparing it to a "ground truth" sensing, so these two should be independent. For example, another way of sensing is by using lidar sensor. The correction phase calculates tha gain and recalculates the robot state and covariance matrix, overwritting previous results at prediction. At the end, update this variables for next iteration.

The key factors for using EKF is the location of ArUco markers as the robot should have these correction references consistently for more precise location. 
