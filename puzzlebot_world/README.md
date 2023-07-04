
All puzzlebot models belong to Manchester robotics, with main page link: https://manchester-robotics.com

### Puzzlebot_world

This package contains all files for running Gazebo simulation with Puzzlebot in it. 

There are several worlds and options to be edited for running the simulation.

For the particular solution, the important file is aruco_world_MCR.launch that loads aruco.world in Gazebo with the respective models.

You can run other worlds if desired by editing the world parameter in the launch file.

The other models are ArUco markers modeled for simulation and other useful models like the CharUco for camera calibration.

The world presented in Gazebo is:

![imagen](https://github.com/RoboticaInteligente8voTecCEM2023/advanced_navigation_mobile_robots/assets/67598380/435afe4a-a096-4be4-bd4a-69909e4bef93)

Parallel visualization in Rviz with trajectories:

![imagen](https://github.com/RoboticaInteligente8voTecCEM2023/advanced_navigation_mobile_robots/assets/67598380/7cfa00b6-9537-4e3c-afe3-02f9d9aa7e7b)

Where the pink balls are the marker detected by camera, and confidence-elipsoid is seen below the Kalman Filter odometry arrow in blue. The robot in red is the Gazebo parallel seen as the 'real robot'.
