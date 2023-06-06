# advanced_navigation_mobile_robots
Advanced Navigation algorithms for Puzzlebot mobile robot

### Packages for running Extended Kalman Filter for Puzzlebot.
In order to run the simulation in Gazebo and Rviz, you have to set up a couple of things.


First make a workspace:
```
mkdir kalman_ws
cd kalman_ws
mkdir src
catkin_make
```
Then download or clone the packages into the workspace so that all packages are in the kalman_ws/src folder.

Go to https://github.com/ManchesterRoboticsLtd/TE3003B_Integration_of_Robotics_and_Intelligent_Systems.git and clone or download the packaged relative to the simulation. These are needed to run it. The particular packages are: 
- puzzlebot_control
- puzzlebot_gazebo

You can find these in https://github.com/ManchesterRoboticsLtd/TE3003B_Integration_of_Robotics_and_Intelligent_Systems/tree/main/Week%201/Challenge/Puzzlebot_Gazebo_Simulator, Week1/Challenge/Puzzlebot_Gazebo_Simulator.

You should have the next packges in kalman_ws/src directory:
- puzzlebot_control
- puzzlebot_gazebo
- puzzlebot_kalman
- puzzlebot_sim
- puzzlebot_world

You might also have an Aruco folder which is not a ROS package, so it's not needed, you can delete it or keep it. It contains useful codes for Aruco generation and detection, as well as other useful openCv code.

Once you have all your packages, go to your workspace directory adn build it again
```
cd kalman_ws
catkin_make
```

You may encounter a compilation error due to an specific line in CMake file in puzzlebot_kalman package. If the error says something similar as follows:
```
CMake Error at /opt/ros/melodic/share/catkin/cmake/catkin_package.cmake:305 (message):
  catkin_package() include dir 'include' does not exist relative to
  '/kalman_ws/src/puzzlebot_kalman'
Call Stack (most recent call first):
  /opt/ros/melodic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
  puzzlebot_kalman/CMakeLists.txt:115 (catkin_package)
```

To solve this issue, you must go to the line 116 in the /kalman_ws/src/puzzlebot_kalman/CMakeLists.txt file and delete the word 'include' in 'INCLUDE DIRS include'

Once you do it, retry catkin_make of the workspace.

If no errors encountered, you can proceed to run the simulation.

### Running Extended Kalman Filter Simulation
Open a terminal in your compliled workspace and run:
```
source devel/setup.bash
roslaunch puzzlebot_world aruco_world_MCR.launch
```
This will open Gazebo world with the Puzzebot ready to start.

In another terminal, run:
```
source devel/setup.bash
roslaunch puzzlebot_kalman rviz_set_up.launch
```
This will open Rviz visualization of the simulation.

Finally in another terminal run:
```
source devel/setup.bash
rosrun puzzlebot_sim bug2.py
```

This will start autonomous navigation for the robot.

Note: be sure that all fileshave execution permissions enabled. If not, open a terminal in those file and run:
```
sudo chmod +x [files]
```
And run it for all files that ask for the permission.

If you encounter any other issues please feel free to open an Issue thread, or contact use directly

### Results
You can find sample result in this link:
https://drive.google.com/file/d/1LXe50Rd2nwZUiwYz4AS2irvtAeMWoyaP/view?usp=sharing
