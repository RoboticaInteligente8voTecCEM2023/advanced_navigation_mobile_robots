## Puzzlebot_sim

Contains relevant ROS node codes for running different navigation algorithms in puzzlebot in Gazebo and Rviz.

The relevnt node for the simulation is bug2.py which controls the robot for reactive navigation based on the bug2 algorithm.

### bug2 algorithm

Bug 2 Algorith, is an essential algorithm for reactive navigation in robotics as an standardized one. It is based on lidar sensor navigation. The algorithm is based on m-lines calculated and drawn between the origin of a goal point and another. The robot goes to goal point, follows wall around an obstacle when detected, and continues goint to goal when there is no obstacle ahead and robot crosses this reference line again.

Main logic of the algorithm is as follows:

![imagen](https://github.com/RoboticaInteligente8voTecCEM2023/advanced_navigation_mobile_robots/assets/67598380/26736770-7372-4ff5-9232-b74da355fc5f)

The robot goes to a point based on its location against the location of the goal point:

![imagen](https://github.com/RoboticaInteligente8voTecCEM2023/advanced_navigation_mobile_robots/assets/67598380/4fd51ba6-a49c-4618-b953-10b64e07f69c)

The robot calculates eculidean distance between the goal and its location, as well as the differential angle between them. The robot should rotate towards the alignment to goal and move towards it. So angular movement depends on differential angle between these two and linear movement can happen as shown above where speed gets slower as the robot gets closer to goal, or just keep constant linear speed.

The robot changes to follow wall when an object has been detected in front based on lidar information. When distance ahead is less than twice the distance the robot always has to be around any object, then it means there is an object, so follow the object using right or left hand rule. For right hand rule for example:

![imagen](https://github.com/RoboticaInteligente8voTecCEM2023/advanced_navigation_mobile_robots/assets/67598380/34df2715-113f-4465-aff3-1f8fb0559465)

Based on lidar information, the robot should always follow a wall at a certain distance. The controller takes lidar information at right side, and uses trigonometry to calculate the angle of the wall and robot to check if they are parallel to each other. If the angle is positive, it means the robot is facing the wall and should turn against it. Otherwise the robot is moving away from wall. At the same time, the robot should always be at a certain wall distance. This controller calculates the desired angular speed to align and move robot parallel to wall and always at certain distance, while keeping a linear speed.

Robot should transition back to going to point when distance to reference line, that should cross each obstacle, is less than a threshold.

Finally, robot reaches goal when euclidean distance between robot and goal is less than a threshold.

We understand that location of robot needs to be precise for the algorithm to work because it relies on robot location for going to goal but mostly to change between algorithm states.
