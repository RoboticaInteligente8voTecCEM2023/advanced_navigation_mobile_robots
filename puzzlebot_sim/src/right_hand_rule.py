#!/usr/bin/env python
import sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RightHandRuleController:
    def __init__(self, wall_dist=0.5, w_max = np.pi, v_max=0.4 ):
        """
        Arguments
        --------------
        wall_dist  : float
           Desired distance to wall
        w_max  : float
           Max angular velocity
        v_max  : float
           Max linear velocity
        """
        self.scan_listener = rospy.Subscriber('/laser/scan', LaserScan,
                                              self.scan_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel' , Twist, queue_size=1 )
        self.rate = rospy.Rate(30.0)
        self.wall_dist = wall_dist
        self.w_max = w_max
        self.v_max = v_max
        self.scan = None
        self.k = 0.3

    def scan_callback(self, msg):
        """Called when a new scan is available from the lidar. """
        self.scan = msg

    def go_to_wall(self):
        """ Go straight ahead until at desired distance from a wall. """
        #--------------------------------------------------------------
        # Your code here
        #--------------------------------------------------------------
        #--------------------------------------------------------------

        pass

    def follow_right_hand_wall(self):
        found_wall = False
        while not rospy.is_shutdown():

            if self.scan is not None:
                #--------------------------------------------------------------
                # Your code here
                # 1) Check distance to wall/obstacle in front and to the right.
                # 2) Based on the information in 1) calculate a suitable angular
                #    velocity, w.
                # 3) Publish the Twist message to steer the puzzlebot.

                distance_ahead = get_distance_in_sector(self.scan,
                                                        0 - 2*np.pi/180,
                                                        0 + 2*np.pi/180)



                distance_to_right = get_distance_in_sector(self.scan,
                                                           -np.pi / 2,
                                                           -np.pi / 2 + 4 * np.pi / 180,
                                                           )

                #--------------------------------------------------------------
                #--------------------------------------------------------------
                error = find_wall_direction(self.scan)
                error_dist = self.wall_dist - distance_to_right
                v = self.v_max
                k_ang = 0.9
                k_dist = 0.75

                w =  (error* k_ang) + (error_dist * k_dist)
                if distance_ahead <= self.wall_dist * 2:
                    w = self.w_max

                print( "error: " + str("%.2f"%error), "distance to right: " +  "%.2f"%distance_to_right, "distance ahead " + "%.2f"%distance_ahead)
                msg = Twist()
                msg.angular.z =  np.clip(w, -self.w_max,self.w_max )
                msg.linear.x = v
                self.vel_pub.publish(msg)

            self.rate.sleep()



def find_wall_direction(scan):
    """Assuming wall is on the right, finds the direction of the wall w.r.t
    the robot frame (x ahead, y to the left). The direction is returned
    as an angle, which is 0 if the wall is parallel to the heading of the
    robot and negative if the robot is heading away from the wall.

    Tests
    -----
    >>> scan = generate_test_scan(straight_wall=True)
    >>> wall_dir = find_wall_direction(scan)
    >>> np.abs(wall_dir) < 1e-6
    >>> print find_wall_direction(scan)
    True
    """
    #--------------------------------------------------------------
    # Your code here
    start_angle = -np.pi/2
    end_angle = -np.pi/2 + np.pi/6
    alpha = abs(end_angle) - abs(start_angle) # usamos un sector de 30 grados
    start_index = range_index(scan,start_angle)
    end_index = range_index(scan,end_angle)

    a = scan.ranges[start_index]
    b = scan.ranges[end_index]
    c = np.sqrt(a**2 +b**2 - (2 * (a*b) * np.cos(abs(alpha))))
    angle = np.arcsin((a - b * np.cos(alpha)) / c) # sin(angle) = d/c

    return angle
    #--------------------------------------------------------------
    #--------------------------------------------------------------


def get_distance_in_sector(scan, start_angle, end_angle) :
    """Returns the distance in m in the given sector by taking the average of the
    range scans.

    Arguments
    ---------
    scan : LaserScan
    start_angle : float
        Start angle of sector, in radians, where 0 is straight ahead.
    end_angle : float
        End angle of sector, in radians, where 0 is straight ahead.

    Tests
    -----
    >>> scan = generate_test_scan()
    >>> start_angle = -np.pi/2
    >>> end_angle = start_angle + np.pi/180 # One degree sector
    >>> expected_distance = np.mean([0,0.1,0.2,0.3])
    >>> np.abs(get_distance_in_sector(scan, start_angle, end_angle) - expected_distance) < 1e-8
    True
    >>> end_angle = np.pi/2
    >>> start_angle = end_angle - 2*np.pi/180 # Two degree sector
    >>> expected_distance = np.mean(np.arange(720-8, 720)/10.0)
    >>> np.abs(get_distance_in_sector(scan, start_angle, end_angle) - expected_distance) < 1e-8
    True
    """
    num_scans = len(scan.ranges)

    #--------------------------------------------------------------
    # Your code here. For documentation of the LaserScan message:
    # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
    start_index = range_index(scan,start_angle)
    end_index = range_index(scan,end_angle)
    if end_index == num_scans-1:
        end_index = end_index + 1

    # 1) Find the indices into scan.ranges corresponding to the start_ange and end_angle
    # 2) Compute the average range in the sector using np.mean()
    #--------------------------------------------------------------
    return np.mean(scan.ranges[start_index:end_index])


def range_index(scan, angle):
    """Returns the index into the scan ranges that correspond to the angle given (in rad).
    If the angle is out of range, then simply the first (0) or last index is returned, no
    exception is raised.

    Arguments
    ---------
    scan : LaserScan
    angle : float
       Angle w.r.t the x-axis of the robot, which is straight ahead

    Tests
    -----
    >>> scan = generate_test_scan()
    >>> range_index(scan, -np.pi/2)
    0
    >>> range_index(scan, np.pi/2)
    719
    >>> range_index(scan, 0)
    360
    """

    #--------------------------------------------------------------
    # Your code here
    min_angle = scan.angle_min
    max_angle = scan.angle_max
    min_index = 0
    max_index = (len(scan.ranges) - 1)

    index = int(np.round((angle - min_angle) * (max_index - min_index) / (max_angle - min_angle) + min_index))

    return np.clip(index,0,720)
    #--------------------------------------------------------------
    #--------------------------------------------------------------



def generate_test_scan(straight_wall=False):
    """Function used for testing. Will create and return a LaserScan message"""
    scan = LaserScan()
    scan.angle_min = -np.pi/2
    scan.angle_max = np.pi/2
    num_scans = 720 # 4 lines per degree
    scan.angle_increment = np.pi/num_scans
    scan.range_min = 0.1
    scan.range_max = 30

    scan.ranges = np.arange(0, 720.0) / 10.0
    if straight_wall: # The wall is on the right at a distance of 10m
        scan.ranges[:320] = scan.range_max
        dth = np.arange(0, 320)*scan.angle_increment
        for i in range(320):
            scan.ranges[i] = 10/np.cos(dth[i])


    return scan

if __name__ == '__main__':
    if len(sys.argv) > 1:
        if sys.argv[1] == "--test":
            import doctest
            doctest.testmod()
            sys.exit(0)

    rospy.init_node('Follow_right_hand_wall')
    rhw = RightHandRuleController()
    rhw.follow_right_hand_wall()
