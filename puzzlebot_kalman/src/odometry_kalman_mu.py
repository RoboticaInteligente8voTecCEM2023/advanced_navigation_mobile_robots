#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion, Point
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty, EmptyResponse
import tf

class Odometry_kalman():

    def __init__(self):
        rospy.init_node("odometry_mu")
        self.t0 = rospy.Time.now()
        self.x_k = 0
        self.y_k = 0
        self.th_k = 0.0
        self.V = 0.0
        self.w = 0.0
        self.tb = tf.TransformBroadcaster()
        self.F = 60.0
        self.rate = rospy.Rate(self.F)
        self.T = float(1/self.F)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.odom = rospy.Publisher('/kalman/mu', PoseStamped, queue_size=1)
        self.Wl = rospy.Publisher('/kalman/mu/wl', Float32, queue_size=10)
        self.Wr = rospy.Publisher('/kalman/mu/wr', Float32, queue_size=10)
        self.pJS = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.r = 0.05
        self.l = 0.188
        self.pose_stamped = PoseStamped()
        self.frame_id = 'base_link'
        self.js = JointState()
        self.js.name = ["base_to_right_w", "base_to_left_w"]


    def cmd_vel_callback(self,msg):
        self.V = msg.linear.x
        self.w = msg.angular.z

    def Odometry_mu(self):
        alpha_k = 0.0
        # alpha_k = np.random.normal(0.0, 0.5)
        x_k1 = self.x_k + (self.T *(self.V + alpha_k))* np.cos(self.th_k)
        y_k1 = self.y_k + (self.T *(self.V + alpha_k))* np.sin(self.th_k)
        th_k1 = self.th_k + (self.T *(self.w + alpha_k))
        Q = tf.transformations.quaternion_from_euler(0, 0, th_k1)
        time_stamp = rospy.Time.now()
        self.pose_stamped.header.stamp = time_stamp
        self.pose_stamped.header.frame_id = self.frame_id
        self.pose_stamped.pose.position = Point(x_k1, y_k1, 0.0)
        self.pose_stamped.pose.orientation = Quaternion(Q[0],Q[1],Q[2],Q[3])
        # print(self.pose_stamped)
        self.odom.publish(self.pose_stamped)
        wl = (self.V - 0.5 * self.l * self. w)/self.r
        self.Wl.publish(wl)
        wr = (self.V + 0.5 * self.l * self.w)/self.r
        self.Wr.publish(wr)
        
        t = time_stamp - self.t0
        self.js.position = [wr * t.to_sec(), wl * t.to_sec()]
        self.js.header.stamp = time_stamp
        self.pJS.publish(self.js)

        self.x_k = x_k1
        self.y_k = y_k1
        self.th_k = th_k1


        print(self.pose_stamped)


    def main(self):
        while not rospy.is_shutdown():
            try:
                self.Odometry_mu()
            except Exception as e :
                print(e)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        challenge = Odometry_kalman()
        challenge.main()
        print("exiting main")

    except (rospy.ROSInterruptException, rospy.ROSException):
        print("topic was closed during publish")
