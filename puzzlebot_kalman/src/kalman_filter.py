#!/usr/bin/env python
'''
Bruno Sanchez Garcia				A01378960
Carlos Antonio Pazos Reyes 			A01378262
Manuel Agustin Diaz Vivanco			A01379673

Nodo algoritmo filtro Kalman
'''

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Quaternion, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler

class KalmanFilter():
    def __init__(self):
        # node config
        rospy.init_node('kalman_filter')
        self.Fs = 60.0
        self.T = rospy.Time.now().to_sec()
        self.dt = 0.0
        self.rate = rospy.Rate(self.Fs)
        # subscribers
        rospy.Subscriber('/cmd_vel',Twist,self.cmd_callback)
        rospy.Subscriber('/wl',Float32,self.wl_callback)
        rospy.Subscriber('/wr',Float32,self.wr_callback)
        rospy.Subscriber('/kalman/landmark',PointStamped,self.landmark_callback)
        rospy.Subscriber('/kalman/landmark/distance',Float32,self.landmark_d_callback)
        rospy.Subscriber('/kalman/landmark/angle',Float32,self.landmark_a_callback)
        # publishers
        self.odom_pub = rospy.Publisher('/kalman/corrected/odom',Odometry,queue_size=10)
        # global variables - robot & simulation
        self.r = 0.05
        self.l = 0.188
        # global variables - mu
        self.v = 0.0
        self.w = 0.0
        self.x_k_mu = 0.0
        self.y_k_mu = 0.0
        self.th_k_mu = 0.0
        # global variables - s
        self.wl = 0.0
        self.wr = 0.0
        self.x_k_s = 0.0
        self.y_k_s = 0.0
        self.th_k_s = 0.0
        # global variables - motion model
        self.kl = 95.0
        self.kr = 95.0
        self.sigma_k = np.zeros((3,3))
        # global variables - observation model
        self.landmark_x = 0.0
        self.landmark_y = 0.0
        self.landmark_d = 0.0
        self.landmark_a = 0.0
        self.R = np.array([[0.03, 0   ],
                           [0,    0.03]])
    
    def cmd_callback(self,msg):
        # cmd_vel info
        self.v = msg.linear.x
        self.w = msg.angular.z
    
    def wl_callback(self,msg):
        # wl encoder callback
        self.wl = msg.data
    
    def wr_callback(self,msg):
        # wr encoder callback
        self.wr = msg.data
    
    def landmark_callback(self,msg):
        # aruco landmark point callback
        self.landmark_x = msg.point.x
        self.landmark_y = msg.point.y
        print(msg)
    
    def landmark_d_callback(self,msg):
        # aruco landmark distance callback
        print(msg)
        self.landmark_d = msg.data
    
    def landmark_a_callback(self,msg):
        # aruco landmark angle callback
        self.landmark_a = msg.data

    def filter(self):
        self.dt = rospy.Time.now().to_sec() - self.T
        if self.dt > 0.1:
            self.dt = 1.0/self.Fs

        # mu -> state without noise
        x_k1_mu = self.x_k_mu + self.dt*self.v*np.cos(self.th_k_mu)
        y_k1_mu = self.y_k_mu + self.dt*self.v*np.sin(self.th_k_mu)
        th_k1_mu = self.th_k_mu + self.dt*self.w
                                    
        
        # s -> state with noise
        v = ((self.wr + self.wl)/2) * self.r
        w = ((self.wr - self.wl)/ self.l) * self.r
        x_k1_s = self.x_k_s + self.dt*v*np.cos(self.th_k_s)
        y_k1_s = self.y_k_s + self.dt*v*np.sin(self.th_k_s)
        th_k1_s = self.th_k_s + self.dt*w
        
        # motion model - Kalman prediction
        H_k = np.array([[1, 0, -self.dt*self.v*np.sin(self.th_k_1_mu)],
                        [0, 1,  self.dt*self.v*np.cos(self.th_k_1_mu)],
                        [0, 0,  1]])
        nabla_wk = (1.0/2.0)*self.r*self.dt*np.array([[np.cos(self.th_k_1_s), np.cos(self.th_k_1_s)],
                                                      [np.sin(self.th_k_1_s), np.sin(self.th_k_1_s)],
                                                      [2.0/self.l,			  -2.0/self.l]])
        sigma_delta_k = np.array([[self.kr*np.abs(self.wr), 0                      ],
                                  [0,						self.kl*np.abs(self.wl)]])
        Q_k = np.dot(nabla_wk,np.dot(sigma_delta_k,nabla_wk.T))
        self.sigma_k = np.dot(H_k,np.dot(self.sigma_k,H_k.T)) + Q_k

        # check if landmark for Kalman correction - observation model
        if self.landmark_d > 0.0:   # distance to landmark is -1.0 when no landmark seen
            # for calculations
            Dx_mu = self.landmark_x - x_k1_mu
            Dy_mu = self.landmark_y - y_k1_mu
            Dx_s = self.landmark_x - x_k1_s
            Dy_s = self.landmark_y - y_k1_s
            p_mu = (Dx_mu**2) + (Dy_mu**2)
            # no noise
            zp_mu = np.sqrt(p_mu)
            zth_mu = np.arctan2(Dy_mu,Dx_mu) - th_k1_mu
            # with noise
            zp_s = np.sqrt((Dx_s**2) + (Dy_s**2)) 
            zth_s = np.arctan2(Dy_s,Dx_s) #- th_k1_s
            # Jacobian
            Gk = np.array([[-Dx_mu/np.sqrt(p_mu), -Dy_mu/np.sqrt(p_mu), 0],
                           [Dy_mu/p_mu,           -Dx_mu/p_mu,         -1]])
            # covariance
            Zk = np.dot(Gk, np.dot(self.sigma_k, Gk.T)) + self.R
            # Kalman Gain for correction
            Kk = np.dot(self.sigma_k,np.dot(Gk.T, np.linalg.inv(Zk)))
            # observation
            mu_k_hat = np.array([[x_k1_mu],
                                 [y_k1_mu],
                                 [th_k1_mu]])
            z_i_k = np.array([[self.landmark_d],
                              [self.landmark_a]])
            z_i_k_hat = np.array([[zp_mu],
                                  [zth_mu]])
            # correction
            c_mu_k =  mu_k_hat + np.dot(Kk, (z_i_k - z_i_k_hat))
            I = np.eye(self.sigma_k.shape[0])
            c_sigma_k = np.dot((I - np.dot(Kk,Gk)),self.sigma_k )
            # overwrite with corrected state and covariance
            x_k1_mu = c_mu_k[0][0]
            y_k1_mu = c_mu_k[1][0]
            th_k1_mu = c_mu_k[2][0]
            self.sigma_k = c_sigma_k

        # publish odometry
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x_k1_mu
        odom.pose.pose.position.y = y_k1_mu
        odom.pose.pose.position.z = self.r
        Q = quaternion_from_euler(0.0,0.0,th_k1_mu)
        odom.pose.pose.orientation = Quaternion(Q[0],Q[1],Q[2],Q[3])
        odom.pose.covariance = [0]*36
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.w
        odom.twist.covariance = [0]*36
        # sustituir 3x3 a 6x6 matriz covarianza
        odom.pose.covariance[0] = self.sigma_k[0][0] # x-x
        odom.pose.covariance[1] = self.sigma_k[1][0] # x-y
        odom.pose.covariance[5] = self.sigma_k[2][0] # x-th
        odom.pose.covariance[6] = self.sigma_k[0][1] # y-x
        odom.pose.covariance[7] = self.sigma_k[1][1] # y-y
        odom.pose.covariance[11] = self.sigma_k[2][1] # y-th
        odom.pose.covariance[30] = self.sigma_k[0][2] # th-x
        odom.pose.covariance[31] = self.sigma_k[1][2] # th-y
        odom.pose.covariance[35] = self.sigma_k[2][2] # th-th
        odom.pose.covariance[14] = 0.0002 # agregar un grosor
        self.odom_pub.publish(odom)
        
        # update variables for next iteration
        # motion model k-1 variables -> before updating to k iteration
        self.th_k_1_mu = self.th_k_mu
        self.th_k_1_s = self.th_k_s
        # mu
        self.x_k_mu = x_k1_mu
        self.y_k_mu = y_k1_mu
        self.th_k_mu = th_k1_mu
        # s
        self.x_k_s = x_k1_s
        self.y_k_s = y_k1_s
        self.th_k_s = th_k1_s
        #time
        self.T = rospy.Time.now().to_sec()
    
    def main(self):
        # main ROS node execution
        # initialize k-1 variables
        self.th_k_1_mu = self.th_k_mu
        self.th_k_1_s = self.th_k_s
        while not rospy.is_shutdown():
            try:
                self.filter()
            except Exception as e:
                print(e)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        KF = KalmanFilter()
        KF.main()
    except(rospy.ROSInternalException,rospy.ROSException):
        print('topic closed during publish')