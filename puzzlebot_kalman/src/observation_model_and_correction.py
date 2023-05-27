#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion, PointStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty, EmptyResponse
from tf.transformations import euler_from_quaternion
import tf

class Observation_model():

    def __init__(self):
        rospy.init_node("observation_model")

        self.F = 60.0
        self.rate = rospy.Rate(self.F)
        self.T = float(1/self.F)
        self.mu_x = 0.0
        self.mu_y = 0.0
        self.mu_th = 0.0
        self.s_x = 0.0
        self.s_y = 0.0
        self.s_th = 0.0
        self.landmark_x = 0.0001
        self.landmark_y = 0.0001
        self.landmark_d = 0.0001
        self.zp_mu = 0.0
        self.zth_mu = 0.0
        self.zp_s = 0.0
        self.zth_s = 0.0
        self.R = np.array([[0.03, 0],[0.0, 0.02]])
        self.r = 0.05	# wheel radius
        self.v_k = 0.0
        self.w_k = 0.0
        self.sigma_k = np.zeros((3,3))
        self.odom_pub = rospy.Publisher('/kalman/corrected/odom', Odometry, queue_size=10)
        self.odom = Odometry()
        rospy.Subscriber('/kalman/mu', PoseStamped, self.mu_callback)
        rospy.Subscriber('/kalman/s', PoseStamped, self.s_callback)
        rospy.Subscriber('/kalman/landmark', PointStamped, self.landmark_callback)
        rospy.Subscriber('/kalman/odom',Odometry, self.odom_callback)
        rospy.Subscriber('/kalman/landmark/distance', Float32, self.d_callback)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel',Twist, self.cmd_vel_callback)
        
    def cmd_vel_callback(self,msg):
        self.v_k = msg.linear.x
        self.w_k = msg.angular.z
                
    def mu_callback(self,msg):
        self.mu_x = msg.pose.position.x
        self.mu_y = msg.pose.position.y
        self.mu_th = self.get_th(msg.pose.orientation)

    def s_callback(self,msg):
        self.s_x = msg.pose.position.x
        self.s_y = msg.pose.position.y
        self.s_th = self.get_th(msg.pose.orientation)
    
    def landmark_callback(self,msg):
        print(self.landmark_d)
        if self.landmark_d < 2 and self.landmark_d > 0.5:
            print(msg)
            self.landmark_x = msg.point.x
            self.landmark_y = msg.point.y

    def d_callback(self,msg):
        self.landmark_d = msg.data
    
    def odom_callback(self,msg):
        self.sigma_k[0][0] = msg.pose.covariance[0]  # xx
        self.sigma_k[1][0] = msg.pose.covariance[1] # xy
        self.sigma_k[2][0] = msg.pose.covariance[5] # xth
        self.sigma_k[0][1] = msg.pose.covariance[6] # yx
        self.sigma_k[1][1] = msg.pose.covariance[7] # yy
        self.sigma_k[2][1] = msg.pose.covariance[11] # yth
        self.sigma_k[0][2] = msg.pose.covariance[30]  # 
        self.sigma_k[1][2] = msg.pose.covariance[31]  # 
        self.sigma_k[2][2] = msg.pose.covariance[35]  # 


    def get_th(self,q):
        # get the yaw angle from quaternion
        Q = [0.0,0.0,0.0,0.0]
        Q[0] = q.x
        Q[1] = q.y
        Q[2] = q.z
        Q[3] = q.w
        (roll,pitch,yaw) = euler_from_quaternion(Q)
        return yaw
  
    def observation_model(self):
        
        Dx_mu = self.landmark_x - self.mu_x
        Dy_mu = self.landmark_y - self.mu_y
        Dx_s = self.landmark_x - self.s_x
        Dy_s = self.landmark_y - self.s_y
        p_mu = (Dx_mu**2) + (Dy_mu**2)
        # estimado sin ruido
        self.zp_mu = np.sqrt((Dx_mu**2) + (Dy_mu**2)) 
        self.zth_mu = np.arctan2(Dy_mu,Dx_mu) - self.mu_th
        # real, con ruido 
        self.zp_s = np.sqrt((Dx_s**2) + (Dy_s**2)) 
        self.zth_s = np.arctan2(Dy_s,Dx_s) - self.s_th

        # Jacobiano de la funcion del modelo de observacion

        Gk = np.array([[-Dx_mu/np.sqrt(p_mu), -Dy_mu/np.sqrt(p_mu), 0],
                       [Dy_mu/p_mu, -Dx_mu/p_mu, -1 ]])
        
        # Covarianza modelo observacion
        Z_k = np.dot(Gk, np.dot(self.sigma_k, Gk.T)) + self.R

        # Ganancia Kalman
        K_k = np.dot(self.sigma_k,np.dot(Gk.T, np.linalg.inv(Z_k)))

        #resultado con filtro de kalman
        mu_k_hat = np.array([[self.mu_x], [self.mu_y],[self.mu_th]])
        z_i_k = np.array([[self.zp_s],[self.zth_s]])
        z_i_k_hat = np.array([[self.zp_mu],[self.zth_mu]])

        # correction
        c_mu_k =  mu_k_hat + np.dot(K_k, (z_i_k - z_i_k_hat))
        I = np.eye(self.sigma_k.shape[0])
        c_sigma_k = np.dot((I - np.dot(K_k,Gk)),self.sigma_k )

        # create odom message
        Q = tf.transformations.quaternion_from_euler(0, 0, c_mu_k[2])

        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = 'world'
        self.odom.child_frame_id = 'base_link'
        self.odom.pose.pose.position = Point(c_mu_k[0,0],c_mu_k[1,0],self.r)
        self.odom.pose.pose.orientation = Quaternion(Q[0],Q[1],Q[2],Q[3])

        self.odom.pose.covariance = [0]*36
        self.odom.twist.twist.linear.x = self.v_k
        self.odom.twist.twist.linear.y = 0.0
        self.odom.twist.twist.linear.z = 0.0
        self.odom.twist.twist.angular.x = 0.0
        self.odom.twist.twist.angular.y = 0.0
        self.odom.twist.twist.angular.z = self.w_k
        self.odom.twist.covariance = [0]*36
        # sustituir 3x3 a 6x6 matriz covarianza
        self.odom.pose.covariance[0]  = c_sigma_k[0][0] # xx
        self.odom.pose.covariance[1]  = c_sigma_k[1][0] # xy
        self.odom.pose.covariance[5]  = c_sigma_k[2][0] # xth
        self.odom.pose.covariance[6]  = c_sigma_k[0][1] # yx
        self.odom.pose.covariance[7]  = c_sigma_k[1][1] # yy
        self.odom.pose.covariance[11] = c_sigma_k[2][1] # yth
        self.odom.pose.covariance[30] = c_sigma_k[0][2] # 
        self.odom.pose.covariance[31] = c_sigma_k[1][2] # 
        self.odom.pose.covariance[35] = c_sigma_k[2][2] # 

        self.odom.pose.covariance[14] = 0.0002 # agregar un grosor

        self.odom_pub.publish(self.odom)
        #print(self.odom)



    def main(self):
        while not rospy.is_shutdown():
            try:
                self.observation_model()
            except Exception as e :
                print(e)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        challenge = Observation_model()
        challenge.main()
        print("exiting main")

    except (rospy.ROSInterruptException, rospy.ROSException):
        print("topic was closed during publish")