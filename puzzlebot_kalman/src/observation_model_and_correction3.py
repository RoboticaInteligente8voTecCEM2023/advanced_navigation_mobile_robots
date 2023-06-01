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
        self.mu_k = 0.0
        self.s_x = 0.0
        self.s_y = 0.0
        self.s_th = 0.0
        self.x_k_mu = 0.0
        self.y_k_mu = 0.0
        self.th_k_mu = 0.0
        self.x_k_mu = 0.0
        self.y_k_mu = 0.0
        self.th_k_mu = 0.0
        self.x_k_s = 0.0
        self.y_k_s = 0.0
        self.th_k_s = 0.0
        self.s_th_k_1 = 0.0
        self.pose_stamped_mu = PoseStamped()
        self.pose_stamped_s = PoseStamped()
        self.landmark_x = 0.0001
        self.landmark_y = 0.0001
        self.landmark_d = 0.0001
        self.wr = 0.0	# right wheel angular speed
        self.wl = 0.0	# left wheel angular speed
        self.zp_mu = 0.0
        self.zth_mu = 0.0
        self.zp_s = 0.0
        self.zth_s = 0.0
        self.kr = 10	# gain right
        self.kl = 10	# gain left
        self.R = np.array([[0.02, 0],[0.0, 0.01]])
        self.r = 0.05	# wheel radius
        self.l = 0.188	# distance between wheels
        self.v_k = 0.0
        self.w_k = 0.0
        self.frame_id = 'base_link'
        self.T = rospy.Time.now().to_sec()		# delta time
        self.sigma_k = np.zeros((3,3))
        self.odom_pub = rospy.Publisher('/kalman/corrected/odom', Odometry, queue_size=10)
        self.odom = Odometry()
        rospy.Subscriber('/kalman/landmark', PointStamped, self.landmark_callback)
        self.odom_pub_c2 = rospy.Publisher('/kalman/odom',Odometry,queue_size=10)
        rospy.Subscriber('/kalman/landmark/distance', Float32, self.d_callback)
        rospy.Subscriber('/wr',Float32,self.wr_callback)
        rospy.Subscriber('/wl',Float32,self.wl_callback)
        rospy.Subscriber('/cmd_vel',Twist, self.cmd_vel_callback)
        self.odom_mu = rospy.Publisher('/kalman/mu', PoseStamped, queue_size=1)
        self.odom_s = rospy.Publisher('/kalman/s', PoseStamped, queue_size=1)
        self.Wl = rospy.Publisher('/kalman/mu/wl', Float32, queue_size=10)
        self.Wr = rospy.Publisher('/kalman/mu/wr', Float32, queue_size=10)
        
    def cmd_vel_callback(self,msg):
        self.v_k = msg.linear.x
        self.w_k = msg.angular.z
            


    def wr_callback(self,msg):
        self.wr = msg.data	# wr from rviz calc node as it's from the control unit

    def wl_callback(self,msg):
        self.wl = msg.data	# wl from rviz calc node as it's from the control unit

    
    def landmark_callback(self,msg):
        #print(self.landmark_d)
        #if self.landmark_d < 2 and self.landmark_d > 0.5:
            #print(msg)
            self.landmark_x = msg.point.x
            self.landmark_y = msg.point.y

    def d_callback(self,msg):
        self.landmark_d = msg.data
    

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
        dt = rospy.Time.now().to_sec() - self.T

        #mu
        x_k1_mu = self.x_k_mu + (self.T *(self.v_k))* np.cos(self.th_k_mu)
        y_k1_mu = self.y_k_mu + (self.T *(self.v_k))* np.sin(self.th_k_mu)
        th_k1_mu = self.th_k_mu + (self.T *(self.w_k))
        Q = tf.transformations.quaternion_from_euler(0, 0, th_k1_mu)
        time_stamp = rospy.Time.now()
        self.pose_stamped_mu.header.stamp = time_stamp
        self.pose_stamped_mu.header.frame_id = self.frame_id
        self.pose_stamped_mu.pose.position = Point(x_k1_mu, y_k1_mu, 0.0)
        self.pose_stamped_mu.pose.orientation = Quaternion(Q[0],Q[1],Q[2],Q[3])
        # print(self.pose_stamped)
        self.odom_mu.publish(self.pose_stamped_mu)
        wl_mu = (self.v_k - 0.5 * self.l * self.w_k)/self.r
        self.Wl.publish(wl_mu)
        wr_mu = (self.v_k + 0.5 * self.l * self.w_k)/self.r
        self.Wr.publish(wr_mu)
        

        self.x_k_mu = x_k1_mu
        self.y_k_mu = y_k1_mu
        self.th_k_mu = th_k1_mu

        self.mu_k = self.pose_stamped_mu
        self.mu_k_1 = self.mu_k
        self.mu_x = self.pose_stamped_mu.pose.position.x
        self.mu_y = self.pose_stamped_mu.pose.position.y
        self.mu_th = self.get_th(self.pose_stamped_mu.pose.orientation)

        #s
        V_s = ((self.wr + self.wl)/2) * self.r
        w_s = ((self.wr - self.wl)/ self.l) * self.r
        # alpha_k = np.random.normal(0.0, 0.5)
        x_k1_s = self.x_k_s + (self.T *(V_s)* np.cos(self.th_k_s))
        y_k1_s = self.y_k_s + (self.T *(V_s)* np.sin(self.th_k_s))
        th_k1_s = self.th_k_s + (self.T *(w_s))
        Q = tf.transformations.quaternion_from_euler(0, 0, th_k1_s)
        time_stamp = rospy.Time.now()
        self.pose_stamped_s.header.stamp = time_stamp
        self.pose_stamped_s.header.frame_id = self.frame_id
        self.pose_stamped_s.pose.position = Point(x_k1_s, y_k1_s, 0.0)
        self.pose_stamped_s.pose.orientation = Quaternion(Q[0],Q[1],Q[2],Q[3])
        # print(self.pose_stamped)
        self.odom_s.publish(self.pose_stamped_s)
        

        self.x_k_s = x_k1_s
        self.y_k_s = y_k1_s
        self.th_k_s = th_k1_s
        
        self.s_x = self.pose_stamped_s.pose.position.x
        self.s_y = self.pose_stamped_s.pose.position.y
        self.s_q_k = self.pose_stamped_s.pose.orientation
        self.s_th_k_1 = self.get_th(self.s_q_k)
        self.s_th = self.get_th(self.pose_stamped_s.pose.orientation)

        #############

        if dt > 0.1:
            dt = 1.0/self.F
        
        mu_th_k_1 = self.get_th(self.mu_k_1.pose.orientation)

        H_k = np.array([[1, 0, -dt*self.v_k*np.sin(mu_th_k_1)],
                        [0, 1,  dt*self.v_k*np.cos(mu_th_k_1)],
                        [0, 0,  1]])


        nabla_wk = (1.0/2.0)*self.r*dt*np.array([[np.cos(self.s_th_k_1), np.cos(self.s_th_k_1)],
                                                [np.sin(self.s_th_k_1), np.sin(self.s_th_k_1)],
                                                [2.0/self.l,				 -2.0/self.l]])


        sigma_delta_k = np.array([[self.kr*np.abs(self.wr), 0],
                                    [0,						self.kl*np.abs(self.wl)]])



        Q_k = np.dot(nabla_wk,np.dot(sigma_delta_k,nabla_wk.T))

        self.sigma_k = np.dot(H_k,np.dot(self.sigma_k,H_k.T)) + Q_k


        # create odom message
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = 'world'
        self.odom.child_frame_id = 'base_link'
        self.odom.pose.pose.position.x = self.mu_k.pose.position.x
        self.odom.pose.pose.position.y = self.mu_k.pose.position.y
        self.odom.pose.pose.position.z = self.r
        self.odom.pose.pose.orientation = self.mu_k.pose.orientation

        self.odom.pose.covariance = [0]*36
        self.odom.twist.twist.linear.x = self.v_k
        self.odom.twist.twist.linear.y = 0.0
        self.odom.twist.twist.linear.z = 0.0
        self.odom.twist.twist.angular.x = 0.0
        self.odom.twist.twist.angular.y = 0.0
        self.odom.twist.twist.angular.z = self.w_k
        self.odom.twist.covariance = [0]*36
        # sustituir 3x3 a 6x6 matriz covarianza
        self.odom.pose.covariance[0] = self.sigma_k[0][0] # xx
        self.odom.pose.covariance[1] = self.sigma_k[1][0] # xy
        self.odom.pose.covariance[5] = self.sigma_k[2][0] # xth
        self.odom.pose.covariance[6] = self.sigma_k[0][1] # yx
        self.odom.pose.covariance[7] = self.sigma_k[1][1] # yy
        self.odom.pose.covariance[11] = self.sigma_k[2][1] # yth
        self.odom.pose.covariance[30] = self.sigma_k[0][2] # 
        self.odom.pose.covariance[31] = self.sigma_k[1][2] # 
        self.odom.pose.covariance[35] = self.sigma_k[2][2] # 

        self.odom.pose.covariance[14] = 0.0002 # agregar un grosor

        self.odom_pub_c2.publish(self.odom)
    
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
        I = np.eye(self.sigma_k.shape[0])
        c_sigma_k = np.dot((I - np.dot(K_k,Gk)),self.sigma_k )
        c_mu_k =  mu_k_hat + np.dot(K_k, (z_i_k - z_i_k_hat))
        

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
        self.mu_k_1 = self.mu_k
        self.s_th_k_1 = self.get_th(self.s_q_k)
        self.T = rospy.Time.now().to_sec()



    def main(self):
        # run once to get k-1 values
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