#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
import numpy as np

class DeadReckoning():
	def __init__(self):
		rospy.init_node('dead_reckoning')
		self.odom_pub = rospy.Publisher('/odom',Odometry,queue_size=10)
		self.s_sub = rospy.Subscriber('/gazebo/model_states',ModelStates,self.gazebo_pose_callback)
		self.mu_sub = rospy.Subscriber('/pose_sim',PoseStamped,self.mu_callback)
		self.wr_sub = rospy.Subscriber('/rviz/wr',Float32,self.wr_callback)
		self.wl_sub = rospy.Subscriber('/rviz/wl',Float32,self.wl_callback)
		self.cmd_vel_sub = rospy.Subscriber('/cmd_vel',Twist, self.cmd_vel_callback)
		self.fs = 60.0		# rate frequency
		self.rate = rospy.Rate(self.fs)
		self.r = 0.05	# wheel radius
		self.l = 0.188	# distance between wheels
		self.wr = 0.0	# right wheel angular speed
		self.wl = 0.0	# left wheel angular speed
		self.v_k = 0.0	# linear velocity -> control signal
		self.w_k = 0.0	# angular velocity -> control signal
		self.s_x = 0.0	# gazebo pose x
		self.s_y = 0.0	# gazebo pose y
		self.s_q_k = Quaternion() # gazebo state orientation
		self.s_th_k_1 = 0.0		  # gazebo th
		self.mu_k = PoseStamped() # rviz state
		self.mu_k_1 = 0.0		  # rviz k-1 state
		self.dt = 1.0/self.fs		# delta time
		self.sigma_k = np.array([[0.0,0.0,0.0],
			   					 [0.0,0.0,0.0],
								 [0.0,0.0,0.0]]) # initial state has no covariance
		self.kr = 8	# gain right
		self.kl = 8	# gain left
		self.odom = Odometry()	# odom message
	
	def wr_callback(self,msg):
		self.wr = msg.data	# wr from rviz calc node as it's from the control unit
	
	def wl_callback(self,msg):
		self.wl = msg.data	# wl from rviz calc node as it's from the control unit

	def gazebo_pose_callback(self,msg):
		# in list [track,ground,puzzlebot]
		self.s_x = msg.pose[1].position.x
		self.s_y = msg.pose[1].position.y
		self.s_q_k = msg.pose[1].orientation
	
	def mu_callback(self,msg):
		self.mu_k = msg
	
	def cmd_vel_callback(self,msg):
		self.v_k = msg.linear.x
		self.w_k = msg.angular.z

	def dead_reckoning(self):
		# dt = rospy.Time.now().to_sec() - self.t0.to_sec()
		# print('dt',self.dt)

		# self.v_k = self.get_v(self.wr,self.wl,self.r)
		# self.w_k = self.get_w(self.wr,self.wl,self.r,self.l)
		mu_th_k_1 = self.get_th(self.mu_k_1.pose.orientation)
		H_k = np.array([[1, 0, -self.dt*self.v_k*np.sin(mu_th_k_1)],
		  				[0, 1,  self.dt*self.v_k*np.cos(mu_th_k_1)],
						[0, 0,  1]])
		# print('H_k',H_k)

		nabla_wk = (1.0/2.0)*self.r*self.dt*np.array([[np.cos(self.s_th_k_1), np.cos(self.s_th_k_1)],
				       						 [np.sin(self.s_th_k_1), np.sin(self.s_th_k_1)],
											 [2.0/self.l,				 -2.0/self.l]])
		# print('nabla_wk',nabla_wk)

		sigma_delta_k = np.array([[self.kr*np.abs(self.wr), 0],
			    				  [0,						self.kl*np.abs(self.wl)]])
		# print('sigma_delta_k',sigma_delta_k)

		# Q_k = np.linalg.multi_dot([nabla_wk,sigma_delta_k,nabla_wk.T])
		Q_k = np.dot(nabla_wk,np.dot(sigma_delta_k,nabla_wk.T))
		# print('Q_k:',Q_k)
		
		# self.sigma_k = np.linalg.multi_dot([H_k,self.sigma_k,H_k.T,]) + Q_k
		self.sigma_k = np.dot(H_k,np.dot(self.sigma_k,H_k.T)) + Q_k
		# print(self.sigma_k)

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

		self.odom_pub.publish(self.odom)
		print(self.odom)

		# actualizar variables k-1
		self.mu_k_1 = self.mu_k
		self.s_th_k_1 = self.get_th(self.s_q_k)

	# def get_v(self,wr,wl,r):
	# 	return r*( (wr+wl)/2 )

	# def get_w(self,wr,wl,r,l):
	# 	return r*( (wr-wl)/l )
	
	def get_th(self,q):
		Q = [0.0,0.0,0.0,0.0]
		Q[0] = q.x
		Q[1] = q.y
		Q[2] = q.z
		Q[3] = q.w
		(roll,pitch,yaw) = euler_from_quaternion(Q)
		return yaw
	
	def main(self):
		# run once to get k-1 values
		self.mu_k_1 = rospy.wait_for_message('/pose_sim',PoseStamped) # get mu_k-1
		ms = rospy.wait_for_message('/gazebo/model_states',ModelStates) # initial k-1 s state
		self.s_th_k_1 = self.get_th(ms.pose[-1].orientation) # initial s state th_k-1 
		while not rospy.is_shutdown():
			try:
				self.dead_reckoning() 
			except Exception as e:
				print(e)
			self.rate.sleep() 
					

if __name__ == '__main__':
	try:
		dr = DeadReckoning()
		dr.main()
	except (rospy.ROSInterruptException, rospy.ROSException):
		print("topic was closed during publish")
