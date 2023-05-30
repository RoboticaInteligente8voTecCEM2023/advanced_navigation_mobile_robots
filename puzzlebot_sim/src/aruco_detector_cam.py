#!/usr/bin/env python
'''
Bruno Sanchez Garcia				A01378960
Carlos Antonio Pazos Reyes 			A01378262
Manuel Agustin Diaz Vivanco		    A01379673

Nodo para detectar arucos y estimar distancia
euclidiana 
'''
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
import cv_bridge

class ArucoDetector():

    def __init__(self):
        # constructor node publishers and subscribers
        rospy.init_node("aruco_detector_camera")
        self.img_pub_output = rospy.Publisher("/processed_image/output",Image,queue_size=1)
        self.wp_pub = rospy.Publisher("/kalman/landmark",PointStamped,queue_size=1)
        self.dist_pub = rospy.Publisher('/kalman/landmark/distance',Float32,queue_size=1)
        self.ang_pub = rospy.Publisher('kalman/landmark/angle',Float32,queue_size=1)
        self.bridge = cv_bridge.CvBridge()
        
        rospy.Subscriber("/camera/image_raw",Image,self.imgCallback)
        rospy.Subscriber("/gazebo/model_states",ModelStates,self.gzMS_callback) # (simulation only)
        self.rate = rospy.Rate(60)
        self.frame = np.array([[]],dtype="uint8")
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50) # must match generated aruco dict
        self.aruco_params = cv2.aruco.DetectorParameters_create() # aruco parameters object
        self.waypoints = {1:Point(1,1,0),2:Point(2,2,0),3:Point(3,3,0),4:Point(4,4,0),5:Point(5,5,0)}
        self.gz_ms = ModelStates()
        self.markerSizecm = 33.0 # distnace in gazebo terms in [cm] -> in reality aprox to 11.91cm from 450px
        self.marker_points = np.array([[-self.markerSizecm / 2, self.markerSizecm / 2, 0],
                                       [self.markerSizecm / 2, self.markerSizecm / 2, 0],
                                       [self.markerSizecm / 2, -self.markerSizecm / 2, 0],
                                       [-self.markerSizecm / 2, -self.markerSizecm / 2, 0]], dtype=np.float32)
        # from callibration with charuco code
        # self.camMtx = np.array([[471.64867169,   0.        , 399.14780147],
        #                         [  0.        , 472.05840263, 398.37543982],
        #                         [  0.        ,   0.        ,   1.        ]])
        # self.distCoeff = np.array([[ 3.31727510e-03, -1.56623087e-02,  9.45674003e-05,
        #                             -4.47679541e-04,  1.55710703e-02]])
        ### from /camera/camera_info ###
    
        self.camMtx = np.array([[476.7030836014194,     0.0,                    400.5],
                                [0.0,                   476.7030836014194,      400.5],
                                [0.0,                   0.0,                    1.0  ]])
        self.distCoeff = np.array([[0.0,0.0,0.0,0.0,0.0]])
        

    def imgCallback(self,data):
        # callback for the img from camera
        try:
            frame = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
            self.frame = frame
        except cv_bridge.CvBridgeError():
            print("Error CvBridge")
    
    def gzMS_callback(self,msg):
        # callback for gazebo model states names (simulation only)
        self.gz_ms = msg

    def processImg(self):

        # copy of image subscriber
        frame = self.frame

        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        print("gray")

        (corners,ids,rejected) = cv2.aruco.detectMarkers(image=gray,
                                                         dictionary=self.aruco_dict,
                                                         parameters=self.aruco_params)
        # print('corners:',corners)
        # print('ids:',ids)
        # print('rejected:',rejected)

        # check at least 1 detection
        if len(corners) > 0:
            # flatten markers ids [list]
            ids = ids.flatten()
            # print('ids:',ids,'corners:',corners)
            # print('corners[0]',corners[0])
            # print(len(ids))
            nadas = []
            rvecs = []
            tvecs = []
            dists = []
            angles = []
            
            for i in range(len(ids)):
                
                # rvec,tvec = cv2.aruco.estimatePoseSingleMarkers(corners,self.markerSizecm,self.camMtx,self.distCoeff)
                n,r,t = cv2.solvePnP(self.marker_points,corners[i],self.camMtx,self.distCoeff,True)
        
                cv2.aruco.drawAxis(frame,self.camMtx,self.distCoeff,r,t,2)
                
                nadas.append(n)
                rvecs.append(r)
                tvecs.append(t)
                x = t[0][0] / 100.0  # [m]
                z = (t[2][0] + 10.0) / 100.0 # add 10cm for distance between camera and base_link frame [m]
                d = np.sqrt(x**2 + z**2)
                dists.append(d)
                a = np.arctan(z/-x)
                angles.append(a)
            print('rvec:',rvecs)
            print('tvec:',tvecs)
            print('dists:',dists)
            print('angles:',angles)

            # find min id in distance
            min_dist = min(dists)
            min_id = ids[dists.index(min_dist)]
            search = 'Aruco tag' + str(min_id)
            print(search)
            try:
                i = self.gz_ms.name.index(search)
            except:
                i = -1
            if i != -1:
                p = PointStamped()
                p.header.stamp = rospy.Time.now()
                p.header.frame_id = 'world'
                # from waypoints
                # p.point = self.waypoints[ids[0]]
                # from gazebo model (simulation only)
                p.point = self.gz_ms.pose[i].position
                if min_dist <= 2:
                    self.dist_pub.publish(min_dist)
                    self.ang_pub.publish(angles[min_id])
                    self.wp_pub.publish(p)
                    print(p)
            cv2.aruco.drawDetectedMarkers(frame,corners,ids)
        else:
            self.dist_pub.publish(-1.0)
        self.img_pub_output.publish(self.bridge.cv2_to_imgmsg(frame,"bgr8"))
	
    def main(self):
        # main execution while node runs
        print("Running main...")
        while not rospy.is_shutdown():
            try:
                self.processImg()
            except Exception as e:
                print("wait")
                print(e)
                print(type(e))
            self.rate.sleep()

if __name__ == "__main__":
	try:
		node = ArucoDetector()
		node.main()
	except (rospy.ROSInterruptException, rospy.ROSException):
		print('topic was closed during publish')
