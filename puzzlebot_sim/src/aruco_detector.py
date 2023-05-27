#!/usr/bin/env python
'''
Bruno Sanchez Garcia				A01378960
Carlos Antonio Pazos Reyes 			A01378262
Manuel Agustin Diaz Vivanco		    A01379673

Nodo para detectar arucos, localizacion y ids
'''
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
import cv_bridge

class ArucoDetector():

    def __init__(self):
        # constructor node publishers and subscribers
        rospy.init_node("sign_detector")
        self.img_pub_output = rospy.Publisher("/processed_image/output",Image,queue_size=1)
        self.wp_pub = rospy.Publisher("/kalman/landmark",PointStamped,queue_size=1)
        self.bridge = cv_bridge.CvBridge()
        rospy.Subscriber("/camera/image_raw",Image,self.imgCallback)
        rospy.Subscriber("/gazebo/model_states",ModelStates,self.gzMS_callback) # (simulation only)
        self.rate = rospy.Rate(60)
        self.frame = np.array([[]],dtype="uint8")
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50) # must match generated aruco dict
        self.aruco_params = cv2.aruco.DetectorParameters_create() # aruco parameters object
        self.waypoints = {1:Point(1,1,0),2:Point(2,2,0),3:Point(3,3,0),4:Point(4,4,0),5:Point(5,5,0)}
        self.gz_ms = ModelStates()
	
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

        (corners,ids,rejected) = cv2.aruco.detectMarkers(frame,self.aruco_dict,parameters=self.aruco_params)
        # print('corners',corners)
        # print('ids',ids)
        # print('rejected',rejected)

        # check at least 1 detection
        if len(corners) > 0:
            # flatten markers ids [list]
            ids = ids.flatten()
            if len(ids) == 1:
                search = 'Aruco tag' + str(ids[0]-1)
                print(search)
                print('id:%d'%ids[0])
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
                    self.wp_pub.publish(p)
                    print(p)
            # extract and draw
            for (markerCorner,markerID) in zip(corners,ids):
                # corners returned in the way:
                # top-left,top-right,bottom-right,bottom-left
                corners = markerCorner.reshape((4,2))
                (topLeft,topRight,bottomRight,bottomLeft) = corners
                # order corners (x,y) pairs per line
                topRight = (int(topRight[0]),int(topRight[1]))
                bottomRight = (int(bottomRight[0]),int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]),int(bottomLeft[1]))
                topLeft = (int(topLeft[0]),int(topLeft[1]))
                # draw enclosing rectangle
                cv2.line(frame,topLeft,topRight,(0,255,0),1)
                cv2.line(frame,topRight,bottomRight, (0,255,0),1)
                cv2.line(frame,bottomRight,bottomLeft,(0,255,0),1)
                cv2.line(frame,bottomLeft,topLeft,(0,255,0),1)
                # draw centroid
                cX = int((topLeft[0]+bottomRight[0])/2.0)
                cY = int((topLeft[1]+bottomLeft[1])/2.0)
                cv2.circle(frame,(cX,cY),4,(0,0,255),-1)
                # draw aruco ID as text
                cv2.putText(frame,str(markerID),(topLeft[0],topLeft[1]-15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)

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
            self.rate.sleep()
        # cv2.destroyAllWindows()

if __name__ == "__main__":
	try:
		node = ArucoDetector()
		node.main()
	except (rospy.ROSInterruptException, rospy.ROSException):
		print('topic was closed during publish')
