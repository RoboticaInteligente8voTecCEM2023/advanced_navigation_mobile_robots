# aruco detection with video
# https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/

import cv2
# aruco_dict must match generated aruco dict
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
# aruco parameters object
aruco_params = cv2.aruco.DetectorParameters_create()

# video constants
cam = 0
delay = 1
window_name = 'Aruco Detector'

# cv2 objects
cap = cv2.VideoCapture(cam)

# video capture loop until 'q' pressed
while True:
    # read frame from camera 0
    ret,frame = cap.read()
    # if receiving frame
    if ret:
        (corners,ids,rejected) = cv2.aruco.detectMarkers(frame,aruco_dict,parameters=aruco_params)
        # print('corners',corners)
        # print('ids',ids)
        # print('rejected',rejected)

        # check at least 1 detection
        if len(corners) > 0:
            # flatten markers ids [list]
            ids = ids.flatten()
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
                cv2.line(frame,topLeft,topRight,(0,255,0),2)
                cv2.line(frame,topRight,bottomRight, (0,255,0),2)
                cv2.line(frame,bottomRight,bottomLeft,(0,255,0),2)
                cv2.line(frame,bottomLeft,topLeft,(0,255,0),2)
                # draw centroid
                cX = int((topLeft[0]+bottomRight[0])/2.0)
                cY = int((topLeft[1]+bottomLeft[1])/2.0)
                cv2.circle(frame,(cX,cY),4,(0,0,255),-1)
                # draw aruco ID as text
                cv2.putText(frame,str(markerID),(topLeft[0],topLeft[1]-15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)

        # show result
        cv2.imshow(window_name,frame)

    # if 'q' -> exit video capture
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break

# destroy window to end program
cap.release()
cv2.destroyWindow(window_name)