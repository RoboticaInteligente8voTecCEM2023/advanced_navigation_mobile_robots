#!usr/bin/env python

import time
import cv2
import numpy as np
import pickle

# create dictionary and board objects
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
board = cv2.aruco.CharucoBoard_create(3,3,0.025,0.0125, dictionary)

# create board image to be used in calibration
image_board = board.draw((200*3,200*3))

# write calibration board image
cv2.imwrite('charuco.png',image_board)

# video
cap = cv2.VideoCapture(0)

all_corners = []
all_ids = []
counter = 0
while counter <= 300:
    ret,frame = cap.read()
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    
    res = cv2.aruco.detectMarkers(gray,dictionary) # 0->corners, 1->ids, 2->rejected
    if len(res[0]) > 0:
        res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board) # 0->corners 1-> ids 2->num
        if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3 and counter % 3 == 0:
            all_corners.append(res2[1])
            all_ids.append(res2[2])
            cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])
            counter += 1
            print(counter)
    
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# calibration
try:
    cal = cv2.aruco.calibrateCameraCharuco(all_corners,all_ids,board,gray.shape,None,None)
except:
    print('calibration failed...')
    exit(0)

# calibration results
retval,cameraMatrix,distCoeffs,rvecs,tvecs = cal
print('retval:',retval)
print('camMatrix:',cameraMatrix)
print('distCoeffs:',distCoeffs)
print('rvecs:',rvecs)
print('tvecs:',tvecs)
