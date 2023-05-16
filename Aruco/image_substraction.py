# image subtract compares and detects when an object has been put on a surface
# based on an initial image taken at the beginning and the video frame
# Then processing to know when an object has been placed to be taken over and not first placed

# aruco package problems:
# pip3 uninstall opencv-python ## (with python3)
# pip3 install opencv-contrib-python ## (with python3) (cv2 with aruco lib included)
# https://stackoverflow.com/questions/45972357/python-opencv-aruco-no-module-named-cv2-aruco

import cv2
# video constants
cam = 0
delay = 1
window_name = 'Objects Detector (by subtraction)'

# cv2 Objects
cap = cv2.VideoCapture(cam)

# take picture of 1st frame
ret,first_frame = cap.read()
if ret:
    cv2.imwrite('Initial.jpg',first_frame)
else:
    exit('No initial camera frame')

# video capture loop until 'q' pressed
while True:
    # read frame from camera 0
    ret,frame = cap.read()
    # if receiving frame
    if ret:
        # show first frame
        cv2.imshow('First frame',first_frame)
        # show video
        cv2.imshow('Video frame',frame)

        # susbstract
        diff = cv2.subtract(first_frame,frame)
        # process noise
        gray = cv2.cvtColor(diff,cv2.COLOR_BGR2GRAY)
        # gray[gray > 0] = 1
        _,thresh = cv2.threshold(gray,10,255,cv2.THRESH_BINARY)
        blur = cv2.medianBlur(thresh,5)
        # show blur
        cv2.imshow(window_name,blur)
        # Morph ops
        erode = cv2.erode(blur,(3,3),iterations=2)
        #show morph ops
        cv2.imshow('Morph Ops',erode)


    # if 'q' -> exit video capture
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break

# destroy window to end program
cap.release()
cv2.destroyWindow(window_name)