import numpy as np
import cv2 as cv
import glob
from matplotlib import pyplot as plt

#The set of images that can be used for camera calibration. Use a glob pattern to create a list images of all the image (i.e., a list of strings).
images = sorted(glob.glob('*.jpg'))
print(images)

#Load an image from the previously constructed list of images.
img = cv.imread(images[0]) # Extract the first image as img
print(img.shape)
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) # Convert to a gray scale image
print(img.shape, gray.shape)
plt.figure(figsize=(20,20))
plt.subplot(231)
plt.imshow(img)
plt.title('Original Image')

# Extract the corners and the return value
retval, corners = cv.findChessboardCorners(gray, (9,6), None)
	# 'gray' is the input image
	# (8,6) is the patern size, corresponding to the interior corners to locate in the chessboard.
	#		In other words, it is the number of inner corners per a chessboard row and column
	#		(points_per_row, points_per_column)

print(corners)
print(corners.shape)
corners = np.squeeze(corners) # Get rid of extraneous singleton dimension
print(corners.shape)
print(corners[:5])  #Examine the first few rows of corners

img2 = np.copy(img)  # Make a copy of original img as img2

# Add circles to img2 at each corner identified
for corner in corners:
    coord = (corner[0], corner[1])
    cv.circle(img=img2, center=coord, radius=5, color=(255, 0, 0), thickness=2)

# Produce a figure with the original image img in one subplot and modified image img2 (with the corners added in).
plt.subplot(232)
plt.imshow(gray, cmap='gray') # Visualize the gray scale image
plt.title('Grayscale Image (sample of dataset)')
plt.subplot(233)
plt.imshow(img2)
plt.title('Original Image with Corners')

#The cornerSubPix function from OpenCV can be used to refine the corners extracted to sub-pixel accuracy. 
#This is based on an iterative technique; as such, one of the inputs criteria uses a tuple to bundle a convergence 
# tolerance and a maximum number of iterations.
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001) # Set termination criteria as a tuple.
corners_orig = corners.copy()  # Preserve the original corners for comparison after
corners = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria=criteria) # Extract refined corner coordinates.

# Examine how much the corners have shifted (in pixels)
shift = corners - corners_orig
print(shift[:4,:])
print(np.linalg.norm(shift.reshape(-1,1), np.inf))

img3 = np.copy(img)

for corner in corners:
    coord = (corner[0], corner[1])
    cv.circle(img=img3, center=coord, radius=5, color=(0, 255, 0), thickness=2)

plt.subplot(234)
plt.imshow(img2[200:400,200:500,:])
plt.title('Close-Up No. 1 (Original Corners)')
plt.subplot(235)
plt.imshow(img3[200:400,200:500,:])
plt.title('Close-Up No. 2 (Refined Corners)')


#The function drawChessboardCorners generates a new image with circles at the corners detected. The corners are 
# displayed either as red circles if the board was not found, or as colored corners connected with lines if the board 
# was found (as determined by the output argument retval from findChessboardCorners).
img4 = cv.drawChessboardCorners(img, (9,6), corners, retval)
plt.subplot(236)
plt.imshow(img4)
plt.title('Chessboard with colored corners connected with lines\n (the chessboard was found)')


#Finally, we are going to repeat this process with all the chessboard images to remove distortion effects. First, 
# assume a 3D world coordinate system aligned with the chessboard.
obj_grid = np.zeros((8*6,3), np.float32)
obj_grid[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)
print(obj_grid)

# Initialize empty list to accumulate coordinates
obj_points = [] # 3D World coordinates
img_points = [] # 2D Image coordinates

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

for fname in images:
    print('Loading {}'.format(fname))
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    retval, corners = cv.findChessboardCorners(gray, (9,6), None)

    if retval:
        obj_points.append(obj_grid)        
        corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        img_points.append(corners2)

#The accumulated lists of object coordinates and image coordinates can be combined to determine 
# an optimal set of camera calibration parameters. The relevant OpenCV utility here is calibrateCamera.

shape = gray.shape[::]
print(shape)
print(obj_points)
print(img_points)

retval, mtx, dist, rvecs, tvecs = cv.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
	#obj_points is a vector of vectors of calibration pattern points in the calibration pattern coordinate space.
	#img_points is a vector of vectors of the projections of calibration pattern points
	#gray.shape is the image used only to initialize the intrinsic camera matrix.

print('retval:',retval) # Objective function value
print('cam mtx:',mtx)    # Camera matrix
print('dist:',dist)   # Distortion coefficients

#The function getOptimalNewCameraMatrix can use the optimized matrix and distortion coefficients to 
# construct a new camera matrix appropriate for a given image. This can be used to remove distortion 
# effects with undistort.




######################################################################s
# #Optimization of a distorted image (radial distortion example No.1)
######################################################################
# img_d1 = cv.imread('original_a.png')
# h,w = img_d1.shape[:2]
# newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# #Undistort
# dst_1 = cv.undistort(img_d1, mtx, dist, None, newcameramtx) 

# #Crop the image
# x,y,w,h = roi
# dst_1 = dst_1[y:y+h, x:x+w] 

# plt.figure(figsize=(20,20))
# plt.subplot(221)
# plt.imshow(img_d1)
# plt.title('Original Distorted Image (Example No. 1)')
# plt.subplot(222)
# plt.imshow(dst_1)
# plt.title('Corrected Image (Example No. 1)');

# #Optimization of a distorted image (radial distortion example No.2)
# img_d2 = cv.imread('original_b.png')
# h,w = img_d2.shape[:2]
# newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# #Undistort
# dst_2 = cv.undistort(img_d2, mtx, dist, None, newcameramtx)

# #Crop the image
# x,y,w,h = roi
# dst_2 = dst_2[y:y+h, x:x+w]

# plt.subplot(223)
# plt.imshow(img_d2)
# plt.title('Original Distorted Image (Example No. 2)')
# plt.subplot(224)
# plt.imshow(dst_2)
# plt.title('Corrected Image (Example No. 2)');

# plt.show()






